"""One fixed safety handoff: ingress records; the default group owns control.

No FIFO, JSON copy, worker, timer, or lock around control/IO. A publication is
linearized by reserve() after its guard and before any publication mutation.
"""
import os
import threading
import time

KEY='BRIDGE_SAFETY_INGRESS'

def selected():
    value=os.environ.get(KEY,'0')
    if value not in ('0','1'):raise ValueError(KEY+' must be 0 or 1')
    return value=='1'

class Handoff:
    def __init__(self,node):
        self.node=node;self.lock=threading.Lock();self.latest=None;self.hold=None;self.failed=None
        self.terminal=False;self.last_state=None;self.sequence=0;self.scheduled=False
        self.servicing=False;self.closed=False;self.reserved=False;self.subscription=None
        self.counts=dict(received=0,hold_edges=0,hold_coalesced=0,failed_edges=0,
            services=0,negative_applied=0,ready_expired=0,reservations=0,
            cancelled_publications=0,negative_after_reservation=0,trigger_errors=0,owner_errors=0,closed_receives=0,shutdown_unapplied_negative=0)
        self.guard=node.create_guard_condition(self._execute,callback_group=node.default_callback_group)
        if self.guard.callback_group is not node.default_callback_group:
            raise RuntimeError('Bridge handoff must use the original control group')

    def bind_subscription(self,subscription):
        if subscription.callback_group is not self.node._r86_intake_group:
            raise RuntimeError('Bridge ingress subscription is not in the R86 intake group')
        self.subscription=subscription

    def _owner(self,node):
        if node is not self.node:raise RuntimeError('bridge_safety_handoff_wrong_owner')

    def _pending(self):
        return self.hold is not None or self.failed is not None or self.latest is not None

    def _notify(self):
        with self.lock:
            notify=not self.closed and self._pending() and not self.scheduled
            if notify:self.scheduled=True
        if notify:
            try:self.guard.trigger()
            except Exception as error:
                with self.lock:self.counts['trigger_errors']+=1
                self.node._r86_prepared_mailbox.fail('bridge_safety_notify:'+type(error).__name__)

    def receive(self,data,receipt):
        # data is the existing callback's parsed private dict; never mutated here.
        with self.lock:
            if self.closed:
                self.counts['closed_receives']+=1;return
            self.sequence+=1;self.counts['received']+=1
            record=(self.sequence,data,receipt);state=data['state']
            if state=='hold' and self.last_state!='hold':
                self.counts['hold_edges']+=1
                if self.reserved:self.counts['negative_after_reservation']+=1
                if not self.terminal:
                    if self.hold is None:self.hold=record
                    else:self.counts['hold_coalesced']+=1
            if state=='failed' and not self.terminal:
                self.terminal=True;self.failed=record;self.counts['failed_edges']+=1
                if self.reserved:self.counts['negative_after_reservation']+=1
            if not self.terminal or state=='failed' and self.failed is record:
                self.latest=record
            self.last_state=state
        self._notify()

    def _take(self,negative_only=False):
        with self.lock:
            self.scheduled=False
            events=sorted([x for x in (self.hold,self.failed)if x is not None],key=lambda x:x[0])
            self.hold=self.failed=None
            if events:
                if self.latest is not None and self.latest[0]<=events[-1][0]:self.latest=None
                return events,True
            if negative_only or self.terminal:return [],False
            value=self.latest;self.latest=None
            return ([]if value is None else [value]),False

    def service(self,node,negative_only=False):
        self._owner(node)
        if self.servicing or self.closed:return False
        events,negative=self._take(negative_only)
        if not events:return False
        self.servicing=True
        try:
            with self.lock:self.counts['services']+=1
            for _,data,receipt in events:
                now=time.monotonic()
                if not negative and data['state']=='ready':
                    # READY cannot gain a lease by waiting in the handoff.
                    if not (0<=now-receipt<=.3 and -.05<=now-float(data['monotonic_wall'])<.3):
                        with self.lock:self.counts['ready_expired']+=1
                        continue
                if negative:
                    # This is an accepted bridge protection event, even when
                    # the previous velocity was already zero. Reuse the original
                    # ordinary owner guard; never alter its thresholds or T0.
                    core=getattr(node,'cascade',None)
                    ordinary=getattr(core,'ordinary_parking',None)
                    terminal=getattr(node,'terminal_parking_owner',None)
                    if getattr(node,'terminal_parking_request',None)!=getattr(node,'request_id',None):terminal=None
                    if ordinary is not None:ordinary.guard('execution_bridge_protection')
                    if terminal is not None and terminal is not ordinary:terminal.guard('execution_bridge_protection')
                node._accept_bridge_safety(data,receipt,force_protection=negative,service_wall=now)
                if negative:
                    with self.lock:self.counts['negative_applied']+=1
        except Exception as error:
            with self.lock:self.counts['owner_errors']+=1
            node._r86_prepared_mailbox.fail('bridge_safety_owner:'+type(error).__name__)
            raise
        finally:
            self.servicing=False
            self._notify()
        return negative

    def _execute(self):
        self.service(self.node)

    def reserve(self,node):
        self._owner(node)
        with self.lock:
            if self.closed or self.terminal or self.hold is not None or self.failed is not None:return False
            self.reserved=True;self.counts['reservations']+=1
            return True

    def end_reservation(self,node):
        self._owner(node)
        with self.lock:self.reserved=False

    def cancel_publication(self,node):
        self._owner(node)
        with self.lock:self.counts['cancelled_publications']+=1
        row=getattr(node,'pid_row',None)
        if row is not None:row['bridge_safety_publication_cancelled']=True
        before=node.counts['commands']
        self.service(node,negative_only=True)
        # Original None path must run even if the old command was already zero:
        # ordinary parking/ACK/T0 protection is a business event, not a speed test.
        if node.counts['commands']==before:return node.publish_command()
        return None

    def close(self,node,*,service_negative=True):
        self._owner(node)
        if self.closed:return
        # Shutdown owner has joined executor workers. Retain pending negatives
        # through their original protection path; never apply a pending READY.
        if service_negative:self.service(node,negative_only=True)
        with self.lock:
            if not service_negative:
                self.counts['shutdown_unapplied_negative']=sum(x is not None for x in (self.hold,self.failed))
            self.closed=True;self.latest=None;self.scheduled=False
            if service_negative:self.hold=self.failed=None

    def snapshot(self):
        with self.lock:
            return dict(schema='bridge_safety_handoff/v1',enabled=True,closed=self.closed,
                same_control_group=self.guard.callback_group is self.node.default_callback_group,
                ingress_group_is_r86=(self.subscription is not None and
                    self.subscription.callback_group is self.node._r86_intake_group),
                ingress_callback_bound_to_node=(self.subscription is not None and
                    getattr(self.subscription.callback,'__self__',None) is self.node),counts=dict(self.counts),
                pending_latest=self.latest is not None,pending_hold=self.hold is not None,
                pending_failed=self.failed is not None,terminal_failed_latched=self.terminal,
                slots=3,source_and_callback_timeout_s=.3,reservation_before_publication_mutation=True,
                lock_contains_control_or_IO=False)
