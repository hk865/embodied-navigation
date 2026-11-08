"""Select original strict target-envelope protection before any zero handoff."""
import os
_OPTION=os.environ.get('R106_PREQUALIFICATION_SOURCE_HOLD','0')
if _OPTION not in ('0','1'):raise ValueError('R106_PREQUALIFICATION_SOURCE_HOLD must be 0 or 1')
PREQUALIFICATION_SOURCE_HOLD_ENABLED=_OPTION=='1'
UNSPECIFIED_ZERO_EVENT=object()
TARGET_ENVELOPE_REASONS=frozenset(('parking_fixed_target_lost','capture_fixed_goal_out_of_range'))

def eligible(owner,reason,terminal_zero_event):
    # An absent argument is unknown, not proof that no typed event exists.
    return bool(owner.prequalification_source_hold_enabled
        and reason in TARGET_ENVELOPE_REASONS and owner.enabled
        and owner.stage in ('capture','controlled_deceleration')
        and owner.failure is None and owner.diagnostic_business_failure_reason is None
        and owner.diagnostic_business_observation_count==0
        and terminal_zero_event is None
        and all(value is None for value in (owner.ready_stamp,owner.ready_clock,
            owner.handoff_clock,owner.first_zero_publication_clock,owner.first_actor_zero_ack)))
