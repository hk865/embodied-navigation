"""Join original SCAN startup predicates inside its existing five-second budget."""

import hashlib
import os
from pathlib import Path
import time
from embodied_navigation.planning.scan_child_consumption import (
    verify_owned_child,
    verify_startup_command,
)

MAX_ARGV_BYTES = 65536
SAMPLE_LIMIT = 8
SAMPLE_BYTES = 4096


def await_startup(
    child,
    binary,
    digest,
    supervisor_pid,
    nonce,
    expected_command,
    require_reference,
    deadline,
    identity,
    *,
    record,
    provisional,
    monotonic=time.monotonic,
    pause=time.sleep,
    read_argv=None,
    hash_exe=None,
):
    """Never treat intended argv or provisional ownership as startup acceptance."""
    binary = Path(binary).resolve()
    expected_raw = b"\0".join(os.fsencode(x) for x in expected_command) + b"\0"
    if len(expected_raw) > MAX_ARGV_BYTES:
        raise ValueError("Expected SCAN startup argv exceeds finite evidence bound")
    if read_argv is None:

        def read_argv():
            with (Path("/proc") / str(child.pid) / "cmdline").open("rb") as stream:
                return stream.read(MAX_ARGV_BYTES + 1)

    if hash_exe is None:

        def hash_exe():
            h = hashlib.sha256()
            with (Path("/proc") / str(child.pid) / "exe").open("rb") as stream:
                for block in iter(lambda: stream.read(1024 * 1024), b""):
                    h.update(block)
            return h.hexdigest()

    samples = []
    count = 0
    saved = None
    last_reason = None
    proof = dict(
        schema="r111_scan_startup_joint_identity/v1",
        pid=child.pid,
        expected_nonce=nonce,
        expected_command=list(expected_command),
        deadline_monotonic=deadline,
        started_monotonic=monotonic(),
        original_predicates_preserved=True,
        full_argv_equality_required=False,
    )

    def sample(reason, current, raw=b""):
        """Append one bounded startup-identity sample, keeping the newest row.

        The first ``SAMPLE_LIMIT - 1`` rows plus the newest row are kept; once
        full, the final row is replaced so a long wait cannot grow the evidence.
        """
        nonlocal count, last_reason
        count += 1
        last_reason = reason
        row = dict(
            sequence=count,
            monotonic=monotonic(),
            reason=reason,
            identity=current,
            actual_argv_bytes=len(raw),
            actual_argv_sha256=hashlib.sha256(raw).hexdigest(),
            actual_argv_prefix_hex=raw[:SAMPLE_BYTES].hex(),
            actual_argv_truncated=len(raw) > SAMPLE_BYTES,
        )
        if len(samples) < SAMPLE_LIMIT:
            samples.append(row)
        else:
            samples[-1] = row

    def verify_argv(raw, current):
        """Parse the live argv and re-run the original startup predicates.

        The raw bytes are bounded and split on NUL; the exact nonce/flag pair
        must be present and ``verify_startup_command`` must still accept it.
        """
        if len(raw) > MAX_ARGV_BYTES:
            raise ValueError("Actual SCAN argv exceeds finite read bound")
        command = raw.decode().strip("\0").split("\0")
        verify_startup_command(saved, current, binary, supervisor_pid, nonce, command)
        expected_flags = {
            "fsm.child_odometry_consumption_ack:=": "true",
            "fsm.child_odometry_ack_nonce:=": nonce,
        }
        if require_reference:
            expected_flags["fsm.reference_protocol_ack:="] = "true"
        for prefix, value in expected_flags.items():
            if [token for token in command if token.startswith(prefix)] != [prefix + value]:
                raise ValueError("Conflicting or duplicate actual SCAN ACK argument: " + prefix)
        if require_reference and command.count("fsm.reference_protocol_ack:=true") != 1:
            raise ValueError("Actual owned child argv lacks strict reference commit ACK parameter")

    try:
        while monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError("SCAN exited during owned bootstrap")
            current = identity(child.pid)
            if current is None or Path(current.get("exe", "")).resolve() != binary:
                if saved is not None:
                    raise ValueError("Owned SCAN identity changed during bootstrap")
                sample("awaiting_original_executable_identity", current)
            else:
                if current.get("pid") != child.pid or current.get("ppid") != supervisor_pid:
                    sample("wrong_owned_parent_or_pid", current)
                    raise ValueError("SCAN child parent differs from this supervisor")
                if saved is None:
                    saved = dict(current)
                    verify_owned_child(saved, current, binary)
                    provisional(dict(saved))
                else:
                    if any(
                        current.get(k) != saved.get(k)
                        for k in ("pid", "ppid", "pgid", "start_ticks", "exe")
                    ):
                        sample("owned_identity_changed", current)
                    verify_owned_child(saved, current, binary)
                raw = read_argv()
                after = identity(child.pid)
                if after is None or any(
                    after.get(k) != saved.get(k)
                    for k in ("pid", "ppid", "pgid", "start_ticks", "exe")
                ):
                    sample("identity_changed_after_argv_read", after, raw)
                verify_owned_child(saved, after, binary)
                try:
                    verify_argv(raw, after)
                except (ValueError, UnicodeError):
                    sample("argv_predicate_not_satisfied", after, raw)
                    # Only an unpopulated /proc image can wait. Explicit wrong
                    # or duplicate flags/nonce remain immediate failures.
                    if raw and not (len(raw) < len(expected_raw) and expected_raw.startswith(raw)):
                        raise
                else:
                    sample("original_argv_predicates_satisfied", after, raw)
                    final_raw = read_argv()
                    final_identity = identity(child.pid)
                    sample("final_identity_argv_verification", final_identity, final_raw)
                    verify_owned_child(saved, final_identity, binary)
                    verify_argv(final_raw, final_identity)
                    if child.poll() is not None:
                        raise RuntimeError("SCAN exited during owned bootstrap")
                    if monotonic() >= deadline:
                        raise RuntimeError("Original SCAN bootstrap deadline exhausted")
                    proof.update(
                        accepted=True,
                        identity=dict(saved),
                        completed_monotonic=monotonic(),
                        sample_count=count,
                        samples=samples,
                    )
                    record(proof)
                    return saved
            remaining = deadline - monotonic()
            if remaining > 0:
                pause(min(0.01, remaining))
        raise RuntimeError("Original SCAN bootstrap deadline exhausted: " + str(last_reason))
    except Exception as error:
        proof.update(
            accepted=False,
            error=type(error).__name__ + ":" + str(error),
            provisional_identity=saved,
            completed_monotonic=monotonic(),
            sample_count=count,
            samples=samples,
        )
        record(proof)
        raise
