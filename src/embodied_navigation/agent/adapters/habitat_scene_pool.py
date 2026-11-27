"""Actor-model Shared-Scene Habitat Host for true 3-tier decoupled concurrency.

Architecture:
1. Vision Server (Tier 1): single stateless GPU FastAPI service (SAM 3 + SigLIP).
2. Agent Sessions (Tier 2): N concurrent worker threads, each owning isolated
   ``PrefixStableContextManager``, ``HierarchicalMemoryStore``, ``GlobalRegionMemory``,
   ``TextureKeyframeSelector``, ``DepthOccupancyGrid``, lightweight ``VisionToolClient``,
   and failover LLM client.
3. Shared Simulation Host (Tier 3): ONE loaded Habitat-Sim scene instance bound to
   a dedicated EGL owner thread, multiplexing per-session robot pose slots via an
   atomic RPC queue.

Isolation Contract:
- Slot isolation covers robot **body pose (position + rotation)** in **static** HM3D
  scenes under the three fixed discrete body actions (``move_forward``, ``turn_left``,
  ``turn_right``). Dynamic objects, independent sensor mounts, and physics simulation
  time are NOT isolated per slot.
- Each ``create_session()`` call allocates a unique monotonic ``generation`` handle;
  duplicate active session IDs are rejected, and stale proxies after ``drop_session()``
  fail loudly without mutating any other active session's pose.
"""

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
import queue
import re
import threading
from typing import Any, Callable, Sequence, TypeVar

import numpy as np

from embodied_navigation.perception.vision_tools import VisionToolClient
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    HabitatObjectGT,
    HabitatSubtaskSpec,
    build_private_hm3d_gt,
)

T = TypeVar("T")

_SAFE_DIR_RE = re.compile(r"^[A-Za-z0-9._-]+$")


@dataclass
class _SessionPoseSlot:
    generation: int
    position: Any
    rotation: Any


def _clone_pose_value(val: Any) -> Any:
    if hasattr(val, "copy"):
        try:
            return val.copy()
        except Exception:
            pass
    if isinstance(val, np.ndarray):
        return val.copy()
    return val


def _clone_observations(obs: Any) -> Any:
    if isinstance(obs, dict):
        return {
            k: (v.copy() if isinstance(v, np.ndarray) else v)
            for k, v in obs.items()
        }
    return obs


@dataclass
class _PureQuat:
    w: float
    x: float
    y: float
    z: float


@dataclass
class _PureAgentState:
    position: np.ndarray
    rotation: Any


def _snapshot_agent_state(st: Any) -> _PureAgentState:
    """Copy a Habitat/pybind11 AgentState into pure-Python thread-safe objects."""
    pos = np.asarray(st.position, dtype=np.float32).copy()
    rot = st.rotation
    if hasattr(rot, "w") and hasattr(rot, "x") and hasattr(rot, "y") and hasattr(rot, "z"):
        rot_safe: Any = _PureQuat(float(rot.w), float(rot.x), float(rot.y), float(rot.z))
    else:
        rot_safe = _clone_pose_value(rot)
    return _PureAgentState(position=pos, rotation=rot_safe)


def _validate_session_output_dirs(
    out_root: Path,
    session_specs: Sequence[tuple[str, Sequence[HabitatSubtaskSpec], int]],
) -> list[tuple[str, Path, Sequence[HabitatSubtaskSpec], int]]:
    """Validate that every session name maps to a unique direct child directory of ``out_root``."""
    root_res = out_root.resolve()
    seen_names: set[str] = set()
    seen_dirs: set[Path] = set()
    validated: list[tuple[str, Path, Sequence[HabitatSubtaskSpec], int]] = []
    for raw_name, subs, seed in session_specs:
        name = str(raw_name).strip()
        if not name or name in (".", ".."):
            raise ValueError(f"Invalid session_name: {raw_name!r}")
        if "/" in name or "\\" in name or "\x00" in name:
            raise ValueError(f"Session name must not contain path separators: {raw_name!r}")
        if not _SAFE_DIR_RE.match(name):
            raise ValueError(f"Session name contains unsupported characters: {raw_name!r}")
        p = Path(name)
        if p.is_absolute() or len(p.parts) != 1:
            raise ValueError(f"Session name must be a single relative directory component: {raw_name!r}")
        cand = (root_res / name).resolve()
        if cand.parent != root_res:
            raise ValueError(f"Session output path escapes output_root: {raw_name!r}")
        if name in seen_names or cand in seen_dirs:
            raise ValueError(f"Duplicate session_name / output directory: {name!r}")
        seen_names.add(name)
        seen_dirs.add(cand)
        validated.append((name, cand, subs, int(seed)))
    return validated


class _ProxyAgentView:
    """Minimal ``sim.agents[0]`` proxy backed by ``HabitatSessionProxy.run_atomic``."""

    def __init__(self, proxy: HabitatSessionProxy) -> None:
        self._proxy = proxy

    def get_state(self) -> Any:
        return self._proxy.run_atomic(lambda sim: _snapshot_agent_state(sim.agents[0].get_state()))

    def set_state(self, state: Any) -> None:
        pos = np.asarray(state.position, dtype=np.float32).copy()
        rot = state.rotation

        def _apply(sim: Any) -> None:
            st = sim.agents[0].get_state()
            st.position = pos
            if isinstance(rot, _PureQuat):
                import quaternion

                st.rotation = quaternion.quaternion(rot.w, rot.x, rot.y, rot.z)
            else:
                st.rotation = _clone_pose_value(rot)
            sim.agents[0].set_state(st)

        self._proxy.run_atomic(_apply)


class HabitatSessionProxy:
    """Thread-safe per-session simulator facade backed by ``SharedHabitatSceneHost``.

    Compatible with ``HabitatWorldDaemon`` (``agents[0]``, ``get_sensor_observations``,
    ``step``) while exposing ``run_atomic(fn)`` for multi-step follower / NavMesh ops.
    Fixed proxy methods (``get_state``, ``get_sensor_observations``, ``step``) copy all
    native pybind11 buffers into thread-safe Python/NumPy objects before crossing threads.
    """

    def __init__(self, host: SharedHabitatSceneHost, session_id: str, *, generation: int) -> None:
        self._host = host
        self.session_id = str(session_id)
        self.generation = int(generation)
        self.agents = [_ProxyAgentView(self)]

    def run_atomic(self, fn: Callable[[Any], T]) -> T:
        return self._host.execute_for_session(
            self.session_id, fn, generation=self.generation
        )

    def get_sensor_observations(self) -> Any:
        return self.run_atomic(lambda sim: _clone_observations(sim.get_sensor_observations()))

    def step(self, action: str) -> Any:
        return self.run_atomic(lambda sim: _clone_observations(sim.step(action)))

    def drop(self) -> None:
        self._host.drop_session(self.session_id, generation=self.generation)


class SharedHabitatSceneHost:
    """Single-EGL-thread Habitat scene host serving multiple concurrent isolated sessions.

    Isolates robot **body pose** (position + rotation) across concurrent sessions on a
    static scene. Callers MUST NOT return raw pybind11 ``sim`` / ``agent`` / ``pathfinder``
    objects from ``run_atomic`` callbacks or invoke host RPCs re-entrantly from inside
    a callback.
    """

    def __init__(
        self,
        *,
        sim_factory: Callable[[], Any],
        target_categories: Sequence[str] = (),
        precompute_gt: bool = True,
        gt_num_samples: int = 260,
        gt_seed: int = 42,
        gt_max_view_dist_m: float = 1.8,
    ) -> None:
        self._sim_factory = sim_factory
        self._target_categories = tuple(
            sorted({str(c).strip().lower() for c in target_categories if str(c).strip()})
        )
        self._precompute_gt = bool(precompute_gt)
        self._gt_num_samples = int(gt_num_samples)
        self._gt_seed = int(gt_seed)
        self._gt_max_view_dist_m = float(gt_max_view_dist_m)
        self._cmd_q: queue.Queue[
            tuple[str | None, int | None, Callable[[Any], Any] | None, Future[Any]] | None
        ] = queue.Queue()
        self._slots: dict[str, _SessionPoseSlot] = {}
        self._next_generation: int = 0
        self._ready = threading.Event()
        self._startup_exc: BaseException | None = None
        self._closed = False
        self._close_lock = threading.Lock()
        self.private_gt: dict[str, list[HabitatObjectGT]] = {}
        self.gt_precomputed: bool = False
        self.gt_params: dict[str, Any] = {
            "num_samples": self._gt_num_samples,
            "seed": self._gt_seed,
            "max_view_dist_m": self._gt_max_view_dist_m,
            "categories": list(self._target_categories),
        }
        self.default_init_position: Any = None
        self.default_init_rotation: Any = None
        self.close_timed_out: bool = False
        self._owner_thread = threading.Thread(
            target=self._owner_loop, name="habitat-scene-owner", daemon=True
        )
        self._owner_thread.start()
        self._ready.wait()
        if self._startup_exc is not None:
            self._owner_thread.join(timeout=15.0)
            raise self._startup_exc

    def _assert_not_owner_thread(self) -> None:
        if threading.get_ident() == self._owner_thread.ident:
            raise RuntimeError("Re-entrant RPC from habitat-scene-owner thread is forbidden")

    def _owner_loop(self) -> None:
        sim: Any = None
        try:
            try:
                sim = self._sim_factory()
                st0 = sim.agents[0].get_state()
                self.default_init_position = _clone_pose_value(st0.position)
                self.default_init_rotation = _clone_pose_value(st0.rotation)
                if self._precompute_gt and self._target_categories:
                    self.private_gt = build_private_hm3d_gt(
                        sim,
                        target_categories=self._target_categories,
                        num_samples=self._gt_num_samples,
                        seed=self._gt_seed,
                        max_view_dist_m=self._gt_max_view_dist_m,
                    )
                    self.gt_precomputed = True
            except BaseException as exc:  # pragma: no cover - surfaced to caller via _startup_exc
                self._startup_exc = exc
                self._ready.set()
                return
            self._ready.set()

            while True:
                item = self._cmd_q.get()
                if item is None:
                    break
                session_id, expected_gen, fn, fut = item
                if fut.cancelled():
                    continue
                try:
                    if session_id is not None:
                        sid = str(session_id).strip()
                        slot = self._slots.get(sid)
                        if slot is None or (
                            expected_gen is not None and slot.generation != expected_gen
                        ):
                            raise RuntimeError(
                                f"stale_or_dropped_session: {sid!r} (expected_gen={expected_gen})"
                            )
                        st = sim.agents[0].get_state()
                        st.position = _clone_pose_value(slot.position)
                        st.rotation = _clone_pose_value(slot.rotation)
                        sim.agents[0].set_state(st)
                        try:
                            res = fn(sim) if fn is not None else None
                        finally:
                            # Always persist the simulator's real post-RPC pose back into the
                            # active slot so partial steps before an exception never diverge.
                            if sid in self._slots and self._slots[sid].generation == slot.generation:
                                st_after = sim.agents[0].get_state()
                                self._slots[sid] = _SessionPoseSlot(
                                    generation=slot.generation,
                                    position=_clone_pose_value(st_after.position),
                                    rotation=_clone_pose_value(st_after.rotation),
                                )
                    else:
                        res = fn(sim) if fn is not None else None
                    fut.set_result(res)
                except BaseException as exc:
                    fut.set_exception(exc)
        finally:
            if sim is not None and hasattr(sim, "close"):
                try:
                    sim.close()
                except Exception:
                    pass

    def create_session(
        self,
        session_id: str,
        *,
        position: Any = None,
        rotation: Any = None,
    ) -> HabitatSessionProxy:
        """Register an isolated robot pose slot on this shared scene host."""
        sid = str(session_id).strip()
        if not sid:
            raise ValueError("session_id must be non-empty")

        def _init_slot(sim: Any) -> int:
            if sid in self._slots:
                raise ValueError(f"Duplicate active session_id: {sid!r}")
            self._next_generation += 1
            gen = self._next_generation
            st = sim.agents[0].get_state()
            pos = _clone_pose_value(
                position if position is not None else self.default_init_position
            )
            rot = _clone_pose_value(
                rotation if rotation is not None else self.default_init_rotation
            )
            st.position = pos
            st.rotation = rot
            sim.agents[0].set_state(st)
            self._slots[sid] = _SessionPoseSlot(generation=gen, position=pos, rotation=rot)
            return gen

        gen = self.execute_raw(_init_slot)
        return HabitatSessionProxy(self, sid, generation=gen)

    def drop_session(self, session_id: str, *, generation: int | None = None) -> bool:
        """Unregister an active session slot. When ``generation`` is given, drops only if matching."""
        sid = str(session_id).strip()

        def _drop(_sim: Any) -> bool:
            slot = self._slots.get(sid)
            if slot is None:
                return False
            if generation is not None and slot.generation != int(generation):
                return False
            self._slots.pop(sid, None)
            return True

        return self.execute_raw(_drop)

    def execute_raw(self, fn: Callable[[Any], T]) -> T:
        self._assert_not_owner_thread()
        with self._close_lock:
            if self._closed:
                raise RuntimeError("SharedHabitatSceneHost is closed")
            fut: Future[T] = Future()
            self._cmd_q.put((None, None, fn, fut))
        return fut.result()

    def execute_for_session(
        self,
        session_id: str,
        fn: Callable[[Any], T],
        *,
        generation: int | None = None,
    ) -> T:
        self._assert_not_owner_thread()
        with self._close_lock:
            if self._closed:
                raise RuntimeError("SharedHabitatSceneHost is closed")
            fut: Future[T] = Future()
            self._cmd_q.put((str(session_id).strip(), generation, fn, fut))
        return fut.result()

    def close(self) -> None:
        self._assert_not_owner_thread()
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            self._cmd_q.put(None)
        self._owner_thread.join(timeout=15.0)
        if self._owner_thread.is_alive():
            self.close_timed_out = True
            raise TimeoutError("SharedHabitatSceneHost owner thread did not stop within 15s")

    def __enter__(self) -> SharedHabitatSceneHost:
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.close()


def run_concurrent_scene_sessions(
    *,
    scene_dataset_config: str | Path,
    scene_id: str | Path,
    output_root: str | Path,
    session_specs: Sequence[tuple[str, Sequence[HabitatSubtaskSpec], int]],
    chat_client: Any,
    vision_client: Any = None,
    vision_endpoint: str = "",
    vision_enhance_overlay: bool = True,
    vision_score_threshold: float = 0.25,
    vision_fuse_threshold: float = 0.35,
    max_workers: int = 3,
    stop_radius_m: float = 1.8,
    max_context_tokens: int = 200_000,
    soft_compaction_tokens: int = 155_000,
    enable_vlm_region_summary: bool = True,
    gpu_device_id: int = 0,
    gt_num_samples: int = 260,
    gt_seed: int = 42,
) -> dict[str, dict[str, Any]]:
    """Run multiple independent Agent sessions concurrently on ONE loaded Habitat scene.

    Each entry in ``session_specs`` is ``(session_name, subtasks, seed)`` and gets its own
    validated direct-child output directory, isolated memory/context, cloned LLM client
    session, lightweight ``VisionToolClient``, and robot pose slot.
    """
    from embodied_navigation.agent.adapters.habitat_runner import HabitatAgentRunner

    if not session_specs:
        return {}

    out_root = Path(output_root)
    out_root.mkdir(parents=True, exist_ok=True)
    validated_specs = _validate_session_output_dirs(out_root, session_specs)

    if len(validated_specs) > 1 and not hasattr(chat_client, "clone_for_session"):
        raise TypeError(
            "run_concurrent_scene_sessions with >1 session requires chat_client.clone_for_session()"
        )

    resolved_vision_endpoint = str(vision_endpoint or "").strip()
    if not resolved_vision_endpoint and vision_client is not None:
        resolved_vision_endpoint = str(getattr(vision_client, "endpoint", "") or "").strip()

    all_cats = tuple(
        sorted(
            {
                st.target_category.strip().lower()
                for _name, _dir, subs, _seed in validated_specs
                for st in subs
                if st.target_category.strip()
            }
        )
    )
    max_stop_r = max(
        (float(st.stop_radius_m) for _name, _dir, subs, _seed in validated_specs for st in subs),
        default=float(stop_radius_m),
    )

    probe_runner = HabitatAgentRunner(
        scene_dataset_config=scene_dataset_config,
        scene_id=scene_id,
        chat_client=chat_client,
        output_dir=out_root,
        stop_radius_m=stop_radius_m,
        vision_client=None,
        vision_enhance_overlay=vision_enhance_overlay,
        vision_score_threshold=vision_score_threshold,
        vision_fuse_threshold=vision_fuse_threshold,
        max_context_tokens=max_context_tokens,
        soft_compaction_tokens=soft_compaction_tokens,
        enable_vlm_region_summary=enable_vlm_region_summary,
        gpu_device_id=gpu_device_id,
        seed=gt_seed,
    )

    results: dict[str, dict[str, Any]] = {}
    with SharedHabitatSceneHost(
        sim_factory=probe_runner._create_sim,
        target_categories=all_cats,
        precompute_gt=True,
        gt_num_samples=gt_num_samples,
        gt_seed=gt_seed,
        gt_max_view_dist_m=max_stop_r,
    ) as host:

        def _run_one(
            name: str, sess_dir: Path, subs: Sequence[HabitatSubtaskSpec], seed: int
        ) -> tuple[str, dict[str, Any]]:
            sess_client = (
                chat_client.clone_for_session()
                if hasattr(chat_client, "clone_for_session")
                else chat_client
            )
            worker_vision = (
                VisionToolClient(resolved_vision_endpoint, timeout_s=10.0)
                if resolved_vision_endpoint
                else vision_client
            )
            sess_runner = HabitatAgentRunner(
                scene_dataset_config=scene_dataset_config,
                scene_id=scene_id,
                chat_client=sess_client,
                output_dir=sess_dir,
                stop_radius_m=stop_radius_m,
                vision_client=worker_vision,
                vision_enhance_overlay=vision_enhance_overlay,
                vision_score_threshold=vision_score_threshold,
                vision_fuse_threshold=vision_fuse_threshold,
                max_context_tokens=max_context_tokens,
                soft_compaction_tokens=soft_compaction_tokens,
                enable_vlm_region_summary=enable_vlm_region_summary,
                gpu_device_id=gpu_device_id,
                seed=seed,
            )
            rep = sess_runner.run_sequential_episode(
                subs,
                shared_scene_host=host,
                session_id=name,
                eval_mode_label="concurrent_independent",
            )
            return name, rep

        workers = max(1, min(int(max_workers), len(validated_specs)))
        with ThreadPoolExecutor(max_workers=workers) as pool:
            fut_to_meta = {
                pool.submit(_run_one, name, sess_dir, subs, seed): (name, seed)
                for name, sess_dir, subs, seed in validated_specs
            }
            for fut in as_completed(fut_to_meta):
                name, seed = fut_to_meta[fut]
                try:
                    out_name, rep = fut.result()
                    results[out_name] = rep
                except BaseException as exc:
                    results[name] = {
                        "status": "error",
                        "session_id": name,
                        "seed": seed,
                        "eval_mode": "concurrent_independent",
                        "sr": 0.0,
                        "spl": 0.0,
                        "false_stops": 0,
                        "wall_s": 0.0,
                        "runtime_metrics": {},
                        "error": f"{type(exc).__name__}: {exc}",
                    }
    return results


__all__ = [
    "HabitatSessionProxy",
    "SharedHabitatSceneHost",
    "run_concurrent_scene_sessions",
]
