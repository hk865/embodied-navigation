# RoboGauge MoE-CTS portable deployment

The required source is in this repository, not imported or linked from a training project or old experiment directory. This is a simulation deployment, not a real-robot controller. RL training stays outside this project.

## Functional ownership

- `robot/unitree/robogauge`: model hash verification, named joint mapping, 45D observation, CPU TorchScript inference, Go2HV numerical contract and replay fixture.
- `simulation/gazebo/robogauge`: native request64/response16 socket loop, contact/command journaling and the complete Gazebo Go2HV C++ actuator.
- `navigation/robogauge`: fail-closed SLAM command admission and bounded Building benchmark orchestration.
- `configs/robogauge`: readable policy and benchmark contracts.

`configs/robogauge/contract.json` records the frozen scales, ordering, history, motor and timing assumptions. The policy consumes body angular velocity, projected gravity, body commands, joints and last unscaled action. It does not consume COM linear velocity, height scans or camera input. Privileged Gazebo pose/clearance remains diagnostic and safety evidence. No COM velocity PID is inserted. Gazebo keeps the archived command slew and immediate zero-command behavior; Isaac used direct commands and is not silently assigned the Gazebo slew.

## Data and weights

Set `PROJECT_DATA_ROOT` to your global external data directory. The shared project helper adds `embodied-navigation`; put the verified export at `embodied-navigation/runtime_assets/models/robogauge_policy.pt` below the global root. It must have SHA256 `bde5210a8114aa08bb0c611e49def1482faa1b2441673e5681374c30b935e32e`. Weights are excluded from Git. A data-only symlink to an existing copy is sufficient; no duplication is required. An explicit `--model` can select the same verified export elsewhere. Loading a different export is rejected, not silently accepted.

The optional CPU fixture needs public PyTorch. Base NumPy tests do not import PyTorch:

```sh
PYTHONPATH=src OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s tests/regression -p 'test_robogauge*.py' -v
PYTHONPATH=src python3 -m embodied_navigation.robot.unitree.robogauge.cpu_fixture
```

The fixture compares thirty stateful frames with an independent raw TorchScript instance and verifies reset. It is not a physics or navigation test.

## Native Gazebo actuator build

```sh
cmake -S src/embodied_navigation/simulation/gazebo/robogauge/motor_src -B build/robogauge-actuator -DCMAKE_INSTALL_PREFIX="$PWD/install/robogauge"
cmake --build build/robogauge-actuator --parallel 2
cmake --install build/robogauge-actuator
```

Public dependencies: C++17, `gz-sim8`, `gz-plugin2` (Gazebo Harmonic). Target `robogauge_actuator` installs `install/robogauge/lib/librobogauge_actuator.so`. It deliberately retains the `teacher_sim::TeacherActuator` plugin ABI name and `TEACHER_SOCKET`/`TEACHER_ACTUATOR_LOG` environment keys used by the archived request64 transport. These names do not select Teacher weights. The source is the actual November 8 Go2HV variant, not the different Student motor implementation.

## Historical results versus this port

These are archived measurements, not new runs of the portable project:

- Gazebo single-floor turn: real FAST-LIVO2/SCAN navigation, one run, 68.22 s, original parking test passed. Native goal XY error 0.01655 m; estimated SLAM XY error 0.01452 m.
- Gazebo short stairs: one real-SLAM run, 83.46 s, parking passed; native height gain 1.4928 m. Native goal XY error 0.06353 m differs from estimated SLAM XY error 0.04558 m.
- Isaac original office first flight: 12/12 physical stair levels and stable landing support in one run; actual rises 14.59–16.46 cm. **GT RouteTracker, not SLAM navigation**. No COM velocity PID. Arrival at 68.815 s and parking end at 73.815 s. Auxiliary ROS bag cleanup was incomplete; do not claim that bag is complete.
- Teacher 015/model1000 with its original trajectory PID: 12/12 levels, strict physical completion and parked; nonfoot contact observed, so clean gait was not certified.
- Teacher Rough002_v4/model12999: **the 6.015 s standing gate failed; the stairs test never started**. Speed v95 0.10946 m/s exceeded 0.02; angular rate v95 0.20273 rad/s exceeded 0.05. The guard was not weakened.

The Isaac live runner, stage/collision restoration, recorder, same-tick effort readback and Teacher-PID comparison are not advertised as portable runnable integration. Their archived runners depended on additional historical Isaac bootstraps and scene helpers. The shared, named 45D policy, Go2HV contract and complete native-pose RouteTracker (14 unchanged behavioral tests) are ported; an actual portable Isaac runner still requires a complete audited simulation source closure, the correct public Isaac installation/assets, and separate integration verification. No empty launcher is substituted.

No simulator, ROS node, GPU job or hardware was started during this source migration. Prior N=1 successes are not reliability statistics. Gazebo and Isaac physics are not claimed equivalent to the training environment.

## Provenance and rights

See `source_manifest.json` for original file hashes and migration mapping. The archive cites upstream model repository `wty-yy/go2_rl_gym_data`, revision `d1f19f0a486e171c327d8c3da2d31dba294e6586`, and author-source freeze `6c95d9feef4814c25eb6536e61bb38e1d1d718d4`. The original navigation checkout was `5688f2868e241ca4a7cc80ba9f166f8afa84c727`. These are recorded evidence, not a claim that the model has a verified training Git SHA: the archive explicitly says it does not.

`UPSTREAM_GO2_RL_LAB_LICENSE.txt` is the license file actually retained with the model archive (MIT notice plus downstream notices). Its presence alone does not establish model, dataset, asset or private deployment-source redistribution rights. No new license is invented or assigned to those files. The complete frozen author source was not present in the recovered model `source_contract` directory, so provenance is limited to recorded revisions, hashes, retained license and the actual deployment source.

## Portable benchmark entry

```sh
# CPU preparation only; writes inputs under the external project data root.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.benchmark --case single_floor_turn --label UNIQUE_NAME --prepare-only
# Missing data or unbuilt binaries are reported; this is not an execution test.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.benchmark --check
# Explicit ROS/Gazebo start, only after building dependencies and supplying assets.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.benchmark --case single_floor_turn --label ANOTHER_UNIQUE_NAME --run
# Offline same-time policy exchange, target and motor-envelope audit.
PYTHONPATH=src python3 -m embodied_navigation.navigation.robogauge.analyze "$RUN_DIRECTORY"
```

This entry preserves the real-SLAM process chain: CPU policy actor, sensor bridge, camera parameters, LiDAR filter, FAST-LIVO2, global backend, localization publication, existing Building navigation, SCAN and Gazebo. It requires the repository's own compiled libraries plus public ROS/Gazebo dependencies. `NAVIGATION_ACTOR_PYTHON` or `--actor-python` selects a Python with public CPU Torch; system ROS Python is used separately. The optional historical camera recorder/video encoder and resource monitor are not necessary navigation dependencies and are not started here.

Route JSON, world template and sensor configuration are real tracked files in `configs/robogauge`. Large meshes, point clouds, numeric mesh caches, models and logs remain outside Git. `BUILDING_SOURCE.json` and associated notices preserve the recorded Building/PCT provenance. The prepared process plan is not proof that any process ran. `analyze` will not pass empty logs or pre-send inference receipts without a matching actuator exchange and physics step.

`isaac_closure_audit.json` lists the actual recovered runner imports and missing historical integration edges. `configs/robogauge/isaac_first_flight_reference.json` contains only the relevant archived scenario values and explicitly does not advertise a live runner. The pure `RouteTracker` is GT diagnostic logic, separate from the real-SLAM benchmark and from the main relative-SLAM navigation task.

## Shared active simulation safety

Both active native backends now call `control/safety/simulation_gate.py`: the original Student 247D loop at `robot/unitree/policy/native/worker.py` and the RoboGauge 45D loop at `simulation/gazebo/robogauge/native_loop.py`. Simulation provides typed, explicitly marked native64 truth through `simulation/safety_state.py`. The shared `common/state_contract.py` forbids admitting that truth as a navigation/SLAM observation and rejects producer, run, clock-domain or timestamp mismatches.

The control gate owns the original fall/clearance decisions and fault latch. Policy faults precede native actuator faults, which precede the physical fall/clearance reason. The existing 1.5 s / 2 s grace times, .8 / 1.1 rad tilt thresholds, .15 / .10 m clearance thresholds and four-second persistent-fall window are unchanged. Existing faults remain latched even if a terrain query throws. Invalid provenance or nonfinite safety measurements fail closed. The original joint targets, observation dimensions, ordinary parking logic and native C++ protection remain in place.

CPU fixtures in `tests/regression/test_native_safety_integration.py` execute both loops with their real observation builders and a fake socket peer. They check that the shared gate is actually called every frame, target outputs agree for the same actor action, fault damping precedes termination at the original grace boundary, mismatched clocks stop the exchange, and a policy fault survives a later terrain exception. This does not claim a new simulator run or real-hardware support.
