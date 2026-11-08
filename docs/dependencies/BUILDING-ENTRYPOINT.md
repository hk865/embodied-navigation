# Building entrypoint closure

The production Building launcher and simulation adapter accept only `slam` / `fast_livo2_async`. Unsupported truth or capture-only selection is rejected before assets, run files, or the Student model are opened. The default mode is SLAM.

Some inherited module, topic and output filenames contain `truth`. These names are compatibility labels, not permission to use simulated pose as navigation localization. Privileged native state remains available for simulation safety, policy input and offline scoring only.

The existing StudentRuntime consumes its original 45-dimensional input. The native legacy observation builder still constructs 247 dimensions before adaptation. No RoboGauge policy, model or control formula was substituted.

`evaluation/process_resources.py` contains the original one-second `/proc` CPU/RSS accounting loop, now behind a `main()` entry so importing it has no side effects. Its source commit, byte count and hashes are recorded in `building-source-closure.json`. It is not a navigation or safety authority. SCAN parameters resolve to the real vendored `third_party/scan/plan_manage/config/planner.yaml`.

Validation: 48 CPU unit tests passed, including seven Building entry/import/prepare tests; an actual `--prepare-only --case stairs_short` produced SLAM-only configuration. This Building path has not been launched or validated for navigation. The separately tested default 45-second Student navigation chain is unchanged.
