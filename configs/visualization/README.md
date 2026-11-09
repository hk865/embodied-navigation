# View a recorded DynaNav SLAM map

From the repository root, validate the estimated map without starting ROS:

```bash
MAP='/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-20261109-livo-dual-stairs/post-vio-static-map/static_map.npz'
python3 scripts/view_dynanav_map.py --map "$MAP" --check
```

Keep the publisher running in one terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=119  # Use the same isolated review domain in both terminals.
python3 scripts/view_dynanav_map.py --map "$MAP"
```

Open RViz in another terminal from the same repository root:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=119
rviz2 -d configs/visualization/dynanav_map.rviz
```

The publisher sends one PointCloud2 sample on `/dynanav/review/static_map` with reliable, transient-local, keep-last-depth-1 QoS. It stays alive so RViz can join later. Stop it with Ctrl-C, or supply a finite positive `--duration` in wall seconds. The viewer has no controller, TF publisher, simulator, arbitrary output-topic option, or GT pose reader. RViz includes viewing tools only.

`map_frame` comes directly from the NPZ. Use `--frame` only for an archive that has no frame metadata; an existing frame cannot be overridden. The static aggregate uses header stamp zero because the file does not supply one acquisition timestamp. XYZ values are encoded as float32 without filtering or downsampling; values that would become nonfinite during conversion are rejected.

This example contains 91,449 points in `dynanav_lio_map`. Its measured bounds are `[-20.855167337, -23.484348744, -0.180133181]` to `[10.089567833, 3.554424299, 6.216008119]` metres. The RViz Orbit view uses their centre `[-5.382799752, -9.964962222, 3.017937469]`, distance 52 m, Z-axis colouring and 0.03 m flat squares. For another map, set RViz's Fixed Frame to the frame reported by `--check`, then adjust the view to its reported bounds.

The viewer reports metadata actually present in the NPZ, such as `pose_source`, `mask_source` and `semantic_oracle`. This file has no separate `provenance` array. Displaying it does not establish complete spatial coverage or turn semantic-oracle labels into learned perception.

The later actual closed-loop run has a separate map containing **92,846 points**, also in `dynanav_lio_map`. To inspect that run, replace the historical `MAP` above with:

```bash
MAP='/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/post-vio-map/static_map.npz'
python3 scripts/view_dynanav_map.py --map "$MAP" --check
```

Then use the same isolated publisher/RViz commands. Its actual screenshot is [rviz-closed-loop-map.png](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/rviz-closed-loop-map.png), with [rviz-review-receipt.json](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/rviz-review-receipt.json) recording the map hash, display scope and visual check. The earlier 91,449-point example and screenshot remain historical artifacts from a different run. This newer map uses estimated poses, retains `sim_oracle` semantic masks, and does not certify complete scene coverage; the screenshot itself is display evidence, separate from the run's [closed-loop audit](/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-post-vio-final-capture-20261109T055946Z/audit/independent-review.json).

If RViz reports that its fixed frame is absent from the TF registry, the map can still render because its header and fixed frame are identical. Optionally register an identity **presentation axis** in the same isolated review domain, from a separate terminal:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=119
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id dynanav_lio_map --child-frame-id dynanav_review_axes
```

This only registers `dynanav_lio_map` and a child display axis for RViz. It is a mathematical identity for presentation, not a robot pose, localization transform, GT input, navigation result, or estimator evidence. Keep the RViz fixed frame as the map's actual `dynanav_lio_map`. The Python viewer itself continues to publish no TF or control messages. Stop only the review processes you started.

On this host a new review RViz window can use `DISPLAY=:0`, `XAUTHORITY=/run/user/1000/gdm/Xauthority`, and `XDG_RUNTIME_DIR=/run/user/1000`; no Xvfb is required. A validated review screenshot is at `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/dynanav-rviz-review-20261109T053029Z/rviz-map-registered-frame.png`. Its `review-receipt.json` records the temporary display-axis scope. That review used domain109; use your own matching isolated domain in the commands above. Restart the map publisher to repopulate a newly opened RViz after its prior finite lifetime has ended.
