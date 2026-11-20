"""Point-cloud SLAM backend for the FAST-LIVO2 keyframe spool (D path).

Keyframes, ScanContext place recognition, ICP, pose graph, map store and
map->odom correction. The frontend keeps its odometry; this package never
writes it. Only ``node`` imports ROS.
"""
