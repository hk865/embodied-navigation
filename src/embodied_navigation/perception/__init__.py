"""Image/depth processing and Layer-2 vision tool clients (zero PyTorch dependency)."""

from embodied_navigation.perception.vision_tools import (
    DEFAULT_CONTEXT_PROMPTS,
    GroundedDetection,
    VisionDetection,
    VisionToolClient,
    build_vision_prompts,
    cluster_multilabel_detections,
    decode_mask_b64,
    encode_mask_b64,
    ground_detections_with_depth,
    ground_mask_with_depth,
    render_visual_enhancement_overlay,
    suppress_overlapping_detections,
)

__all__ = [
    "DEFAULT_CONTEXT_PROMPTS",
    "GroundedDetection",
    "VisionDetection",
    "VisionToolClient",
    "build_vision_prompts",
    "cluster_multilabel_detections",
    "decode_mask_b64",
    "encode_mask_b64",
    "ground_detections_with_depth",
    "ground_mask_with_depth",
    "render_visual_enhancement_overlay",
    "suppress_overlapping_detections",
]
