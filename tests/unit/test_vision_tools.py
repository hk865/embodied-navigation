"""Unit tests for Layer-2 vision tools, mask-depth 3D grounding, overlays, and HTTP client."""

from __future__ import annotations

import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import tempfile
import threading
import unittest
import zlib

import numpy as np
from PIL import Image

from embodied_navigation.localization_mapping.semantic_mapping import (
    HierarchicalMemoryStore,
    KeyframeSceneGraph,
)
from embodied_navigation.perception.vision_tools import (
    GroundedDetection,
    VisionDetection,
    VisionToolClient,
    build_vision_prompts,
    decode_mask_b64,
    encode_mask_b64,
    ground_detections_with_depth,
    ground_mask_with_depth,
    render_visual_enhancement_overlay,
)
from embodied_navigation.planning.exploration.habitat_memory_runner import (
    HabitatMemoryRunner,
)
from embodied_navigation.planning.exploration.llm_agent_client import LlmClientConfig
from embodied_navigation.planning.exploration.streaming_task_agent import (
    OpenCodeTaskPlanner,
)


class TestMaskCodecAndPrompts(unittest.TestCase):
    def test_mask_roundtrip_and_bounds(self) -> None:
        mask = np.zeros((64, 96), dtype=bool)
        mask[12:30, 20:55] = True
        payload = encode_mask_b64(mask)
        decoded = decode_mask_b64(payload)
        self.assertEqual(decoded.shape, (64, 96))
        self.assertTrue(np.array_equal(mask, decoded))

    def test_decode_rejects_oversized_corrupt_and_trailing_data(self) -> None:
        with self.assertRaises(ValueError):
            decode_mask_b64({"h": 5000, "w": 5000, "data_b64": ""})

        # Missing required fields raise ValueError (not KeyError)
        with self.assertRaises(ValueError):
            decode_mask_b64({"h": 4, "w": 4})

        # Valid zlib stream with extra unpacked bytes beyond expected packed mask size
        bad_bytes = zlib.compress(b"\xff" * 20)
        with self.assertRaises(ValueError):
            decode_mask_b64(
                {"h": 4, "w": 4, "data_b64": base64.b64encode(bad_bytes).decode("ascii")}
            )

        # Valid zlib stream followed by trailing unused_data must be rejected
        valid_stream = zlib.compress(b"\xff\xff")
        with self.assertRaises(ValueError):
            decode_mask_b64(
                {
                    "h": 4,
                    "w": 4,
                    "data_b64": base64.b64encode(valid_stream + b"TRAILING").decode("ascii"),
                }
            )

    def test_build_vision_prompts_dedups_and_keeps_target_first(self) -> None:
        prompts = build_vision_prompts(
            "  Bed ",
            context_prompts=("chair", "bed", "table", "couch", "chair", "lamp"),
            max_prompts=4,
        )
        self.assertEqual(prompts, ["bed", "chair", "table", "couch"])


class TestMaskDepthGroundingAndOverlay(unittest.TestCase):
    def test_pinhole_center_and_left_bearing_match_occupancy_grid(self) -> None:
        h, w = 128, 128
        depth = np.full((h, w), 2.0, dtype=np.float32)

        # Center patch -> x=2.0, y~0.0, z~0.88, bearing~0
        mask_c = np.zeros((h, w), dtype=bool)
        mask_c[58:70, 58:70] = True
        res_c = ground_mask_with_depth(mask_c, depth, hfov_deg=90.0, camera_height_m=0.88)
        self.assertIsNotNone(res_c)
        assert res_c is not None
        self.assertAlmostEqual(res_c["local_xyz"][0], 2.0, places=2)
        self.assertAlmostEqual(res_c["local_xyz"][1], 0.0, delta=0.05)
        self.assertAlmostEqual(res_c["local_xyz"][2], 0.88, delta=0.05)
        self.assertAlmostEqual(res_c["bearing_deg"], 0.0, delta=1.5)

        # Left side of image (small u) -> +y and positive bearing_deg
        mask_l = np.zeros((h, w), dtype=bool)
        mask_l[58:70, 12:28] = True
        res_l = ground_mask_with_depth(mask_l, depth, hfov_deg=90.0, camera_height_m=0.88)
        self.assertIsNotNone(res_l)
        assert res_l is not None
        self.assertGreater(res_l["local_xyz"][1], 1.0)
        self.assertGreater(res_l["bearing_deg"], 25.0)

    def test_foreground_band_trims_majority_background_wall_bleed(self) -> None:
        h, w = 100, 100
        depth = np.full((h, w), 6.5, dtype=np.float32)
        mask = np.zeros((h, w), dtype=bool)
        # 40% foreground chair at 1.6 m, 60% background wall bleed at 6.5 m
        mask[30:60, 30:60] = True  # 900 px
        depth[30:42, 30:60] = 1.6  # 360 px (40%) foreground, 540 px (60%) background
        res = ground_mask_with_depth(mask, depth)
        self.assertIsNotNone(res)
        assert res is not None
        self.assertEqual(res["range_source"], "vision_mask_depth_backproj")
        self.assertAlmostEqual(res["local_xyz"][0], 1.6, delta=0.15)
        self.assertAlmostEqual(res["surface_distance_m"], 1.6, delta=0.2)
        self.assertLess(res["extent_xyz_m"][0], 0.5)

    def test_box_fallback_and_mask_resize(self) -> None:
        depth = np.full((128, 128), 3.0, dtype=np.float32)
        # Box-only fallback explicitly tags range_source so it never masquerades as a mask
        res_box = ground_mask_with_depth(None, depth, box_xyxy=(40, 40, 88, 88))
        self.assertIsNotNone(res_box)
        assert res_box is not None
        self.assertEqual(res_box["range_source"], "vision_box_depth_fallback")
        self.assertAlmostEqual(res_box["local_xyz"][0], 3.0, places=2)

        # Mask at half resolution auto-resizes to depth shape
        small_mask = np.zeros((64, 64), dtype=bool)
        small_mask[20:44, 20:44] = True
        res_rs = ground_mask_with_depth(small_mask, depth)
        self.assertIsNotNone(res_rs)
        assert res_rs is not None
        self.assertEqual(res_rs["range_source"], "vision_mask_depth_backproj")

    def test_overlay_does_not_mutate_input_and_writes_png(self) -> None:
        rgb = np.full((96, 96, 3), 40, dtype=np.uint8)
        rgb_copy = rgb.copy()
        mask = np.zeros((96, 96), dtype=bool)
        mask[20:60, 20:60] = True
        det = GroundedDetection(
            det_id="V1",
            label="bed",
            score=0.88,
            box_xyxy=(20.0, 20.0, 60.0, 60.0),
            local_xyz=(1.4, 0.1, 0.5),
            extent_xyz_m=(0.8, 1.2, 0.6),
            surface_distance_m=1.35,
            planar_depth_m=1.3,
            bearing_deg=4.0,
            valid_pixels=1200,
            backend="sam3",
            mask=mask,
        )
        with tempfile.TemporaryDirectory() as td:
            out_path = render_visual_enhancement_overlay(
                rgb, [det], Path(td) / "vis.png", target_label="bed"
            )
            self.assertTrue(out_path.is_file())
            self.assertTrue(np.array_equal(rgb, rgb_copy))
            vis_arr = np.asarray(Image.open(out_path))
            self.assertEqual(vis_arr.shape, (96, 96, 3))
            # Center of highlighted target mask must have higher green channel than raw (40)
            self.assertGreater(int(vis_arr[40, 40, 1]), 70)


class TestVisionToolClientAndIntegration(unittest.TestCase):
    def test_ssrf_guard_rejects_non_loopback_by_default(self) -> None:
        with self.assertRaises(ValueError):
            VisionToolClient("http://192.168.1.50:8114")
        cli = VisionToolClient("http://192.168.1.50:8114", allow_remote=True)
        self.assertEqual(cli.base_url, "http://192.168.1.50:8114")

    def test_http_client_end_to_end_with_mock_server(self) -> None:
        mask_gt = np.zeros((64, 64), dtype=bool)
        mask_gt[16:48, 16:48] = True
        mask_payload = encode_mask_b64(mask_gt)

        class _Handler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args: object) -> None:  # noqa: A002
                return

            def do_GET(self) -> None:
                if self.path == "/health":
                    body = json.dumps(
                        {"ok": True, "backend": "sam3", "embed_model": "ViT-B-16-SigLIP"}
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "application/json")
                    self.end_headers()
                    self.wfile.write(body)
                else:
                    self.send_response(404)
                    self.end_headers()

            def do_POST(self) -> None:
                n = int(self.headers.get("Content-Length", "0"))
                req = json.loads(self.rfile.read(n).decode("utf-8"))
                if self.path == "/segment_text":
                    resp = {
                        "backend": "sam3",
                        "detections": [
                            {
                                "label": req["prompts"][0],
                                "score": 0.84,
                                "box_xyxy": [16.0, 16.0, 48.0, 48.0],
                                "mask": mask_payload,
                                "embedding": [3.0, 4.0, 0.0],
                            },
                            {
                                "label": "corrupt_chair",
                                "score": 0.99,
                                "box_xyxy": [10.0, 10.0, 30.0, 30.0],
                                "mask": {"h": 64, "w": 64, "data_b64": "NOT_VALID_ZLIB"},
                            },
                        ],
                    }
                elif self.path == "/embed_image":
                    resp = {"embedding": [0.0, 1.0, 0.0]}
                elif self.path == "/embed_text":
                    resp = {"embedding": [0.0, 2.0, 0.0]}
                else:
                    self.send_response(404)
                    self.end_headers()
                    return
                body = json.dumps(resp).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(body)

        srv = ThreadingHTTPServer(("127.0.0.1", 0), _Handler)
        port = srv.server_address[1]
        th = threading.Thread(target=srv.serve_forever, daemon=True)
        th.start()
        try:
            client = VisionToolClient(f"http://127.0.0.1:{port}", timeout_s=3.0)
            h = client.health()
            self.assertTrue(h["ok"])
            self.assertEqual(client.backend_name, "sam3")
            self.assertEqual(client.embed_model, "ViT-B-16-SigLIP")

            rgb = np.zeros((64, 64, 3), dtype=np.uint8)
            depth = np.full((64, 64), 1.5, dtype=np.float32)
            dets = client.segment_text(rgb, ["toilet", "chair"], include_crop_embeddings=True)
            self.assertEqual(len(dets), 1)
            self.assertEqual(dets[0].label, "toilet")
            self.assertIsNotNone(dets[0].mask)
            self.assertIsNotNone(dets[0].embedding)
            assert dets[0].embedding is not None
            self.assertAlmostEqual(dets[0].embedding[0], 0.6, places=4)
            self.assertAlmostEqual(dets[0].embedding[1], 0.8, places=4)

            grounded = ground_detections_with_depth(dets, depth)
            self.assertEqual(len(grounded), 1)
            self.assertEqual(grounded[0].det_id, "V1")
            self.assertAlmostEqual(grounded[0].surface_distance_m, 1.5, delta=0.1)

            emb_i = client.embed_image(rgb)
            emb_t = client.embed_text("a photo of a toilet")
            self.assertEqual(emb_i, (0.0, 1.0, 0.0))
            self.assertEqual(emb_t, (0.0, 1.0, 0.0))

            # Verify HierarchicalMemoryStore preserves embedding across annotate_keyframe calls,
            # replaces surfaces when explicitly provided, and retrieves via continuous embedding
            mem = HierarchicalMemoryStore(KeyframeSceneGraph())
            mem.graph.add_keyframe("kf1", np.eye(4), rgb_path="kf1.png")
            mem.annotate_keyframe("kf1", surfaces=("old_desk",), embedding=emb_i)
            mem.annotate_keyframe("kf1", room_type="hallway", scene_description="plain corridor")
            self.assertEqual(mem._keyframe_meta["kf1"].surfaces, ("old_desk",))
            mem.annotate_keyframe("kf1", surfaces=("counter", "shelf"))
            self.assertEqual(mem._keyframe_meta["kf1"].surfaces, ("counter", "shelf"))
            self.assertEqual(mem._keyframe_meta["kf1"].embedding, emb_i)
            c2f = mem.coarse_to_fine_search("toothbrush", query_embedding=emb_t, top_k_keyframes=2)
            self.assertEqual(len(c2f["top_keyframes"]), 1)
            self.assertEqual(c2f["top_keyframes"][0]["keyframe_id"], "kf1")

            # Verify planner injects vision_tool_detections into [TOOL_RESULTS]
            planner = OpenCodeTaskPlanner(LlmClientConfig(supports_vision=False))
            msgs = planner.build_messages(
                task_instruction="find toilet",
                step_index=1,
                robot_xyz=[0.0, 0.0, 0.0],
                yaw_rad=0.0,
                candidates=[],
                vision_detections=[grounded[0].to_prompt_dict()],
            )
            self.assertIn("vision_tool_detections", msgs[1]["content"])
            self.assertIn('"id":"V1"', msgs[1]["content"])

            # Verify deferred L3 scanner respects output_dir boundary & strips rgb_path
            with tempfile.TemporaryDirectory() as td, tempfile.TemporaryDirectory() as outside_td:
                out_dir = Path(td)
                inside_img = out_dir / "kf1.png"
                outside_img = Path(outside_td) / "secret.png"
                Image.fromarray(rgb).save(inside_img)
                Image.fromarray(rgb).save(outside_img)

                runner = HabitatMemoryRunner(
                    scene_dataset_config="dummy.json",
                    scene_id="dummy.glb",
                    planner=planner,
                    output_dir=out_dir,
                    vision_client=client,
                )
                hits, rgbs = runner._deferred_vision_scan_keyframes(
                    "toilet",
                    [
                        {"keyframe_id": "kf_bad", "rgb_path": str(outside_img), "pose_xyz": [9, 9, 0]},
                        {"keyframe_id": "kf_ok", "rgb_path": str(inside_img), "pose_xyz": [1, 2, 0]},
                    ],
                    has_seg=True,
                    max_kfs=2,
                )
                self.assertEqual(len(hits), 1)
                self.assertEqual(hits[0]["keyframe_id"], "kf_ok")
                self.assertNotIn("rgb_path", hits[0])
                self.assertEqual(rgbs, [str(inside_img.resolve())])
        finally:
            srv.shutdown()
            srv.server_close()

    def test_client_fails_open_when_service_unavailable(self) -> None:
        client = VisionToolClient("http://127.0.0.1:1", timeout_s=0.5)
        self.assertFalse(client.health()["ok"])
        self.assertEqual(client.segment_text(np.zeros((32, 32, 3), dtype=np.uint8), ["bed"]), [])
        self.assertIsNone(client.embed_text("bed"))
        self.assertIsNotNone(client.last_error)


if __name__ == "__main__":
    unittest.main()
