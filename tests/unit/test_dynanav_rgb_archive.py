"""Raw RGB evidence preserves camera bytes and sensor timestamps without video decoding."""

import tempfile
import unittest
import zipfile
from pathlib import Path

import numpy as np

from embodied_navigation.simulation.dynanav.rgb_archive import RawRgbArchive


class RawRgbArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name) / "rgb_raw"
        self.archive = RawRgbArchive(self.directory)

    def test_noncontiguous_renderer_rgb_is_exact_uncompressed_and_timestamped(self):
        rgba = np.arange(512 * 512 * 4, dtype=np.uint8).reshape(512, 512, 4)
        rgb = rgba[..., :3]
        expected = rgb.copy()
        source_ns = round(float(np.float32(73.715)) * 1e9)
        path = self.archive.write(rgb, source_ns=source_ns, frame_index=0)
        rgba[:] = 0
        with np.load(path, allow_pickle=False) as saved:
            self.assertEqual(saved["rgb"].shape, (512, 512, 3))
            self.assertEqual(saved["rgb"].dtype, np.uint8)
            self.assertEqual(saved["rgb"].tobytes(), expected.tobytes())
            self.assertEqual(saved["source_ns"].dtype, np.int64)
            self.assertEqual(saved["source_ns"].item(), source_ns)
            self.assertEqual(saved["frame_index"].item(), 0)
            self.assertEqual(saved["frame_id"].item(), "go2_camera_optical")
            self.assertEqual(saved["encoding"].item(), "rgb8")
        with zipfile.ZipFile(path) as saved:
            self.assertTrue(
                all(
                    item.compress_type == zipfile.ZIP_STORED
                    for item in saved.infolist()
                )
            )
        receipt = self.archive.receipt(
            acquired_frames=1, completed=True, published_frames=1, video_frames=1
        )
        self.assertTrue(receipt["all_frames_condition_met"])
        self.assertFalse(
            self.archive.receipt(acquired_frames=2)["all_acquired_frames_archived"]
        )
        self.assertFalse(
            self.archive.receipt(
                acquired_frames=1, completed=True, published_frames=0, video_frames=1
            )["all_frames_condition_met"]
        )

    def test_existing_frame_is_never_overwritten(self):
        target = self.directory / "000000.npz"
        target.write_bytes(b"existing evidence")
        with self.assertRaises(FileExistsError):
            self.archive.write(
                np.zeros((2, 3, 3), dtype=np.uint8), source_ns=1, frame_index=0
            )
        self.assertEqual(target.read_bytes(), b"existing evidence")
        self.assertEqual(self.archive.count, 0)
        with self.assertRaises(FileExistsError):
            RawRgbArchive(self.directory)

    def test_invalid_pixels_or_nonconsecutive_index_do_not_create_archive_frames(self):
        for pixels in [
            np.zeros((2, 3, 3), dtype=float),
            np.zeros((2, 3, 4), dtype=np.uint8),
        ]:
            with self.assertRaises(ValueError):
                self.archive.write(pixels, source_ns=1, frame_index=0)
        with self.assertRaises(ValueError):
            self.archive.write(
                np.zeros((2, 3, 3), dtype=np.uint8), source_ns=1, frame_index=1
            )
        self.assertEqual(list(self.directory.iterdir()), [])


if __name__ == "__main__":
    unittest.main()
