"""Overview/RGB video encoding for the capture probe.

This module owns the ffmpeg subprocess and the per-frame 1024x512 panel: it
concatenates the overview camera with the robot RGB input, draws the unchanged
overlay text, and writes raw frames to the unchanged MP4 path. Close semantics
are reused by both the success path (15 s wait) and the failure receipt (10 s).
"""

import numpy as np


class OverviewVideo:
    """Own the ffmpeg encoder process and its frame counter."""

    def __init__(self, out):
        self.out = out
        import subprocess

        from PIL import Image as PILImage
        from PIL import ImageDraw

        video_process = subprocess.Popen(
            [
                "/usr/bin/ffmpeg",
                "-hide_banner",
                "-loglevel",
                "error",
                "-y",
                "-f",
                "rawvideo",
                "-pixel_format",
                "rgb24",
                "-video_size",
                "1024x512",
                "-framerate",
                "10",
                "-i",
                "pipe:0",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "ultrafast",
                "-crf",
                "23",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(out / "multiwaypoint05_overview_and_rgb.mp4"),
            ],
            stdin=subprocess.PIPE,
            stdout=subprocess.DEVNULL,
            stderr=(out / "video_encoder.log").open("w"),
        )
        video_frames = 0
        self.process = video_process
        self.frames = video_frames

    def write(self, overview, rgb, actual_time, target_speed, gt_diagnostic):
        """Write one overview+RGB panel and return the new frame count."""
        from PIL import Image as PILImage
        from PIL import ImageDraw

        video_process = self.process
        video_frames = self.frames
        out = self.out
        panel = PILImage.fromarray(np.concatenate([overview, rgb], axis=1))
        draw = ImageDraw.Draw(panel)
        draw.rectangle((0, 0, 1024, 24), fill=(0, 0, 0))
        draw.text(
            (8, 6),
            f"Go2 | t={actual_time:.2f}s | target {target_speed:.2f}m/s | {'GT diagnostic' if gt_diagnostic else 'SLAM commands'}",
            fill=(255, 255, 255),
        )
        draw.text((522, 30), "Robot RGB input", fill=(255, 255, 255))
        video_process.stdin.write(np.asarray(panel, dtype=np.uint8).tobytes())
        video_frames += 1
        if video_frames % 10 == 0:
            panel.save(out / "latest_overview.jpg")

        self.frames = video_frames
        return video_frames

    def close(self, timeout=15):
        """Close stdin and wait for the encoder, preserving the original timeout."""
        self.process.stdin.close()
        self.process.wait(timeout=timeout)

    @property
    def returncode(self):
        return self.process.returncode
