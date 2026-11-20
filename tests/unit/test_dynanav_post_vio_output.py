"""Source wiring checks; actual publication has a separate native contract test."""

import unittest

from embodied_navigation.project import project_root


class PostVioOutputWiring(unittest.TestCase):
    def test_opt_in_final_state_is_separate_from_pre_vio_output(self):
        root = project_root() / "third_party/fast_livo2/fast_livo2_core"
        source = (root / "src/LIVMapper.cpp").read_text()
        header = (root / "include/fast_livo2_core/core/LIVMapper.h").read_text()
        self.assertIn(
            'declare_parameter<bool>("publish.post_vio_odometry_en", false)', source
        )
        self.assertIn("bool post_vio_odometry_en = false;", header)
        handle_vio = source.split("void LIVMapper::handleVIO() {", 1)[1].split(
            "void LIVMapper::handleLIO() {", 1
        )[0]
        self.assertLess(
            handle_vio.index("vio_manager->processFrame("),
            handle_vio.index("publish_post_vio_odometry("),
        )
        self.assertEqual(handle_vio.count("publish_post_vio_odometry("), 1)
        # An empty cloud returns before processFrame and must not masquerade as
        # the output of a visual processing cycle.
        self.assertLess(
            handle_vio.index("return;"), handle_vio.index("vio_manager->processFrame(")
        )
        self.assertNotIn("publish_odometry(", handle_vio)
        handle_lio = source.split("void LIVMapper::handleLIO() {", 1)[1].split(
            "void LIVMapper::", 1
        )[0]
        self.assertNotIn("publish_post_vio_odometry(", handle_lio)
        self.assertIn("publish_odometry(pubOdomAftMapped)", handle_lio)
        publishers = source.split("if (post_vio_odometry_en) {", 1)[1].split(
            "\n    }", 1
        )[0]
        self.assertIn('("/post_vio_odometry", 10)', publishers)
        self.assertIn('"/post_vio_odometry_status", 10', publishers)


if __name__ == "__main__":
    unittest.main()
