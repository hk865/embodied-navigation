"""The bounded launcher is explicit; other Isaac entry points keep file logs."""

import sys
import types
import unittest
from unittest.mock import patch

from embodied_navigation.simulation.dynanav.capture.scene import start_simulation_app


class KitLogRoutingTests(unittest.TestCase):
    def app_config(self, **options):
        module = types.ModuleType("isaacsim")
        module.SimulationApp = lambda config: config
        with patch.dict(sys.modules, {"isaacsim": module}):
            return start_simulation_app(False, **options)

    def test_unwrapped_entry_preserves_kit_file_defaults(self):
        args = self.app_config()["extra_args"]
        self.assertFalse(any(argument.startswith("--/log/") for argument in args))

    def test_wrapped_entry_keeps_info_without_duplicate_file_or_channel_filter(self):
        args = self.app_config(log_to_stdout=True)["extra_args"]
        self.assertIn("--/log/file=", args)
        self.assertIn("--/log/enableStandardStreamOutput=true", args)
        self.assertIn("--/log/outputStreamLevel=info", args)
        self.assertFalse(any("/log/channels/" in argument for argument in args))
        self.assertFalse(any(argument.startswith("--/log/level=") for argument in args))


if __name__ == "__main__":
    unittest.main()
