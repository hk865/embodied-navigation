import os
from pathlib import Path
import unittest
from unittest.mock import patch
from embodied_navigation.project import data_root


class DataPathTests(unittest.TestCase):
    def test_data_root_is_the_named_project_not_global_parent(self):
        with patch.dict(os.environ, {"PROJECT_DATA_ROOT": "/tmp/example-project-data"}):
            self.assertEqual(data_root(), Path("/tmp/example-project-data/embodied-navigation"))

    def test_data_root_missing_is_actionable(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "PROJECT_DATA_ROOT"):
                data_root()


if __name__ == "__main__":
    unittest.main()
