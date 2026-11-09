import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from embodied_navigation.project import data_root
from embodied_navigation.simulation import mesh_terrain


class DataPathTests(unittest.TestCase):
    def test_data_root_is_the_named_project_not_global_parent(self):
        with patch.dict(os.environ, {"PROJECT_DATA_ROOT": "/tmp/example-project-data"}):
            self.assertEqual(data_root(), Path("/tmp/example-project-data/embodied-navigation"))

    def test_data_root_missing_is_actionable(self):
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaisesRegex(RuntimeError, "PROJECT_DATA_ROOT"):
                data_root()

    def test_mesh_identity_accepts_data_alias_but_not_another_copy(self):
        with tempfile.TemporaryDirectory() as temporary:
            parent = Path(temporary)
            repo = parent / "repo"
            repo.mkdir()
            data = parent / "data"
            assets = data / "embodied-navigation/runtime_assets/assets"
            building = assets / "building"
            building.mkdir(parents=True)
            mesh = building / "Building.dae"
            mesh.write_text("<COLLADA/>")
            np.savez(
                building / "building_triangles.npz",
                triangles=np.array([[[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [0.0, 1.0, 0.0]]]),
            )
            (repo / "assets").symlink_to(assets)
            other = parent / "other.dae"
            other.write_text(mesh.read_text())
            with (
                patch.dict(os.environ, {"PROJECT_DATA_ROOT": str(data)}),
                patch.object(mesh_terrain, "_project_root", return_value=repo),
            ):
                surface = mesh_terrain.MeshSurface(
                    "Building", "file://" + str(repo / "assets/building/Building.dae")
                )
                np.testing.assert_array_equal(surface.intersect(np.array([[0.0, 0.0, 1.0]])), [1.0])
                with self.assertRaises(ValueError):
                    mesh_terrain.MeshSurface("Building", "file://" + str(other))


if __name__ == "__main__":
    unittest.main()
