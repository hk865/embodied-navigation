"""RTX geometry is expanded in memory before renderer stage population."""
import contextlib
import hashlib
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import Mock, patch

from embodied_navigation.simulation.dynanav.capture import scene


class RtxScenePreparationTests(unittest.TestCase):
    def test_session_only_nested_expansion_precedes_cache_attachment(self):
        events = []
        class Prim:
            def __init__(self, path):
                self.path, self.instance = path, True
            def IsInstance(self): return self.instance
            def GetPath(self): return self.path
            def SetInstanceable(self, value):
                self.instance = value
                events.append(('instanceable', self.path, value))
                return True
        outer, inner = Prim('/Root/Outer'), Prim('/Root/Outer/Inner')
        stage = Mock()
        stage.Traverse.side_effect = lambda: [outer] if outer.instance else [outer, inner]
        session = object()
        stage.GetSessionLayer.return_value = session
        stage.GetPrimAtPath.side_effect = lambda path: {outer.path: outer, inner.path: inner}[path]
        @contextlib.contextmanager
        def edit_context(actual_stage, actual_layer):
            self.assertIs(actual_stage, stage)
            self.assertIs(actual_layer, session)
            events.append('session_enter')
            yield
            events.append('session_exit')
        cache = Mock()
        def insert(actual_stage):
            self.assertIs(actual_stage, stage)
            self.assertFalse(outer.instance or inner.instance)
            self.assertEqual(events[-1], 'session_exit')
            return types.SimpleNamespace(ToLongInt=lambda: 17)
        cache.Insert.side_effect = insert
        pxr = types.SimpleNamespace(
            Sdf=types.SimpleNamespace(Layer=types.SimpleNamespace(FindOrOpen=lambda _: object(), CreateAnonymous=lambda _: session)),
            Usd=types.SimpleNamespace(Stage=types.SimpleNamespace(Open=lambda *args: stage), EditContext=edit_context),
            UsdUtils=types.SimpleNamespace(StageCache=types.SimpleNamespace(Get=lambda: cache)),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.usd'
            path.write_bytes(b'original source')
            with patch.dict(sys.modules, {'pxr': pxr}):
                stage_id, receipt = scene.prepare_rtx_scene_stage(path)
            self.assertEqual(path.read_bytes(), b'original source')
        self.assertEqual(stage_id, 17)
        self.assertEqual(receipt['expanded_instance_paths'], ['/Root/Outer', '/Root/Outer/Inner'])
        self.assertEqual(receipt['expanded_instance_count'], 2)
        self.assertEqual(receipt['remaining_instance_count'], 0)
        self.assertFalse(receipt['source_saved'])
        self.assertFalse(receipt['geometry_authored'])
        self.assertFalse(receipt['collision_attributes_authored'])
        self.assertEqual(receipt['source_sha256_before'], hashlib.sha256(b'original source').hexdigest())
        self.assertEqual(receipt['source_sha256_before'], receipt['source_sha256_after'])
        stage.Save.assert_not_called()
        stage.GetRootLayer.assert_not_called()

    def test_failed_expansion_refuses_attachment(self):
        prim = Mock()
        prim.IsInstance.return_value = True
        prim.GetPath.return_value = '/Root/Failed'
        prim.SetInstanceable.return_value = False
        stage = Mock()
        stage.Traverse.return_value = [prim]
        stage.GetPrimAtPath.return_value = prim
        cache = Mock()
        pxr = types.SimpleNamespace(
            Sdf=types.SimpleNamespace(Layer=types.SimpleNamespace(FindOrOpen=lambda _: object(), CreateAnonymous=lambda _: object())),
            Usd=types.SimpleNamespace(Stage=types.SimpleNamespace(Open=lambda *args: stage), EditContext=lambda *_: contextlib.nullcontext()),
            UsdUtils=types.SimpleNamespace(StageCache=types.SimpleNamespace(Get=lambda: cache)),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'source.usd'; path.touch()
            with patch.dict(sys.modules, {'pxr': pxr}), self.assertRaisesRegex(RuntimeError, 'Cannot expand'):
                scene.prepare_rtx_scene_stage(path)
        cache.Insert.assert_not_called()

    def test_rtx_uses_prepared_attachment_and_rgbd_retains_open(self):
        for source in ('rtx_lidar', 'rgbd'):
            with self.subTest(source=source):
                stage = object()
                context = Mock()
                context.get_stage.return_value = stage
                context.get_stage_id.return_value = 17
                omni = types.ModuleType('omni')
                omni.usd = types.ModuleType('omni.usd')
                omni.usd.get_context = lambda: context
                geom = Mock()
                pxr = types.SimpleNamespace(UsdGeom=geom)
                result = {}
                with patch.dict(sys.modules, {'omni': omni, 'omni.usd': omni.usd, 'pxr': pxr}), patch.object(scene, 'prepare_rtx_scene_stage', return_value=(17, {'expanded_instance_count': 2})) as prepare, patch.object(scene, 'attach_prepared_rtx_stage') as attach:
                    self.assertIs(scene.open_stage('asset.usd', {'point_cloud_source': source}, result, Mock()), stage)
                if source == 'rtx_lidar':
                    prepare.assert_called_once_with('asset.usd')
                    attach.assert_called_once_with(context, 17)
                    context.open_stage.assert_not_called()
                    self.assertEqual(result['rtx_scene_preparation']['expanded_instance_count'], 2)
                else:
                    prepare.assert_not_called()
                    context.open_stage.assert_called_once_with('asset.usd')
                    attach.assert_not_called()


    def test_attachment_waits_for_success_callback(self):
        context = Mock()
        callbacks = []
        context.attach_stage_with_callback.side_effect = lambda ident, cb: callbacks.append(cb) or True
        context.get_stage_id.return_value = 17
        update = Mock(side_effect=lambda: callbacks[0](True, ""))
        pxr = types.SimpleNamespace(Usd=Mock(), UsdUtils=Mock())
        with patch.dict(sys.modules, {'pxr': pxr}):
            scene.attach_prepared_rtx_stage(context, 17, update=update)
        update.assert_called_once()
        context.close_stage.assert_not_called()
        pxr.UsdUtils.StageCache.Get.assert_not_called()

    def test_failed_callback_cleans_only_owned_cache_entry(self):
        for active_id in (17, 42):
            context = Mock()
            context.get_stage_id.return_value = active_id
            context.attach_stage_with_callback.side_effect = lambda ident, cb: cb(False, "bad stage") or True
            cache = Mock()
            pxr = types.SimpleNamespace(
                Usd=types.SimpleNamespace(StageCache=types.SimpleNamespace(Id=types.SimpleNamespace(FromLongInt=lambda n: ('id', n)))),
                UsdUtils=types.SimpleNamespace(StageCache=types.SimpleNamespace(Get=lambda: cache)),
            )
            with patch.dict(sys.modules, {'pxr': pxr}), self.assertRaisesRegex(RuntimeError, 'bad stage'):
                scene.attach_prepared_rtx_stage(context, 17, update=Mock())
            cache.Erase.assert_called_once_with(('id', 17))
            cache.Clear.assert_not_called()
            self.assertEqual(context.close_stage.call_count, int(active_id == 17))

if __name__ == '__main__':
    unittest.main()
