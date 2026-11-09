"""离线检查配置选择：不启动 ROS、仿真器，不联网或分配 socket。"""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("observer_transport", ROOT / "scripts/dynanav_observer_transport.py")
TRANSPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(TRANSPORT)


class ObserverTransport(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.cwd = Path(temporary.name)
        self.env = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", "FASTDDS_BUILTIN_TRANSPORTS": "UDPv4"}

    def select(self, mode="livo", policy="auto"):
        before = dict(self.env)
        receipt = TRANSPORT.select_transport(mode, policy, self.env, ROOT, self.cwd)
        self.assertEqual(self.env, before)
        return receipt

    def test_default_udp_selects_exact_profile_without_writes(self):
        receipt = self.select()
        self.assertEqual(receipt["receive_buffer_request_bytes"], 4194304)
        self.assertEqual(receipt["environment_assignment"], {"FASTRTPS_DEFAULT_PROFILES_FILE": str(ROOT / TRANSPORT.PROFILE)})
        self.assertEqual(len(receipt["profile_sha256"]), 64)
        self.assertFalse(receipt["socket_buffer_verified"])
        self.assertEqual(list(self.cwd.iterdir()), [])

    def test_baseline_and_lio_preserve_environment(self):
        self.assertEqual(self.select(policy="baseline")["environment_assignment"], {})
        self.assertEqual(self.select(mode="lio")["environment_assignment"], {})

    def test_custom_profiles_and_environment_controls_are_not_overridden(self):
        for key in (*TRANSPORT.PROFILE_KEYS, "FASTDDS_ENVIRONMENT_FILE", "SKIP_DEFAULT_XML"):
            for value in ("", "/custom/missing.xml"):
                with self.subTest(key=key, value=value):
                    self.env[key] = value
                    receipt = self.select()
                    self.assertEqual(receipt["environment_assignment"], {})
                    self.assertEqual(receipt["inherited_environment"][key], value)
                    del self.env[key]

    def test_implicit_cwd_profile_is_preserved(self):
        for name in TRANSPORT.DEFAULT_FILES:
            path = self.cwd / name
            path.write_text("user XML")
            self.assertEqual(self.select()["inherited_default_xml"], [str(path)])
            self.assertEqual(self.select()["environment_assignment"], {})
            path.unlink()

    def test_other_or_unspecified_rmw_and_transports_are_preserved(self):
        for key, values in (("RMW_IMPLEMENTATION", (None, "rmw_cyclonedds_cpp")),
                            ("FASTDDS_BUILTIN_TRANSPORTS", (None, "SHM", "DEFAULT", "LARGE_DATA"))):
            original = self.env[key]
            for value in values:
                with self.subTest(key=key, value=value):
                    if value is None:
                        self.env.pop(key, None)
                    else:
                        self.env[key] = value
                    self.assertEqual(self.select()["environment_assignment"], {})
            self.env[key] = original

    def test_invalid_policy_is_rejected(self):
        with self.assertRaises(ValueError):
            self.select(policy="typo")

    def test_xml_contains_only_validated_transport_settings(self):
        namespace = {"f": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
        root = ET.parse(ROOT / TRANSPORT.PROFILE).getroot()
        descriptors = root.findall(".//f:transport_descriptor", namespace)
        self.assertEqual(len(descriptors), 1)
        fields = {node.tag.rsplit("}", 1)[-1]: node.text for node in descriptors[0]}
        self.assertEqual(fields, {"transport_id": "observer_udp_rx4m", "type": "UDPv4", "receiveBufferSize": "4194304"})
        participant = root.find(".//f:participant", namespace)
        self.assertEqual(participant.attrib["is_default_profile"], "true")
        self.assertEqual(participant.find("f:rtps/f:useBuiltinTransports", namespace).text, "false")
        self.assertEqual(participant.find("f:rtps/f:userTransports/f:transport_id", namespace).text, fields["transport_id"])
