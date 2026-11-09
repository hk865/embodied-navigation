"""Launcher-owned profile pins report when they replace configured values."""

import contextlib
import io
import unittest

from embodied_navigation.launch.prepare import _pin


class PinTest(unittest.TestCase):
    def pin(self, mapping, key, value):
        stream = io.StringIO()
        with contextlib.redirect_stderr(stream):
            _pin(mapping, key, value, "test reason", "some.key")
        return stream.getvalue()

    def test_differing_value_is_applied_and_reported(self):
        mapping = {"some": {"key": True}}
        text = self.pin(mapping["some"], "key", False)
        self.assertFalse(mapping["some"]["key"])
        self.assertIn("pinning some.key", text)
        self.assertIn("config had True", text)
        self.assertIn("test reason", text)

    def test_equal_value_is_silent(self):
        mapping = {"value": 3}
        self.assertEqual(self.pin(mapping, "value", 3), "")

    def test_absent_value_is_reported_as_unset(self):
        mapping = {}
        text = self.pin(mapping, "value", "x")
        self.assertEqual(mapping["value"], "x")
        self.assertIn("config had '<unset>'", text)


if __name__ == "__main__":
    unittest.main()
