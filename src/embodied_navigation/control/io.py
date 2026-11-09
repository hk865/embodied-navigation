"""Current live control I/O. Developer archive codecs are separate tools."""

from embodied_navigation.common.runtime_telemetry import RuntimeTelemetry
from embodied_navigation.common.runtime_io import LatestCommandWriter


def make_evidence_writer(run, profile):
    """Return the live runtime telemetry writer (developer codecs live elsewhere)."""
    return RuntimeTelemetry()


def make_command_writer(writer, history, configuration):
    """Return the latest-command writer over the shared runtime I/O sink."""
    return LatestCommandWriter(writer, None)
