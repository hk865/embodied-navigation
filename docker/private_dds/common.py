"""Small file-only helpers for the private Fast DDS build tools."""
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MATERIALS = ROOT / 'third_party/fastdds'


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def write_new(path, value):
    with Path(path).open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def run(command):
    return subprocess.run([str(x) for x in command], check=True,
                          capture_output=True, text=True,
                          env={**os.environ, 'DEBUGINFOD_URLS': ''}).stdout


def private_path(path):
    path = Path(path).expanduser().resolve()
    forbidden = ('/usr', '/lib', '/lib64', '/bin', '/sbin', '/etc', '/opt', '/var', '/boot')
    if path == Path('/') or any(path.is_relative_to(p) for p in forbidden):
        raise ValueError('A private output prefix is required; system/ROS paths are forbidden')
    return path


def elf_identity(path):
    path = Path(path).resolve(strict=True)
    with path.open('rb') as stream:
        header = stream.read(64)
    if header[:6] != b'\x7fELF\x02\x01' or struct.unpack_from('<H', header, 18)[0] != 62:
        raise ValueError('Only the reviewed Linux x86_64 little-endian ELF64 ABI is supported')
    notes = run(['readelf', '-n', path])
    match = re.search(r'Build ID: ([0-9a-f]+)', notes)
    if not match or len(match[1]) != 40:
        raise ValueError('Expected a 20-byte GNU BuildID; cannot reuse another ABI identity')
    dynamic = run(['readelf', '-d', path])
    if 'Library soname: [libfastrtps.so.2.14]' not in dynamic:
        raise ValueError('Expected Fast DDS 2.14 SONAME')
    if path.name != 'libfastrtps.so.2.14.6':
        raise ValueError('Expected the canonical Fast DDS 2.14.6 library file')
    private_path(path)
    return dict(path=str(path), sha256=sha(path), build_id=match[1])


def artifact(path):
    path = Path(path).resolve(strict=True)
    return dict(path=str(path), sha256=sha(path))


def allocated_sections(path):
    """Check the split-debug transformation without executing the ELF."""
    raw = Path(path).read_bytes()
    if raw[:6] != b'\x7fELF\x02\x01':
        raise ValueError('Expected ELF64 little endian')
    offset = struct.unpack_from('<Q', raw, 40)[0]
    size, count = struct.unpack_from('<HH', raw, 58)
    result = []
    for index in range(count):
        _, kind, flags, address, start, length, _, _, align, _ = struct.unpack_from(
            '<IIQQQQIIQQ', raw, offset + index * size)
        if flags & 2:
            result.append((kind, flags, address, length, align,
                           None if kind == 8 else hashlib.sha256(raw[start:start+length]).hexdigest()))
    return result
