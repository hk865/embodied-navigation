"""Small isolated vendored trees for source-stage contract tests."""
from difflib import unified_diff
from pathlib import Path


def file_bytes(root):
    return {str(path.relative_to(root)): path.read_bytes()
            for path in sorted(Path(root).rglob('*')) if path.is_file()}


def make_vendor(directory, newline=b'\n'):
    root = Path(directory) / 'repository' / 'third_party' / 'scan'
    (root / 'src').mkdir(parents=True)
    (root / 'patches').mkdir()
    (root / 'assets').mkdir()
    (root / 'src' / 'value.cpp').write_bytes(newline.join([
        b'// context must survive staging', b'int value = 1;', b'// trailing context', b'']))
    (root / 'assets' / 'opaque.bin').write_bytes(bytes(range(256)) + b'\x00\r\n\xff')
    (root / 'README.txt').write_bytes(b'Unchanged bytes.\r\nNo normalization here.\r\n')
    return root


def replacement_patch(old, new):
    before = ['// context must survive staging\n', f'int value = {old};\n', '// trailing context\n']
    after = ['// context must survive staging\n', f'int value = {new};\n', '// trailing context\n']
    target = 'third_party/scan/src/value.cpp'
    lines = list(unified_diff(before, after, fromfile='a/' + target, tofile='b/' + target))
    return ('diff --git a/' + target + ' b/' + target + '\n' + ''.join(lines)).encode()
