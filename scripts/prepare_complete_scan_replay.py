#!/usr/bin/env python3
"""Export SHA-verified stored original RTX records and recorded IMU, read-only.

Writes only an explicitly requested new fixture directory. Does not claim the
original map/state/image history exists. CPU dual-route tests use fresh identical
state and blank images, not an original full-trajectory replay.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import struct
import zipfile
import numpy as np


def export(run, output):
    output.mkdir(exist_ok=False, parents=True)
    receipts = {x['stamp_ns']: x for x in map(json.loads, (run/'capture/rtx_lidar_frames.jsonl').read_text().splitlines())}
    scans = []
    for p in sorted(run.glob('raw_scan_*.npz'), key=lambda p: int(p.stem.split('_')[-1])):
        ns = int(p.stem.split('_')[-1])
        with zipfile.ZipFile(p) as z, z.open('records.npy') as f:
            version = np.lib.format.read_magic(f)
            shape, order, dtype = np.lib.format._read_array_header(f, version)
            raw = f.read()
        r = receipts[ns]
        digest = hashlib.sha256(raw).hexdigest()
        assert not order and dtype.itemsize == 32 and len(raw) == shape[0]*32
        assert digest == r['point_sha256']
        assert r['scan_complete'] == 1 and r['header_rebase_ns'] == 0
        pose = r['native_world_pose_scoring_only']
        assert pose['end']['timestamp_ns'] == ns+100_000_000
        assert pose['start']['timestamp_ns'] == ns+15_000_000
        scans.append((ns, raw, digest, str(p)))
    assert len(scans) == 12 and all(b[0]-a[0] == 100_000_000 for a,b in zip(scans,scans[1:]))
    imus=[]
    with (run/'observer/diagnostics/records.bin').open('rb') as f:
        for row in csv.DictReader((run/'observer/diagnostics/index.csv').open()):
            if row['kind'] != '1': continue
            ns=int(row['stamp_ns'])
            if not scans[0][0] <= ns <= scans[-1][0]+10_000_000: continue
            f.seek(int(row['offset'])+56)
            v=struct.unpack('<'+'d'*int(row['n_values']),f.read(8*int(row['n_values'])))
            assert len(v)==42 and v[3]==0 and v[4]==0
            imus.append((ns,v[5:]))
    with (output/'fixture.bin').open('xb') as f:
        f.write(struct.pack('<Q',len(scans)))
        for ns,raw,_,_ in scans: f.write(struct.pack('<QQ',ns,len(raw))); f.write(raw)
        f.write(struct.pack('<Q',len(imus)))
        for ns,values in imus: f.write(struct.pack('<Q37d',ns,*values))
    (output/'source-manifest.json').write_text(json.dumps({'run':str(run),'scans':[{'source_ns':ns,'raw_sha256':sha,'file':p} for ns,_,sha,p in scans], 'imu_samples':len(imus), 'source_files':{str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (run/'capture/rtx_lidar_frames.jsonl',run/'observer/diagnostics/index.csv',run/'observer/diagnostics/records.bin')}, 'fixture_sha256':hashlib.sha256((output/'fixture.bin').read_bytes()).hexdigest(), 'limits':'Fresh equal initial state; no original image/history/map, not original trajectory replay.'},indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('run',type=Path);parser.add_argument('output',type=Path)
    args=parser.parse_args();export(args.run,args.output)
