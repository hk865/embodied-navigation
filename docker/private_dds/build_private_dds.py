#!/usr/bin/env python3
"""Build the pinned, privately installed Fast DDS 2.14.6 backport. No downloads.

Without --execute, validate paths and print the complete build plan.
Every build uses fresh source/build/prefix directories, never a reused CMake cache.
"""
import argparse
from pathlib import Path
import subprocess
import sys
import tarfile
sys.dont_write_bytecode = True
from common import ROOT, MATERIALS, private_path, read, write_new


def extract(archive, destination, expected_root):
    with tarfile.open(archive, 'r:gz') as source:
        for member in source.getmembers():
            target = destination / member.name
            if (not target.resolve().is_relative_to(destination) or
                    member.name.split('/')[0] != expected_root or member.isdev()):
                raise ValueError('Unsafe/unexpected source archive member')
        source.extractall(destination, filter='data')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fastdds-archive', type=Path, required=True)
    parser.add_argument('--asio-archive', type=Path, required=True)
    parser.add_argument('--work-dir', type=Path, default=ROOT/'build/dds')
    parser.add_argument('--prefix', type=Path, default=ROOT/'install/dds')
    parser.add_argument('--ros-prefix', type=Path, default=Path('/opt/ros/jazzy'))
    parser.add_argument('--gtest-source', type=Path, default=Path('/usr/src/googletest'))
    parser.add_argument('--gtest-dir', type=Path, help='Existing private/system GTest CMake directory instead of building the local source')
    parser.add_argument('--jobs', type=int, default=4)
    parser.add_argument('--baseline', action='store_true', help='Build unpatched library with the same regression fixtures')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error('--jobs must be positive')
    pins = read(MATERIALS/'SOURCES.json')
    for archive in (args.fastdds_archive,args.asio_archive):
        if not archive.is_file() or archive.stat().st_size == 0:
            raise ValueError('Missing/empty source archive: '+str(archive))
    work, prefix = private_path(args.work_dir), private_path(args.prefix)
    if work.exists() or prefix.exists() or work.is_relative_to(prefix) or prefix.is_relative_to(work):
        raise ValueError('Separate, fresh private work and install directories required')
    ros = args.ros_prefix.resolve(strict=True)
    source = work/pins['fastdds']['archive_root']
    asio = work/pins['asio']['archive_root']/'asio/include'
    build = work/'build'
    gtest = args.gtest_dir.resolve(strict=True) if args.gtest_dir else work/'gtest-install/lib/cmake/GTest'
    steps = []
    if not args.gtest_dir:
        local_gtest = args.gtest_source.resolve(strict=True)
        steps += [['cmake', '-S', str(local_gtest), '-B', str(work/'gtest-build'),
                   '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_INSTALL_PREFIX='+str(work/'gtest-install'),
                   '-DBUILD_GMOCK=ON', '-DINSTALL_GTEST=ON'],
                  ['cmake', '--build', str(work/'gtest-build'), '--parallel', str(args.jobs)],
                  ['cmake', '--install', str(work/'gtest-build')]]
    patches = ([] if args.baseline else [pins['backport']['production_patch']]) + pins['backport']['test_patches']
    steps += [['patch', '--batch', '--forward', '-p1', '-d', str(source), '-i', str(MATERIALS/name)] for name in patches]
    configure = ['cmake', '-S', str(source), '-B', str(build),
                 '-DCMAKE_BUILD_TYPE=RelWithDebInfo', '-DCMAKE_INSTALL_PREFIX='+str(prefix),
                 '-DCMAKE_PREFIX_PATH='+str(ros), '-DAsio_INCLUDE_DIR='+str(asio),
                 '-DTHIRDPARTY=OFF', '-DTHIRDPARTY_UPDATE=OFF', '-DSECURITY=ON',
                 '-DFASTDDS_STATISTICS=ON', '-DSQLITE3_SUPPORT=ON', '-DLOG_NO_INFO=ON',
                 '-DSHM_TRANSPORT_DEFAULT=ON', '-DCOMPILE_EXAMPLES=OFF', '-DCOMPILE_TOOLS=ON',
                 '-DEPROSIMA_BUILD_TESTS=ON', '-DSYSTEM_TESTS=OFF', '-DPERFORMANCE_TESTS=OFF',
                 '-DGTest_DIR='+str(gtest)]
    steps += [configure, ['cmake', '--build', str(build), '--target', 'fastrtps',
                         'SharedMemTests', 'fast-discovery-server', '--parallel', str(args.jobs)],
              ['cmake', '--install', str(build)]]
    plan = dict(schema='private_fastdds_build/v1', execute=args.execute,
                variant='unpatched' if args.baseline else 'patched',
                source_pins=pins, commands=steps, prefix=str(prefix), work_dir=str(work),
                no_download=True, no_system_install=True,
                test_command=[str(build/'test/unittest/transport/SharedMemTests'),
                              '--gtest_filter=SHMTransportTests.reviewed_heartbeat_*'])
    if not args.execute:
        import json
        print(json.dumps(plan, indent=2))
        return
    work.mkdir(parents=True)
    write_new(work/'BUILD_PLAN.json', plan)
    extract(args.fastdds_archive, work, pins['fastdds']['archive_root'])
    extract(args.asio_archive, work, pins['asio']['archive_root'])
    # Upstream enables Docker-network integration test configuration together
    # with unit tests. The image deliberately has no Docker daemon/socket.
    # Disable only those unrelated integration fixtures; SharedMemTests remains
    # configured, compiled and executed by the parent build command.
    test_cmake=source/'test/dds/communication/CMakeLists.txt'
    text=test_cmake.read_text()
    old='if(UNIX AND NOT(APPLE) AND NOT(QNXNTO) AND NOT(ANDROID))\n    add_subdirectory(dyn_network)'
    if text.count(old)!=1:raise ValueError('Unexpected upstream Docker test selector')
    test_cmake.write_text(text.replace(old,
        'option(NAVIGATION_DOCKER_NETWORK_TESTS "Build unrelated nested-Docker network fixtures" OFF)\n'
        'if(NAVIGATION_DOCKER_NETWORK_TESTS AND UNIX AND NOT(APPLE) AND NOT(QNXNTO) AND NOT(ANDROID))\n'
        '    add_subdirectory(dyn_network)'))
    for index, command in enumerate(steps):
        with (work/('step_%02d.log' % index)).open('x') as log:
            subprocess.run(command, check=True, stdout=log, stderr=subprocess.STDOUT)
    library = prefix/'lib/libfastrtps.so.2.14.6'
    if not library.is_file():raise FileNotFoundError(library)
    write_new(work/'BUILD_RESULT.json',dict(library=str(library),bytes=library.stat().st_size,
        patches_applied=patches,tests_built=True,tests_executed=False))
    print('Private DDS built; regression tests are executed by the parent build command.')



if __name__ == '__main__':
    main()
