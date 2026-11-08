# Private Fast DDS 2.14.6 backport

This directory contains source patches and notices, not a replacement system DDS
installation. `SOURCES.json` fixes the official Fast DDS and Asio commits and
archive SHA256 values. The build tool accepts those local archives and never
downloads them. Upstream source archives, build trees, libraries and generated
host identities belong in ignored local directories.

Apply these patches to a fresh source copy in this order:

1. `heartbeat_2_14_6.patch`: production change to
   `src/cpp/rtps/transport/shared_mem/SharedMemGlobal.hpp`.
2. `heartbeat_2_14_6_tests.patch`: regression fixtures, including an explicitly
   legacy listener negative control.
3. `heartbeat_fixture_r2.patch`: correct the test watchdog timestamp and assert
   live port-node identity. These are test-only changes, not production code.

The local production change backports the two-sided handshake described in
[eProsima/Fast-DDS #6559](https://github.com/eProsima/Fast-DDS/pull/6559), related to
[#6535](https://github.com/eProsima/Fast-DDS/issues/6535): the checker wakes idle
listeners and retries after 1ms; a listener also refreshes its heartbeat after a
data-less wake. Data/close fast paths, the total health timeout, reliable QoS and
SHM+UDP topology remain unchanged. Both ends must use the selected private build.
The reviewed PR head is recorded in `SOURCES.json`; the captured upstream status
was open/unmerged on 2026-10-25. This is a local backport, not an upstream release
or a claim about current PR status.

## Notices

Fast DDS and the modified upstream test/source files retain their original
copyright and Apache-2.0 notices. A copy is in `LICENSE.Apache-2.0`. The upstream
archive has no root `NOTICE` file. This README identifies the local modifications;
do not remove the existing notices when applying or redistributing the patches.

Asio and the bundled Boost material retain the Boost-1.0 text in
`LICENSE.Asio-Boost-1.0` and `LICENSE.Boost-1.0`. Bundled filewatch retains its MIT
copyright/license in `LICENSE.filewatch-MIT`. Fetched source copies retain their
own complete notices. The project-authored integration/probe code is separate
from these upstream licenses; this directory does not relicense other project
files.

See [restoration and build layout](../../docs/restoration.md). Build-source archive checks establish ordinary reproducible inputs; they are not startup or business acceptance gates. Runtime uses the project-local patched library and verifies its actual loaded location without historical BuildID/SHA approval binding.
