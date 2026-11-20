# Asynchronous complete-scan A/B, zero-action trial

## Scope and current status

Approved simulation only. Same repaired original office asset and spawn, with explicit initial yaw0 reset, never navigation progress. RTX native SLAM, front RGB, ideal downward simulated depth and safety fusion remain enabled. Original source stamps, arrival times, unknown/floor gates, complete IMU requirements, physical guard0.65m/s and TTL30wall are unchanged. Command ceiling0.1m/s is a configured upper limit; every actual body command in these diagnostic runs is zero. No DSH or waypoint was submitted. Nonrealtime sensor lockstep is off.

Native prefix: `build_cache/dynanav_complete_scan_20261110T231145Z`, coreSHA `f7c7cc0df9e3c69184f18c017a3681bf14831555a57afe39f73c96d63cab55f1`. Reviewed combined manifestSHA `8d2cd3886a34c6e06f22af5744c55529918fcb61328d6e882f89c4f299e69777`; producerSHA `d98f7fce2a15bbd003907f79ab6dddd8b4de475b6587176c39318e92acc4b553`; trackerSHA `bccec255b83e9f2cceb2b69022cba00a72370869763b7df3e43d2a2f628f2e57`. No commits, original asset changes or safety-rule changes.

## Excluded startup generations

- `rtx_async_ab_A_20261110T233405Z`, domain185: strict constructor attestation rejected the pre-existing explicit output-coordinate token overrides because the frozen USD had these attributes unauthored. No physics/input/action. Actual runtime repr was not archived. Strict token/type/schema contract was subsequently fixed and reviewed; no validation was skipped.
- `rtx_async_ab_A187_20261110T235043Z`, domain187: passed constructor, then native Kit `corrupted double-linked list` during first RTX render.19 clock/IMU samples, no cloud/RGB. Owned supervisor stopped the crashed process group; all descendants exited. Source archive lacks terminal summary and is incomplete. This is not a performance sample.
- `rtx_async_ab_B188_20261110T235403Z`, domain188: native correctly subscribed typed complete-scan topic, but launcher PYREADY still checked raw topic and timed out. No capture/GPU/input/action began. All owned processes exited. The original owner later fixed only this pre-capture check, with off branch preserved and on branch checking the exact typed DDS subscription. See valid B below.

## Valid A: legacy native admission, typed producer also active

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_async_ab_A_retry_20261110T235206Z`. Domain189. Native `common.complete_scan_watermark=false`; producer envelope enabled. New generation, no prior map reused.324 frozen source/build/helper hashes were checked at launch.

Actual16.8sim seconds, loop31.10546wall seconds, loopRTF0.54010. Original terminal `interactive_pose_lost` occurred at11.715sim, followed by original parking/cleanup. Capture exit0. This label is not evidence that native VIO stopped: all124 LIO/post-VIO pairs were continuous from4.3–16.6s, including two±1ns floating conversions preserved exactly.

All840 controller commands and840 policy body commands are zero. In the6sim→preterminal window,285 normal ticks include2 idle tracking returns (8.415/8.435sim). Frozen control flow proves these idle returns occur after the full perception gate; no local action or direction reread race exists in this zero-action run. Thus gate-pass duty is2/285=0.70%, not motion duty. The control journal does not carry success `perception.validated` fields. The independent observation snapshots retain last actual validation at8.435sim/source8.199999999s; its later current=false state is historical, never relabeled fresh.

Of100 published perception samples,10 were accepted after compute and8 were still fresh at the independent post-publication read. Depth source-age medians were0.310sim seconds and0.41763wall seconds.90 producer rejections were `post_compute:cloud_sim_expired`. Native post-init source age at VIO finish versus independently received /clock: median0.14000sim, p95 0.21500sim; this is not instantaneous physics-now. Median LIO estimation7.22ms, VIO processing3.83ms. Native queue failure is empty and orderly drain has zero pending/inflight/uncommitted packets.

## Full input and measurement integrity

Source publisher archive7218 records is complete:3360 clock,3360 IMU,166 raw PointCloud2,166 typed envelopes and166 original RGB images. All stored CDR SHA/offset/length checks, deserialization, original headers/fields and raw data SHA checks passed. All166 typed embedded clouds equal their corresponding raw clouds. Certificate sequence1–166 is continuous, E=S+100ms, coordinate frame start=S+15ms; coordinate phase is not a rewritten source stamp. Native ingress separately confirms all166 raw scans,166 RGB and3360 IMU inputs. Publisher CDR is equivalent reserialization after publish, not original DDS padding.

The independent best-effort raw=True recorder is NOT DDS-complete: it received134/166 raw scans,70/166 typed envelopes and151/166 RGB. Clock and IMU3360 each are complete. Every received input is semantically identical to a publisher input; no unexplained extra records. Its archive queue had zero drops/errors, so missing DDS delivery must not be mislabeled queue overflow or native input loss. Full original inputs remain available at the publisher side for later CPU replay. Receiver CDR preserves its own RMW-delivered bytes.

Source synchronous archive hooks total0.34929wall seconds across the run. Raw/typed/RGB per-call median0.253/0.216/0.219ms, each peak below0.374ms. Source writer queue peak8.06MB/4records, zero overflow; terminal receipt hook flushed once before native fast shutdown in55.7ms. Total run archive about1.3GiB. This is instrumented performance, not a no-overhead claim.

Heavy native detail remains115–165, outside the20sim budget; kind4 count0. Light native ingress trace3.24MB, sync/boundary ledger1.232MB with1090 sync rows, no row gaps and a final summary. The existing8MiB cap was not reached. Watermark valid alone never proves health; stderr, pipeline failure and final summaries are also retained. The earlier proposed8–10GB heavy-detail scheme was not adopted. Old .5/nonrealtime terminal data occurred during heavy115–165 detail logging and is not a single-variable baseline for this A/B.

Evidence: `coarse_audit.json`, `full_cdr_audit.json`, `native_ab_audit.json`, `native_ab_timing_rows.json`, source/received CDR directories, native boundary/ingress traces, original capture/controller journals, video, map, stderr and lifecycle files. The valid B below completed under the reviewed readiness correction. Actual navigation is a separate experiment; no motion occurred in either A/B arm.


## Valid B190 and bounded A/B conclusion

Run: `/media/hyh001/新加卷/6.project_data/embodied-navigation/experiments/rtx_async_ab_B190_20261111T000856Z`, domain190. Native watermark=true, same producer/tracker/native binary and steady-state recording as A189. Final combined readiness-v2 manifestSHA `85c17501a54d75b995ac380e290ba2a356dc62e858782c0ba7da443c1d3604c3`; launcherSHA `4f9f4317d13d8bcf985f3d516d5216f1e84276895ab30574f9594a94451d95ac`. Launcher helper revision differs only before capture admission. Startup/readiness times and script bytes are not identical and are not performance comparisons.

B completed20sim/36.05512wall, loopRTF0.55471, exit0, all owned processes gone. Original interactive motion budget terminated commands at15sim, followed by parking through20sim. No source-loss terminal occurred. All1000 controller and1000 policy body commands were zero.

Common active control window6≤sim<11.715 contains285 ticks per arm. Original idle full-gate passes rose fromA2/285(0.70%) toB76/285(26.67%). B all active6–15sim passed124/450(27.56%). This is a measured improvement, not all-tick availability, motion or a hardware realtime claim. The remaining holds are preserved.

Common **perception source-stamp** window6≤stamp<11.715 is a separately labeled population: A45 published samples,3 postcompute accepted and3 independent postpub fresh; B44 samples,all44 accepted and fresh. Median source sim age fell0.31000→0.21000s and depth wall age0.44247→0.23673s. Controller odometry source-age median over the common control window fell0.235→0.135s. Full-run B postcompute121/124 and postpub115/124; full-run rates have different run lengths and must not replace the common-window comparison.

The native certified path actually received198 typed scans and committed a continuous watermark fromS=.1s throughE=19.9s, sequence198. Native198 image inputs and4000 IMUs also arrived;157 LIO/post-VIO output pairs are continuous from4.3–19.9s, all normal flags. Full stderr contains no native watermark fault, thread-safe queue failure is empty and orderly drain has no uncommitted packets. `valid=true` is not used alone as health proof.

Early release is demonstrated by actual publication/selection wall clocks: in the58 common-window LIO selections, B29 occurred before the next nominal raw scan was even published (median positive lead0.27242wall seconds); A0/58 did. The certificate covered the image target without waiting for a later scan’s last retained point. The corrected certificate-mapping audit preserves authoritative native target integer ns without rounding, and finds the first actual certificate S<T≤E separately from the legacy first scan whose last retained return covers T. All58 B targets were covered by the committed watermark. The older nominal same-header association is not used to identify B’s required input. Full-run cumulative WaitPointCoverage wasA1617 vsB0; these attempt counts are not duration measurements.

B source archive8594 records is complete (4000 clock/4000 IMU/198 raw/198 typed/198 RGB); all CDR/semantic/raw-SHA checks and198 typed-to-raw joins passed. Native inputs are complete independently. The Python raw=True receiver again has separately reported DDS gaps; it does not establish native loss or double-ended completeness. Source callback hook wall sum and all queue/write statistics are retained in `full_cdr_audit.json`. B native sync ledger has final summary/no row gaps and remains under its8MiB cap; heavy kind4 remains0.

Additional evidence: `coarse_ab_comparison.json`, `common_window_age_audit.json`, `early_release_publication_audit.json`, full CDR and native audits. This A/B supports a conservative source-age improvement under this instrumented static scene, not a claim of completed multi-room exploration,1m/s navigation or full realtime readiness.


## Retraction and corrected scan dependency mapping

The preserved original `native_ab_audit.json` fields `same_stamp_lidar_arrival_minus_image_s` and `lio_start_after_latest_same_stamp_input_s` join equal numeric raw scan header S=T to image/native LIO target T. **That is not the certified B dependency. Those fields must not be used as B critical-path latency or delay evidence.** Their earlier interpretation is retracted; original audit files are unchanged. A negative arrival difference alone is not negative processing latency or evidence of improvement.

The added `scan_dependency_mapping_audit.json` uses the actual native `pending_pair.source_stamp_ns`, original full typed certificate intervals and actual retained-return endpoints. It separates (1) B’s first complete certificate with S<T≤E, usually S=T−0.1s, from (2) the old path’s first raw scan whose last retained point reaches T, usually header S=T. It preserves each source’s native receipt wall time, image receipt wall time, selection time and publication call time. No source stamp is adjusted, including floating conversion ±1ns differences.

All58 B common-window targets have committed certificate coverage. Selection occurred before the **legacy-needed next scan** was published for29/58 targets, positive lead median272.419ms; A0/58. This is the bounded early-release evidence, not a claim that B selected before its required certified input arrived. The actual mode-required scan arrived relative to its image at median A+133.254ms versus B−16.901ms. The latter means the certified dependency was already present before that image, not negative compute latency. The remaining gate availability limitation is separate: B’s44 common source-window post-publication samples are all fresh, but normal control gates pass76/285 ticks, and freshness margin still needs its own source-by-source audit.
