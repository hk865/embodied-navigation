# DynaNav 估计器配置

本目录保存办公室 RGB-D/IMU 估计器的显式实验配置，不包含场景资产、策略权重、日志或地图。

- [fastlivo_lio.yaml](fastlivo_lio.yaml)：默认图像关闭的 `ONLY_LIO`，输入静态 RGB-D 点云和 IMU；须显式允许 image-free 模式。
- [fastlivo_livo.yaml](fastlivo_livo.yaml)：RGB 辅助 LIVO，增加 `/dynanav/camera`，禁止回退成 image-free LIO。启用最终 post-VIO pose/status 输出；视觉参数是本轮实验设置，结果须看实际保留的更新与独立评分。
- [camera_rgbd.yaml](camera_rgbd.yaml)：512×512 pinhole 相机参数基准。LIVO launcher 还从实际 capture 读取安装标定并校验参数服务，不把硬编码 fallback 当作标定成功。
- [fastlivo_lio_gravity_estimation.yaml](fastlivo_lio_gravity_estimation.yaml)：仅打开 gravity estimation 的对照覆盖文件；默认仍关闭，已有对照未显示改善。
- [README_LIO.md](README_LIO.md)：LIO/LIVO 启动、真实标定约定、隔离 namespace、post-VIO 评分、构建身份和生命周期修复记录。

入口为 [run_dynanav_lio.sh](../../../scripts/run_dynanav_lio.sh)。它不启动模拟器，也不直接发运动命令；默认 GT 诊断不会使用其估计，显式 `--slam-reference` 路线才按参考来源消费 live LIO 或 dedicated post-VIO pose。完整导航云与静态估计云分别保留，动态图像掩码尚未实现。

已完成的双楼梯旁路对照、live post-VIO 契约和当前闭环状态见[续篇](../../../docs/experiments/dynanav-20261109-continuation.md)，先前失败保留于[历史实验记录](../../../docs/experiments/dynanav-20261109.md)。不要由配置存在或 `--check` 通过推断路线/视觉精度已通过。
