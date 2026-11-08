# plan_env：滚动占据与机身碰撞

`GridMap`把当前LiDAR云和sensor/body位姿转换为滚动3D log-odds占据及膨胀层，供A*、B样条碰撞及安全检查读取。它不加载RGB档案，也不维护动态对象实例/速度预测。

## 核心处理

`src/grid_map.cpp::initMap/updateSlidingMap/raycastProcess`分配并维护环形buffer：double占据log-odds、char膨胀层、重叠膨胀计数、每帧射线标签/队列。真实表面hit、射线穿越miss累加到概率对数；滑动保留重叠部分、清理新入窗口。

射线修复保证每帧标签清零、hit/miss去重、hit优先；超量程/边界截断不生成虚假占据终点，不因共享体素提前结束其他射线。`raycast.cpp`提供网格遍历。

## 参数与输出

当前覆盖分辨率.08m、范围10×10×5m、局部半范围5×5×2.5m。默认hit=.85、miss=.30、概率夹紧[.12,.98]、occupied=.80、射线最长5m。`getInflateOccupancy`在前后±.18m的两圆柱中心按yaw查询，r=.25m、Z clearance上下各.12m。

地图更新/可视化请求周期50ms wall，但与其他planner回调共享单线程；不是20Hz实测保证。ROS输入remap与使用见[SCAN总览](../../README.md)。

调试先查云是否world坐标、同代且新鲜、sensor pose与地图窗口是否一致，再看ray hit/miss及膨胀。动态清障依赖新射线清空证据及Python实际clear窗；静态图排除动态物不代表运行局部图也排除它。该近似不证明足端落点或接触稳定。公式见[技术报告](../../../../docs/technical_report.zh-CN.md)。
