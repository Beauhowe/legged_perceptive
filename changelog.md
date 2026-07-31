# 2026-07-31 Unreleased

## [added]

### S1：感知落脚规划配置基线

- 新增 `PerceptiveFootholdPlanningSettings`，集中管理 Raibert 反馈、运动学惩罚、晚摆动冻结和地形净空四个独立开关，以及十个设计标量；四项开关默认均为 `false`
- 新增 Boost INFO 配置加载和标量校验：缺失配置组或字段时逐项使用安全默认值；拒绝 NaN、Inf、负长度/时间/权重以及超出 `[0,1]` 的滤波与相位参数
- `PerceptiveLeggedInterface` 在 S1 只加载并保存 settings；K20 `task.info` 显式加入同名配置组，但尚不把任何增强接入落脚流程
- 验证：settings focused 测试 5/5、阶段包级 3/3 CTest 与 7/7 gtest 通过

### S2：contact→HAA 映射和左右派生

- 新增 `legRootJointByContact` 显式配置，以及 `ContactLegRootResolver`；解析结果按运行时 `modelSettings.contactNames3DoF` 排列，不依赖配置顺序或腿数组下标
- 运动学惩罚关闭时跳过可选映射校验；开启时拒绝缺失/未知/重复 contact、重复 joint、不存在或非 1-DoF joint、非 floating-base 直接子关节和静态横向位置近零
- 左右侧只由 Pinocchio 模型中 HAA 相对 base 的静态 y 符号派生；真实 K20 URDF 验证 LF/LH 为 `+0.08 m`、RF/RH 为 `-0.08 m`
- K20 `task.info` 新增 LF/RF/LH/RH 足端到对应 HAA 的显式映射；Interface 只保存解析结果，尚未改变候选代价
- 验证：resolver focused 测试 7/7、settings 5/5、阶段包级 4/4 CTest 与 18/18 gtest 通过

### S3：Raibert 数学纯函数

- 新增 `ConvexRegionSelector::computeRaibertOffset()`，按 `sqrt(h/9.81) * (v_measured-v_desired)` 计算世界系 XY 落脚修正，输出 z 恒为零
- 水平偏移按二维欧氏模长统一限制到 `raibertMaxOffset`，不分别裁剪 x/y；速度、参数或中间结果非有限以及参数为负时安全回退零向量
- S3 只提供无状态纯函数和单元测试，尚未接入第一触地事件、历史滤波或候选提交，四项功能开关保持关闭
- 验证：Raibert focused 测试 7/7、阶段包级 5/5 CTest 与 26/26 gtest 通过

# 2026-06-25 v0.0.1

## [added]

- `legged_perceptive_controllers/config/p1/task.info`：`kalmanFilter` 段新增外部里程计位置融合参数（`externalOdomEnable` / `externalOdomTopic` / `externalOdomPosNoiseX` / `externalOdomPosNoiseY` / `externalOdomPosNoiseZ` / `externalOdomPosGate` / `externalOdomMaxAge` / `externalOdomTimeout`，话题 `/Odometry`）
- `legged_perceptive_controllers/config/p1/task.info`：`kalmanFilter` 段新增地形图高度融合参数（`terrainHeightEnable` / `terrainTopic` / `terrainHeightLayer` / `terrainHeightFallbackLayer` / `terrainComHeight` / `terrainPosNoiseZ` / `terrainPosGate`，默认开启，与感知 MPC 共用 `planar_terrain`）
- `legged_perceptive_controllers/config/p1/task.info`：新增 `enableCsvLogging=true`
- `PlanarTerrainReceiver`：新增 `resolveSdfLayer()`，按首选图层 → `smooth_planar` → `elevation_before_postprocess` → `elevation` 自动回退
- `legged_perceptive_controllers/package.xml`：新增 `elevation_mapping_ros2` 运行时依赖

## [changed]

- `PerceptiveController`：PlanarTerrainReceiver 默认 SDF 图层由 `elevation` 改为 `smooth_planar`
- `PlanarTerrainReceiver`：构造参数 `elevationLayer` 重命名为 `preferredSdfLayer`；仅 SDF 成功构建后才置 `updated_=true`
- `legged_perceptive_description/package.xml`：更新 D435 mesh 说明注释（mesh 已内置，`realsense2_description` 可选）

## [fixed]

- `PlanarTerrainReceiver`：`planar_terrain` 消息不含 `elevation` 图层时 `grid_map::get()` 抛异常导致节点崩溃
- `PlanarTerrainReceiver`：图层全 NaN 或无效时仍标记 `updated_`，污染 MPC 求解器的问题
