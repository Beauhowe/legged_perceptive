# 2026-06-25 v0.0.1

## [added]

- `legged_perceptive_controllers/config/p1/task.info`：`kalmanFilter` 段新增外部里程计高度融合参数（`externalOdomEnable` / `externalOdomTopic` / `externalOdomPosNoiseZ` / `externalOdomPosGate` / `externalOdomMaxAge` / `externalOdomTimeout`，默认关闭，话题 `/Odometry`）
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
