# K20 感知落脚规划实施计划表

日期：2026-07-31

状态：执行中；S0、S1、S2 已通过，等待 Beauhao 确认后进入 S3

设计基线：`2026-07-30-k20-perceptive-foothold-planning-design-implementation-ready.md`

审核输入：`2026-07-31-k20-perceptive-foothold-planning-implementation-ready-review.md`

## 1. 目标和执行原则

本计划只实施以下四项可独立关闭的增强：

1. 当前摆动腿第一触地点的 Raibert XY 修正。
2. K20 固定规划髋坐标系下的候选运动学惩罚。
3. 摆动后半程的落脚决策冻结。
4. 基于实时 `elevation` 的摆动净空增高。

执行原则：

- 四项开关在开发、单元测试和初始 Gazebo 阶段均默认关闭。
- 配置缺失或四项关闭时，输出必须保持现有行为。
- 严格按 S0–S14 串行执行；上一阶段未通过，不开始下一阶段。
- 每阶段只处理一个关注点，形成一个可独立审阅和回退的变更集；未经 Beauhao 明确要求不自动创建 Git commit。
- 每阶段依次执行：补充定向测试、确认测试因缺少本阶段行为而失败、实现最小代码、定向测试通过、包级全量回归通过、`git diff --check` 通过。
- 如果新增测试一开始就通过，先确认它是否真正覆盖目标行为，不能把无效测试当作阶段完成。
- 测试失败时只修复本阶段引入的问题；发现前置设计问题则停止、记录证据并更新计划。
- 后续新增或修改的接口、数据结构、关键算法、坐标系/单位、状态生命周期、回退路径和安全边界必须同步添加准确的中文注释；接口优先使用 Doxygen，非显然实现使用行内注释，简单赋值不机械逐行注释。
- 修改代码行为时必须同步更新受影响的既有中文注释；与实现不一致或已经失效的注释视为阶段回归。
- projection、polygon、历史滤波值和冻结 owner 按同一事务提交，禁止部分更新。
- 不修改 MPC 动力学、求解器、WBC、状态估计和硬件接口。
- 发现必须修改“预计不修改”文件时，先更新本计划并说明原因。

## 2. 审核修正的落地决策

| 编号 | 审核问题 | 本计划的确定做法 | 验收位置 |
|---|---|---|---|
| C1 | 摆高端点来源未指定 | 路径 XY、`h_lift`、`h_touch` 和 `maxHeightSequence` 全部取自 `modifyProjections()` 后、传给同一次 `getHeights()` 的 `projections` 副本 | S12、T5 |
| C2 | 冻结恢复可能只写一个 phase | 正常提交和冻结恢复都覆盖同一接触事件的全部 phase 下标，保持现有 `lastStandMiddleTime` 去重语义 | S8–S9、T4 |
| C3 | 冻结快照含冗余真值 | 删除 `planeTransformToWorld` 和 `valid`；不保存未被消费的 `projectionCost`，恢复 projection 时将 `cost` 显式初始化为 `0.0` | S7、T4 |
| C4 | 在线 layer 命令组合脆弱 | topic 命令只确认 publisher 和 `elevation` layer；有限样本由运行时 `terrainClearanceActive`、样本计数和单元测试验证 | R0a–R0b、S11–S12、T5 |
| C5 | owner 清理测试不可直接断言 | 用 `weak_ptr`、旧 raw 地址和新输出联合验证提交顺序；释放旧 owner 后只检查过期状态，不解引用旧 raw 指针 | S10、T4 |

## 3. 依赖关系和里程碑

```text
S0 → S1 → S2 → S3 → S4 → S5 → S6 → S7 → S8 → S9
                                                 ↓
S14 ← S13 ← S12 ← S11 ← S10

R0a 可与 S1–S12 并行；R0b 必须在 S12 后执行。
S14 → G0 → G0.5 → G1 → G2 → G3
R0a + R0b ──────────────┘（只门禁 G1 的地形净空分支）

G3 → H0 → H1 → H2 → H3 → REL
```

- S0–S14：严格串行的离线修改与测试阶段。
- R0a：只确认实时 publisher 和 `elevation` layer，可与离线阶段并行。
- R0b：用 S12 新增的诊断确认有限样本确实被消费；阻塞地形净空进入 G1。
- G0–G3：Gazebo 基线、标定和场景验收。
- H0–H3：K20 实机低速验收。
- REL：仅开启已经通过对应门禁的功能。

## 4. 每阶段固定执行流程

| 步骤 | 动作 | 必须留下的证据 | 未满足时 |
|---|---|---|---|
| A | 确认上一阶段为绿色，并记录当前相关文件 diff | 上一阶段定向测试、包级测试和 `diff --check` 结果 | 不开始本阶段 |
| B | 只增加本阶段测试或 fixture | 测试名称与目标行为一一对应 | 缩小测试范围 |
| C | 运行定向测试，确认预期失败 | 断言失败或预期的缺少本阶段 API 编译失败；不能是环境故障或无关回归 | 先修测试环境或测试本身 |
| D | 编写满足测试的最小生产代码并同步补充中文注释 | diff 只涉及本阶段允许文件；新增/修改语义的接口、状态、关键逻辑和回退路径均有准确中文注释 | 回退无关修改或补齐注释 |
| E | 重新运行定向测试 | 本阶段新增测试全部通过 | 停止并修复 |
| F | 运行 `legged_perceptive_interface` 全部测试 | 原有和此前阶段测试全部通过 | 停止并修复回归 |
| G | 检查四项开关、中文注释与 diff | 无意外启用；注释与实现一致；`git diff --check` 无格式错误且无范围扩大 | 不进入下一阶段 |
| H | 更新本表状态和阶段记录 | 写明修改文件、测试结果和遗留风险 | 阶段不算完成 |

每阶段结束后先向 Beauhao 汇报结果，再进入下一阶段。Gazebo 和实机阶段同样逐项执行，不把多个开关同时首次开启。

## 5. 串行修改与测试表

| ID | 本阶段唯一目标 | 允许修改 | 先写的定向测试 | 进入下一阶段的硬门禁 | 状态 |
|---|---|---|---|---|---|
| S0 | 固化未增强基线 | 测试记录，不改生产逻辑 | 运行现有两个测试目标；记录增强前基线和现有 projection/摆高覆盖缺口 | 当前构建和全部现有测试通过；工作树相关差异已记录 | [x] |
| S1 | 只加入配置结构、默认值和数值校验 | `PerceptiveLeggedInterface.cpp`、配置声明/加载测试；暂不把参数接入算法 | 缺配置、全默认关闭、NaN/Inf、负值和 `[0,1]` 越界 | 配置缺失时启动行为不变；四项功能仍不可改变输出 | [x] |
| S2 | 只完成 contact name→HAA 解析和左右派生 | 配置映射、Pinocchio 查询、对应测试 | 默认 LF/RF/LH/RH、打乱 contact 顺序、缺项、未知项、重复 joint、静态 y 近零 | 映射与数组顺序无关；非法映射只在相关功能开启时拒绝启动 | [x] |
| S3 | 只实现 Raibert 数学纯函数 | selector 内部纯计算和测试；不接候选流程 | 零/正/负速度误差、二维模长限幅、z=0、非有限输入 | 定向测试通过；开关关闭时生产输出完全未变 | [ ] |
| S4 | 只把 Raibert 接入第一触地事件和历史提交 | `ConvexRegionSelector.h/.cpp`、selector 测试 | 当前摆动腿第一事件、后续事件不修正、死区/低通、候选失败不更新历史 | T2 全通过；Raibert 单独开启和关闭回归均通过 | [ ] |
| S5 | 只构造固定规划髋坐标和左右方向 | Pinocchio/selector 辅助逻辑及测试；惩罚权重保持不生效 | 四腿 HAA 根变换、LF/LH 与 RF/RH 方向相反、HAA 原生零旋转 | 不按腿下标或世界 y 判断左右；候选选择尚未改变 | [ ] |
| S6 | 只接入运动学惩罚 | selector 候选代价和测试 | 正常、阈值、过伸、内跨、触地开始/结束累加、非有限回退 | T3 全通过；Raibert 回归通过；两个开关可独立工作 | [ ] |
| S7 | 只建立最小冻结快照和稳定 owner | `ConvexRegionSelector.h/.cpp`、owner 单测；暂不实现相位冻结 | 完整 `PlanarRegion` 深拷贝、重建 projection、冗余字段不存在、`cost=0.0` | `regionPtr == frozenRegion.get()`；变换、边界和 bbox 完整一致 | [ ] |
| S8 | 只实现接触事件匹配、全部 phase 覆盖和事务提交 | selector 事件/phase 逻辑及测试；暂不按 `freezePhase` 锁定 | 时间容差内/外、同一事件多 phase、正常候选失败不部分提交 | 正常提交覆盖事件全部 phase；旧 owner 在新输出提交前不释放 | [ ] |
| S9 | 只实现 `freezePhase` 状态机和清理条件 | selector 冻结逻辑和 timing 测试 | 阈值前后、首次观察已过阈值、小幅/完全失配、新事件、步态切换、当前全局 STANCE | 冻结恢复覆盖全部 phase；后半程不重新查平面；所有清理条件可断言 | [ ] |
| S10 | 只完成 owner 生命周期端到端验证 | 优先只改测试；生产代码仅在测试暴露缺陷时最小修复 | `weak_ptr` 提交顺序、地图整体替换、真实 `PerceptiveLeggedPrecomputation::request()` | T4 全通过，无悬空 raw 指针；S3–S9 全量回归通过 | [ ] |
| S11 | 只实现 `elevation` 路径采样纯逻辑 | selector/独立 helper 和地形测试；暂不调用三参数 planner | 平/上下坡、路径中台阶、路径外障碍、NaN、全无效、地图外、尖峰、同栅格 | 只读取有限 `elevation`；回退均得到 `h_adapt=0`；不读取占位层 | [ ] |
| S12 | 只把同源摆高接入 ReferenceManager | `PerceptiveLeggedReferenceManager.h/.cpp` 及其测试 | 验证路径 XY、`h_lift`、`h_touch` 来自 `modifyProjections()` 后同一副本；三参数 update；不重复加 `swingHeight` | T5 全通过；功能关闭/回退与旧二参数结果一致；R0b 诊断可用 | [ ] |
| S13 | 只做开关组合和跨功能回归 | 测试、必要的最小缺陷修复 | 16 种开关组合做参数化回归；运行级重点覆盖全关、单开、冻结+净空和四项全开 | T1–T6 全通过；全关等于 S0；P1 和非感知配置不受影响 | [ ] |
| S14 | 离线实现封板 | 文档、测试注册和必要配置；不新增功能 | 完整构建、全包测试、`test-result --verbose`、`git diff --check`、文件范围审核 | 无失败、无新增非有限值、无未说明范围扩大；才允许进入 Gazebo | [ ] |
| R0a | 确认实时地图接口 | 不改代码 | topic 存在且 layers 输出含 `elevation` | 结果记录完成；失败不阻塞 S1–S14，但净空保持关闭 | [ ] |
| R0b | 确认有限 elevation 被实际消费 | S12 诊断；不改变地图生产链 | 有限样本数大于零，实际采样周期 `terrainClearanceActive=true` | 通过后地形净空才允许进入 G1 | [ ] |

## 6. 仿真、实机与发布门禁

| ID | 阶段 | 依赖 | 环境 | 修改/测试内容 | 完成标准 | 状态 |
|---|---|---|---|---|---|---|
| G0 | Gazebo 基线 | S14 | Gazebo 场景与诊断 | 四项关闭，记录 MPC 频率、求解时间、足端高度、落脚变化和安全故障 | 可稳定复现且与 S0 行为一致 | [ ] |
| G0.5 | 阈值与 margin 标定 | G0 | Pinocchio FK、Gazebo | 惩罚权重为零只发布诊断；完成正常步态和 FK 扫描；检查 elevation 后处理与 margin 叠加 | 得到有数据依据的 `nominalLegExtension` 和 clearance margin；正常步态不持续受罚 | [ ] |
| G1 | 平地逐项开启 | G0.5；净空还需 R0a、R0b | Gazebo 平地 | 按 Raibert→运动学→冻结→净空顺序启用；0–0.20 m/s；150 N/0.20 s 推扰；全开 60 s | 无摔倒、NaN、跨平面冻结或持续超时；规划耗时增幅 ≤20% | [ ] |
| G2 | 缓坡 | G1 | 8° 上下坡 | 0.15 m/s Trot，检查侧别、冻结和摆高 | 所有 Gazebo 通用标准通过 | [ ] |
| G3 | 低台阶 | G2 | 0.05 m 台阶 | 0.10 m/s Trot；注入 elevation 缺失故障 | 足端参考高于路径障碍；缺层时 inactive 并节流告警 | [ ] |
| H0 | 实机静态检查 | G3 | K20、急停、保护绳 | 检查 contact 映射、规划髋方向、数据有限性、owner 和摆高诊断 | 静态与原地抬腿均无异常 | [ ] |
| H1 | 实机平地 | H0 | K20 平地 | 0.10 m/s 连续 20 s，逐项开启后最高 0.20 m/s | 无终止条件，关闭单项可立即回退 | [ ] |
| H2 | 实机缓坡 | H1 | 初始 5° 缓坡 | 0.10 m/s Trot | 无拖脚、跨平面切换或误差持续扩大 | [ ] |
| H3 | 实机低台阶 | H2 | 0.03–0.05 m 台阶 | 速度不高于 0.10 m/s，先 0.03 m 后 0.05 m | 无连续拖脚或碰撞，净空诊断与实际地形一致 | [ ] |
| REL | 发布和默认值决策 | H3 | K20 感知配置、文档 | 汇总参数、测试和回退记录；只对已验收功能决定是否默认开启 | 所有证据可追溯；未验收功能继续默认关闭 | [ ] |

G1 内部也必须串行：先只开 Raibert 并测试，再只增加运动学惩罚，再增加冻结，R0b 通过后最后增加地形净空；每次新增一项后重新执行平地测试，不允许首次直接四项全开。

## 7. S7–S10 冻结实现约束

最小快照字段：

```text
FootholdDecisionSnapshot
{
  contactStartTime
  contactEndTime
  filteredHeuristicXY
  projectionPositionInTerrainFrame
  projectionPositionInWorld
  convexPolygon
  touchdownHeight
  frozen
  frozenRegion: shared_ptr<const PlanarRegion>
}
```

必须保持以下顺序：

1. 在临时对象中计算并验证候选 projection、polygon 和事件覆盖的 phase 下标集合。
2. 对所选 `PlanarRegion` 做完整值拷贝，建立 `shared_ptr<const PlanarRegion>`。
3. 用 `frozenRegion.get()` 构造所有目标 phase 的 projection，并将 `cost` 初始化为 `0.0`。
4. 一次性替换该事件对应的 projection、polygon、历史值和快照 owner。
5. 确认新输出不再引用旧 raw 地址后，才释放旧快照。

禁止：把冻结平面追加回 `planarTerrain_.planarRegions`、冻结时重新选择“相似平面”、先清 owner 再覆盖 projection，或只恢复事件中的单个 phase。

## 8. S11–S12 摆高数据流

`PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner()` 对每条腿固定按以下顺序执行：

```text
projections = selector.getProjections(leg)        // 值拷贝
modifyProjections(..., projections)               // 修正当前支撑端点
getHeights(contactFlags, projections)             // 得到 h_lift / h_touch
sampleTerrainProfile(contactFlags, projections,   // 同一份 projections
                     h_lift, h_touch)
build maxHeightSequence
swingTrajectoryPlanner.update(modeSchedule,
                              liftOffHeightSequence,
                              touchDownHeightSequence,
                              maxHeightSequence)
```

回退规则：

- 功能关闭、缺少 `elevation`、没有有限样本、地图外、异常查询或同栅格短路径：`h_adapt=0`。
- NaN 栅格只跳过该样本；全部无效才回退。
- `terrainClearanceActive` 仅在本周期实际使用至少一个有限 `elevation` 样本时为 true。
- `maxHeightSequence=max(h_lift,h_touch)+h_adapt`，不得包含既有 `swingHeight`。

## 9. 单元测试覆盖表

| ID | 对应阶段 | 测试范围 | 必须断言 | 建议落点 | 状态 |
|---|---|---|---|---|---|
| T1 | S1–S2 | 配置兼容 | 缺配置/全关闭保持旧输出；contact 顺序打乱不改变映射；非法映射拒绝启动 | focused settings 测试；必要时新增测试目标 | [ ] |
| T2 | S3–S4 | Raibert | 零/正/负误差、第一触地点限定、XY 模长限幅、z=0、死区/低通、非有限回退、失败不提交历史 | selector 测试 | [ ] |
| T3 | S5–S6 | 运动学 | 四腿侧别、左右内跨相反、HAA 原生零旋转、起止两次评分、阈值边界和非有限回退 | selector + Pinocchio fixture | [ ] |
| T4 | S7–S10 | 冻结与 owner | 阈值前后、首次晚观察、时间容差、全部 phase 覆盖、地图替换、Precomputation 真消费、事务释放顺序 | `test_convex_region_selector_timing.cpp` 及 focused owner 测试 | [ ] |
| T5 | S11–S12 | 地形摆高 | 平/上下坡、中间台阶、路径外障碍、NaN/全无效/地图外/尖峰/同栅格、同源端点、无重复 swingHeight | `test_perceptive_reference_manager.cpp` | [ ] |
| T6 | S13–S14 | 组合与回归 | 四开关逐项及组合；全关闭等于 S0；P1/非感知配置不受影响 | interface 集成测试 | [ ] |

T4 的 owner 生命周期断言顺序：

1. 保存 `oldRaw = oldSnapshot.frozenRegion.get()`，并创建指向同一对象的 `weak_ptr`。
2. 构造并提交下一组输出。
3. 遍历新 `feetProjections_`，断言没有 `regionPtr == oldRaw`。
4. 旧 snapshot owner 尚未释放时，断言 `weak.expired() == false`。
5. 释放旧 snapshot 及测试中的其他强引用，断言 `weak.expired() == true`。
6. 第 5 步之后不得解引用 `oldRaw`。

地图替换测试还必须在清空原地图后调用真实 `PerceptiveLeggedPrecomputation::request()`，断言足端约束参数有限且与冻结前一致。

## 10. 每阶段构建和验证命令

每个 S 阶段至少执行一次定向测试和一次包级全量测试。以下以现有测试目标为例；新增 focused target 时只替换 `-R` 后的正则：

```bash
colcon build --packages-select legged_perceptive_interface \
  --cmake-args -DBUILD_TESTING=ON

# Red：只运行本阶段目标，保存预期失败证据
colcon test --packages-select legged_perceptive_interface \
  --ctest-args -R test_convex_region_selector_timing --output-on-failure

# Green：最小实现后重复同一条定向测试
colcon test --packages-select legged_perceptive_interface \
  --ctest-args -R test_convex_region_selector_timing --output-on-failure

# Regression：运行包内全部测试
colcon test --packages-select legged_perceptive_interface \
  --event-handlers console_direct+
colcon test-result --verbose
git -C /workspace/src/legged_perceptive diff --check
```

S14 再增加 `legged_perceptive_controllers` 构建，确认 K20 配置所在包无集成问题：

```bash
colcon build --packages-select \
  legged_perceptive_interface legged_perceptive_controllers \
  --cmake-args -DBUILD_TESTING=ON
```

R0a 在线 layer 检查：

```bash
ros2 topic list | rg '^/convex_plane_decomposition_ros/planar_terrain$'
timeout 10s ros2 topic echo \
  /convex_plane_decomposition_ros/planar_terrain \
  --once --field gridmap.layers
```

人工确认 layer 输出包含 `elevation` 且命令未超时。不要重新加入同时使用 `--field` 和访问完整消息 `m.gridmap` 的 `--filter`。

R0b 在 S12 完成后执行：通过运行时诊断确认有限样本数大于零，并且一次确实使用地形剖面的周期中 `terrainClearanceActive=true`。R0a 或 R0b 失败均保持净空关闭。

## 11. 文件范围

预计修改：

- `legged_perceptive_interface/include/legged_perceptive_interface/PerceptiveFootholdPlanningSettings.h`
- `legged_perceptive_interface/src/PerceptiveFootholdPlanningSettings.cpp`
- `legged_perceptive_interface/include/legged_perceptive_interface/ConvexRegionSelector.h`
- `legged_perceptive_interface/src/ConvexRegionSelector.cpp`
- `legged_perceptive_interface/include/legged_perceptive_interface/PerceptiveLeggedReferenceManager.h`
- `legged_perceptive_interface/src/PerceptiveLeggedReferenceManager.cpp`
- `legged_perceptive_interface/src/PerceptiveLeggedInterface.cpp`
- `legged_perceptive_controllers/config/k20/task.info`
- `legged_perceptive_interface/CMakeLists.txt`，仅在新增测试目标时修改
- `legged_perceptive_interface/test/` 下对应测试

预计不修改：

- `PerceptiveLeggedPrecomputation.h/.cpp`：仅作为 T4 的真实消费方。
- `SwingTrajectoryPlanner.h/.cpp`：复用现有三参数 `update()`。
- `PerceptiveController.cpp`、`PlanarTerrainReceiver.cpp`：不改变地图接收和 SDF 策略。
- MPC、WBC、状态估计和硬件接口。

## 12. 完成定义

- [ ] S0–S14 严格按顺序完成，每阶段均有 Red、Green、Regression 和 diff 证据。
- [ ] T1–T6 全部完成，任一失败都未被带入下一阶段。
- [ ] 四项功能均可独立关闭，缺配置和全关闭与旧行为一致。
- [ ] 冻结恢复覆盖同一事件全部 phase，所有 raw `regionPtr` 始终有存活 owner。
- [ ] 地图整体替换后，真实 Precomputation 仍能安全消费冻结 projection。
- [ ] 摆高路径 XY、`h_lift`、`h_touch` 和中点基准来自同一份修改后 projection。
- [ ] R0a、R0b 通过后才在 G1 开启地形净空；失效时自动回退固定摆高。
- [ ] G0–G3 和 H0–H3 达到门禁，未触发终止条件。
- [ ] 参数、测试证据、已知回退和最终默认值均记录完成。

## 13. 阶段执行记录

### S0：固化未增强基线（已通过）

执行时间：2026-07-31 15:48:10 China/Shanghai

基线：

- 分支：`bug/perc_mpc_ext`
- 提交：`a89129619a846c9b4fdbd051db127ab596315086`
- 执行前 `git status --short` 无输出。
- 未修改生产代码、配置或测试代码；构建产物只写入工作区的 `build/`、`install/` 和 `log/`。

结果：

| 检查 | 结果 |
|---|---|
| `colcon build --packages-select legged_perceptive_interface --cmake-args -DBUILD_TESTING=ON` | 通过，1 个包成功 |
| 定向运行 `test_convex_region_selector_timing` | 通过；非 ASan 构建执行 1 个用例，ASan 专用用例因未定义 `__SANITIZE_ADDRESS__` 未编译 |
| 定向运行 `test_perceptive_reference_manager` | 通过；执行 1 个用例 |
| `colcon test --packages-select legged_perceptive_interface --event-handlers console_direct+` | 通过，2/2 个 CTest 目标、0 失败 |
| 包级 `colcon test-result --test-result-base /workspace/build/legged_perceptive_interface --verbose` | 通过，0 errors、0 failures、0 skipped |
| `git diff --check` | 通过 |
| 构建和测试后的 `git status --short` | 无输出 |

结果解释：

- 无参数的全工作区 `colcon test-result --verbose` 会读到 `/workspace/build/legged_controllers` 在 2026-07-29 留下的 7 个历史失败；它们不属于本次测试包。后续阶段统一使用包级 `--test-result-base`，同时以本阶段刚执行的 CTest 输出为准。
- 当前两个测试目标只覆盖 selector 末端 stance 索引和 ReferenceManager 输入重采样，尚未锁定完整 projection 与摆高输出。S1 必须在配置开始影响任何算法之前，先加入“配置缺失/四项关闭保持旧行为”的回归断言；该覆盖缺口不得带入 S3 的算法实现。

结论：S0 通过。按串行规则停在此处，不自动进入 S1。

### S1：配置结构、默认值和数值校验（已通过）

执行时间：2026-07-31 16:22:45 China/Shanghai

Red：

- 先新增 `test_perceptive_foothold_planning_settings` 及 CMake 测试目标，未添加生产 API。
- `colcon build --packages-select legged_perceptive_interface --cmake-args -DBUILD_TESTING=ON` 按预期失败，唯一首因是 `PerceptiveFootholdPlanningSettings.h` 不存在。
- Red 与本阶段缺失行为直接对应，不是环境故障或旧测试回归。

最小实现：

- 新增独立 `PerceptiveFootholdPlanningSettings` 头源文件，包含四个默认 `false` 开关和十个设计标量。
- 缺失 `perceptive_foothold_planning` 配置组或任一开关时使用默认关闭；其余标量缺失时使用设计默认值。
- 所有标量必须有限；高度、距离、裕量和权重不得为负；`previousFootholdFactor` 与 `freezePhase` 必须位于 `[0,1]`。
- `PerceptiveLeggedInterface` 只加载并保存 settings，没有传给 `ConvexRegionSelector`、`PerceptiveLeggedReferenceManager` 或 `SwingTrajectoryPlanner`。
- K20 感知 `task.info` 显式加入同一配置组，四项开关全部保持 `false`。
- 删除当前无消费方的公开 getter，只保留后续阶段可在 Interface 内部使用的 protected 成员。

Green 与 Regression：

| 检查 | 结果 |
|---|---|
| S1 focused gtest | 5/5 通过：缺配置默认、显式全关、显式值加载、非有限/越界校验、loader 调用校验 |
| `legged_perceptive_interface` 构建 | 通过 |
| Interface 包级完整回归 | 3/3 CTest 目标、7/7 gtest 用例通过 |
| 包级 `test-result` | 0 errors、0 failures、0 skipped |
| `legged_perceptive_controllers` 构建 | 通过，确认配置所在包可正常构建安装 |
| `git diff --check` | 通过 |
| 新文件行长与尾随空白检查 | 通过 |

说明：

- 两个包构建仅出现仓库既有的 Boost bind deprecated 提示。
- 环境未安装 `clang-format`，命令返回 127 且未写文件；已用行长、尾随空白、编译和 diff 检查替代。
- S1 没有建立 contact name→HAA 映射，也没有使四项算法生效；这些属于后续阶段。

结论：S1 通过。按串行规则停在此处，不自动进入 S2。

### S2：contact name→HAA 解析和左右派生（已通过）

执行时间：2026-07-31 17:12:01 China/Shanghai

Red：

- 先新增 `test_contact_leg_root_resolver`、settings 映射断言和 CMake 测试目标，未添加 resolver 生产 API。
- `colcon build --packages-select legged_perceptive_interface --cmake-args -DBUILD_TESTING=ON` 按预期失败，首因仅为 `ContactLegRootResolver.h` 和 `legRootJointByContact` 尚不存在。
- Red 与 S2 的配置解析及 Pinocchio 映射缺失直接对应，不是环境故障或旧测试回归。

最小实现：

- settings 新增 `legRootJointByContact` 原始映射数组；loader 保留配置顺序和重复 contact，供 resolver 做严格校验。
- 新增 `ContactLegRootResolver`：运动学惩罚关闭时直接返回空结果，不校验缺失或非法的可选映射。
- 开关开启时按运行时 `modelSettings.contactNames3DoF` 顺序解析；拒绝重复运行时 contact、配置缺项/未知项/重复 contact、重复 joint、不存在或非 1-DoF joint，以及不是 floating base 直接子关节的映射。
- 左右侧只由 Pinocchio 中 HAA 相对 base 的静态 y 符号派生；y 必须有限且绝对值大于 `1e-6 m`，不使用数组下标、contact 名称前缀或世界系足端位置。
- `PerceptiveLeggedInterface` 只保存解析结果，尚未传给 `ConvexRegionSelector` 或改变候选代价；K20 配置加入四个显式 contact→HAA 映射，四项功能仍全部为 `false`。
- 新增和修改的接口、数据结构、映射逻辑、回退边界及配置均补充中文 Doxygen 或行内注释。

Green 与 Regression：

| 检查 | 结果 |
|---|---|
| S2 resolver focused gtest | 7/7 通过：关闭态、默认四腿、打乱 contact 顺序、缺失/未知 contact、重复 contact/joint、无效/非腿根 joint、静态 y 近零 |
| S1 settings focused gtest | 5/5 通过，新增映射加载与缺配置空映射断言 |
| Interface 包级完整回归 | 4/4 CTest 目标、18/18 gtest 用例通过 |
| 包级 `test-result` | 0 errors、0 failures、0 skipped |
| 真实 K20 URDF Pinocchio C++ 核验 | 通过：LF/LH y=`+0.08 m`，RF/RH y=`-0.08 m`；四个 HAA 均为 floating base 的直接 1-DoF 子关节 |
| `legged_perceptive_interface` 与 `legged_perceptive_controllers` 构建 | 通过；安装后的 K20 task.info 包含映射且四项开关仍为 false |
| `git diff --check`、S2 新增文件 120 字符行长和尾随空白 | 通过 |

说明：

- 两个包和临时 Pinocchio C++ 核验仅出现仓库既有的 Boost bind deprecated 提示。
- Python 环境未安装 `pinocchio` 模块，独立 Python 核验不可用；已改用同一已安装 Pinocchio C++ 库验证真实 K20 URDF。
- `PerceptiveLeggedInterface.h/.cpp` 存在本阶段之前的 122–135 字符长行，S2 未重排这些无关代码；S2 实际新增/扩展文件单独通过 120 字符检查。
- S2 尚未构造规划髋坐标系、Raibert 数学函数或运动学惩罚；这些分别属于 S5、S3 和 S6。

结论：S2 通过。按串行规则停在此处，不自动进入 S3。
