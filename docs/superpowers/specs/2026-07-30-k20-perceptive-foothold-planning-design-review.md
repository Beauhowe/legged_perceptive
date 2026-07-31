# K20 感知落脚规划设计审核记录

日期：2026-07-30
审核对象：`2026-07-30-k20-perceptive-foothold-planning-design-revised.md`（最终版）
审核结论：可作为实现基线；实现前需补齐第 3 节的一处设计空白和第 4、5 节的四处文档修改

## 1. 审核范围和方法

本文只审核设计文档本身，不修改代码。审核方法：

- 对文档中所有涉及当前代码行为的事实性声明，逐条回到工作区源码核实。
- 对涉及数学推导的部分（π 旋转的内跨方向、`maxHeightSequence` 叠加关系），独立重算验证。
- 追踪文档规定的数据流在现有消费方是否成立，特别是跨 MPC 周期的对象生命周期。

## 2. 代码核实通过的声明

| 文档声明 | 核实位置 | 结果 |
|---|---|---|
| 启动占位地图只含全零 `elevation_before_postprocess` 和 `smooth_planar`，无 `elevation` | `PerceptiveLeggedInterface.cpp:43-46` | 通过 |
| KFE 关节上限为 `-0.698 rad` | `k20.urdf:220` | 通过 |
| KFE 相对 HFE 偏置 `xyz="0 0.145 -0.35"` | `k20.urdf:215` | 通过 |
| `ModelSettings::contactNames3DoF` 不从 `task.info` 加载，为硬编码默认值 | `ModelSettings.cpp:40-62`、`ModelSettings.h:54` | 通过 |
| 硬编码顺序为 `LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT` | `ModelSettings.h:54` | 通过 |
| 四个 HAA 在 URDF 中均为 `rpy="0 0 0"` | `k20.urdf:152/334/516/698` | 通过 |
| 左右腿由 HAA 的 base-y 符号可区分（左 `+0.08`，右 `-0.08`） | `k20.urdf:153/335/517/699` | 通过 |
| 当前 SDF 层解析优先 `smooth_planar` | `PlanarTerrainReceiver.cpp:22-33` | 通过 |
| `SwingTrajectoryPlanner` 自行叠加 `scaling * swingHeight` | `SwingTrajectoryPlanner.cpp:124` | 通过 |
| 二参数 `update()` 内部构造 `max(liftOff, touchDown)` 作为 `maxHeightSequence` | `SwingTrajectoryPlanner.cpp:80-88` | 通过 |
| Raibert 反馈已计算但未用于返回值 | `ConvexRegionSelector.cpp:208-218` | 通过 |
| 候选平面附加评分恒为零 | `ConvexRegionSelector.cpp:131` | 通过 |
| 每周期整体拷贝地图，旧 `regionPtr` 必然失效 | `ConvexRegionSelector.cpp:85` | 通过 |
| OCS2 内跨方向公式 `gravityNormalInHip × UnitX` | `KinematicFootPlacementPenalty.cpp:13` | 通过 |
| OCS2 滤波和时间死区语义 | `SwingTrajectoryPlanner.cpp:383,566-573`（ocs2_switched_model_interface） | 通过 |

独立重算验证通过的推导：

- 第 9.2 节右腿绕 z 旋转 `pi` 后，内跨方向在两侧规划髋坐标系中同为 `[0,-1,0]`，且左右语义均正确（左腿 hip-y 负向为内、右腿 hip-y 负向经 `pi` 旋转映射回 base 正 y 侧，对负 y 侧的髋同样为内）。该构造能绕开四个 HAA 零旋转导致的方向退化。
- 第 11.3 节 `maxHeightSequence = max(h_lift, h_touch) + h_adapt`，`h_adapt = 0` 时与二参数 `update()` 的内部构造完全一致，功能关闭时输出不变。

## 3. 关键设计空白：冻结恢复无法产生有效 `regionPtr`

严重度：高。会导致空指针解引用崩溃，且只在冻结路径被触发时暴露。

### 3.1 问题

`PerceptiveLeggedPrecomputation.cpp:33-45` 是 `getProjection()` 的消费方，直接解引用 `regionPtr`：

```cpp
auto projection = convexRegionSelectorPtr_->getProjection(i, t);
if (projection.regionPtr == nullptr) {  // Swing leg
  continue;
}
...
params.a = polytopeA * p * projection.regionPtr->transformPlaneToWorld.inverse().linear();
params.b = polytopeB + polytopeA * projection.regionPtr->transformPlaneToWorld.inverse().translation().head(2);
```

设计文档第 10.1 节正确禁止在快照中保存 `regionPtr`（因为 `ConvexRegionSelector.cpp:85` 每周期整体拷贝地图，旧指针必然悬空），并要求"恢复冻结结果时应使用快照中的变换、位置和多边形重建本周期输出"。

但文档没有规定重建出的 `PlanarTerrainProjection` 的 `regionPtr` 从哪里来。三种可能的结果都不可接受：

- `regionPtr == nullptr`：`PerceptiveLeggedPrecomputation` 会把该腿当成摆动腿 `continue`，冻结期间足端软约束直接消失。
- `regionPtr` 指向上周期地图：悬空指针，未定义行为。
- `regionPtr` 指向本周期地图中"看起来最像"的平面：等于放弃冻结语义，地图更新仍会改变约束。

### 3.2 建议方案（方案 A）

在快照中增加一个值拥有的 `std::shared_ptr<convex_plane_decomposition::PlanarRegion>`：

- 写入快照时，从当时选中平面深拷贝构造一个独立的 `PlanarRegion` 对象，至少填充 `transformPlaneToWorld` 和 `boundaryWithInset`。
- 该对象由 `shared_ptr` 持有，与 `planarTerrain_.planarRegions` 无任何关联，不受地图替换影响。
- 冻结恢复时，`getProjection()` 返回的 `PlanarTerrainProjection::regionPtr` 指向该 `shared_ptr` 管理的对象。
- 快照清理时释放该 `shared_ptr`。

该方案的优点是不需要修改 `PerceptiveLeggedPrecomputation`，且生命周期由 `shared_ptr` 显式管理，符合文档第 10.1 节"值拥有"的原则。

### 3.3 备选方案

- 方案 B：把快照平面追加回本周期 `planarTerrain_.planarRegions`，让 `regionPtr` 指向它。需要处理四腿并发追加、`getBestPlanarRegionAtPositionInWorld` 会把合成平面当候选、以及容器扩容导致已有 `regionPtr` 失效三个问题。不推荐。
- 方案 C：修改 `PerceptiveLeggedPrecomputation` 使其直接接受平面变换而不经 `regionPtr`。需要把该文件移入"必须修改"列表。接口更干净，但改动范围超出当前设计边界。

### 3.4 建议的文档补充

在第 10.1 节的快照字段列表后补充：

> 快照额外持有 `std::shared_ptr<convex_plane_decomposition::PlanarRegion> frozenRegion`，在写入快照时从当时选中平面深拷贝构造，只填充 `transformPlaneToWorld` 和 `boundaryWithInset`，不引用 `planarTerrain_.planarRegions` 中的任何对象。冻结恢复时，输出的 `PlanarTerrainProjection::regionPtr` 指向 `frozenRegion.get()`，从而在地图整体替换后仍保持有效。快照清理时释放 `frozenRegion`。

对应地在第 14.1 节冻结测试中增加一条：

> 地图对象整体替换并且原容器被清空后，冻结期 `getProjection()` 返回的 `regionPtr` 仍非空、仍指向快照持有的平面，且 `transformPlaneToWorld` 与冻结时刻一致。

## 4. 文件列表漏项

`PerceptiveLeggedPrecomputation.h/.cpp` 既不在第 15 节"必须修改"，也不在"预计不修改"列表中，但它是 `getProjection()` 的唯一 MPC 内消费方，且直接解引用 `regionPtr`。无论第 3 节采用哪个方案，该文件的处置意图都应显式写明：

- 采用方案 A：列入"预计不修改"，并注明"冻结恢复通过快照持有的 `shared_ptr<PlanarRegion>` 保证 `regionPtr` 有效，本文件无需改动"。
- 采用方案 C：列入"必须修改"，并按第 15 节末尾的规则说明原因。

## 5. 措辞歧义（三处）

### 5.1 第 10.3 节末尾"STANCE"

原文：
> 进入计划支撑相、新摆动事件、步态切换、时间超过缓存触地时间容差或 STANCE 后清除旧快照

"STANCE"与"进入计划支撑相"含义重叠；"STANCE"单独出现含义不明（全局模式还是该腿自身状态）。建议改为：

> …步态切换、时间超过缓存触地时间容差，或全局 `modeNumber == STANCE`（四腿同时接触）时清除旧快照。

### 5.2 第 16 节步骤 1 的阻塞范围

当前写法将步骤 2–8（全部离线代码和单元测试）阻塞在运行时 `ros2 topic echo` 验证之后。但步骤 2–8 全程使用合成/mock 地图，不依赖运行中的感知栈。建议在步骤 1 中补充一句：

> 该步骤只作为步骤 9（Gazebo 验收）和地形净空功能上线的前置门槛，不阻塞步骤 2–8 的离线代码实现和单元测试。

### 5.3 第 9.4 节几何上界的表述

原文写"忽略关节限位的 HAA 根到足端直腿几何上界约为 0.733 m"。该数值是各段向量模的标量和（HAA→HFE≈0.105 m、HFE→KFE≈0.379 m、KFE→足端待核实），不是实际伸展距离（实际距离还受各段方向夹角影响，必然小于标量和）。文档已注明"不视为已验证阈值"且要求 G0.5 标定，措辞本身不会导致错误实现，但建议在括号内加注"（各段长度标量和，非真实可达距离）"以防读者误用。

## 6. 其余章节审核结论

| 章节 | 结论 |
|---|---|
| §1 修订目的（六项歧义） | 所有六项均有代码或文档依据，准确 |
| §2 背景 | 四项简化与现有代码一致 |
| §3 目标、§4 非目标 | 范围划定清晰，无越界 |
| §5 对应关系表 | 与 OCS2 示例行为对照准确；K20 适配说明恰当 |
| §6 架构图和职责划分 | 数据流与文件边界一致 |
| §7 配置结构 | `legRootJointByContact` map 格式可被 Boost ptree info 解析；移除可切换层名配置与§11.1 固定 `elevation` 的决策一致；兼容规则全面 |
| §8 Raibert 修正 | CoM 速度语义、适用条件、二维限幅、XY 专项滤波均正确；"候选未提交时历史值不改变"是重要的安全约束 |
| §9.1–9.3 运动学惩罚 | π 旋转数学已独立验证；代价公式与 OCS2 完全一致；两次评估（触地开始/结束）符合 OCS2 示例语义 |
| §9.4 标定门槛 | G0.5 步骤是原版设计没有的有效风险缓解措施 |
| §10.1 冻结快照字段 | 字段完备，但缺少 `frozenRegion` 条目（见第 3 节） |
| §10.2 接触事件匹配 | 时间容差匹配逻辑清晰；扩展结束时间处理恰当 |
| §10.3 冻结行为 | 除末尾"STANCE"歧义外，状态机覆盖完整；"清理依据是计划接触相和时间边界"的声明正确且诚实 |
| §11.1 地图层来源辨析 | 占位层声明已代码核实；固定使用 `elevation` 的决策与 OCS2 `SegmentedPlanesTerrainModel` 一致；运行时验证门槛合理 |
| §11.2 路径采样 | 按地图分辨率采样（不引入额外参数）、NaN 跳过、边缘栅格处理均正确 |
| §11.3 接入三次样条 | `maxHeightSequence` 语义已核实；功能关闭时与现有实现完全一致 |
| §12 回退表 | 新增三行地形净空回退条目语义正确；所有条目均有对应处理路径 |
| §13 诊断 | `terrainClearanceActive` 要求脱耦于配置开关的设计正确；节流日志约束重要 |
| §14.1 单元测试 | "打乱 contact 顺序后映射结果一致"测试是防腿根错配的核心保障；"HAA 零旋转时测试通过"保护 K20 特有问题 |
| §14.2 Gazebo | G0.5 标定在 G1 前强制执行，防止阈值问题推迟到实机 |
| §14.3 实机 | 立即终止条件完整；逐步提速策略合理 |
| §16 实施顺序 | 除步骤 1 阻塞范围歧义外，顺序逻辑正确 |
| §17 成功标准 | 与正文逐项对应，可检验 |
| §18 检查清单 | 三项待勾选项与实际状态相符 |

## 7. 需在设计文档中补充/修改的条目（汇总清单）

| # | 位置 | 动作 | 优先级 |
|---|---|---|---|
| 1 | §10.1 快照字段 | 增加 `frozenRegion: std::shared_ptr<PlanarRegion>`，并补充深拷贝构造和生命周期说明 | 阻塞实现 |
| 2 | §14.1 冻结测试 | 增加"地图清空后 `regionPtr` 非空且变换不变"测试用例 | 阻塞实现 |
| 3 | §15 文件列表 | 把 `PerceptiveLeggedPrecomputation.h/.cpp` 明确归入"预计不修改（方案 A）"或"必须修改（方案 C）" | 阻塞实现 |
| 4 | §10.3 | 把末尾"STANCE"改为"全局 `modeNumber == STANCE`（四腿同时接触）" | 建议 |
| 5 | §16 步骤 1 | 补充"不阻塞步骤 2–8 的离线开发和单元测试" | 建议 |
| 6 | §9.4 | 在 0.733 m 后加注"（各段长度标量和，非真实可达距离）" | 建议 |

## 8. 待确认项（继承自被审核文档）

以下三项来自设计文档第 18 节检查清单，审核期间无法从离线代码核实，须在进入实现前完成：

1. K20 Pinocchio 模型中 `LF_FOOT→LF_HAA`、`RF_FOOT→RF_HAA`、`LH_FOOT→LH_HAA`、`RH_FOOT→RH_HAA` 均可解析且唯一（需实际启动并调用 Pinocchio API 验证）。
2. 启动完整感知栈后，从实时 `PlanarTerrain` 消息确认 `elevation` 层存在且含有限样本（命令见设计文档第 11.1 节；2026-07-30 检查时无目标 topic publisher）。
3. G0.5 完成 `nominalLegExtension` 标定，确认正常步态无持续误惩罚（依赖 Gazebo 运行）。
