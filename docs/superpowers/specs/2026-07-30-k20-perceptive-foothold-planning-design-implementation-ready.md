# K20 感知落脚规划增强设计（实现就绪版）

日期：2026-07-30
状态：实现基线；运行时地图层验证和 Gazebo 标定待完成
目标机器人：K20
主要步态：Trot

本文合并并替代以下文档作为实现依据：

- `2026-07-30-k20-perceptive-foothold-planning-design-revised.md`
- `2026-07-30-k20-perceptive-foothold-planning-design-review.md` 中已确认的修改建议

上述文档与 `2026-07-28-k20-perceptive-foothold-planning-design.md` 均继续保留，分别用于历史对照和审核追踪。

## 1. 设计结论

本阶段只增强当前 K20 参考规划，不宣称复现论文的完整控制器。四项功能分别是：

1. 当前摆动腿第一触地点的 Raibert XY 修正。
2. 候选平面的腿过伸和向身体内侧跨步惩罚。
3. 摆动后半程的落脚决策冻结。
4. 基于路径障碍高度的摆动净空自适应。

已确定的关键决策：

| 项目 | 决策 |
|---|---|
| Raibert 速度 | 使用世界系 CoM 速度代理，诊断中不得称为 base velocity |
| 冻结阈值 | `freezePhase=0.5`，作为“摆动后半程”的 K20 实现 |
| contact 与腿根 | 使用足端名称到 HAA joint 的显式映射，不使用并行数组下标 |
| 左右腿方向 | 从 URDF 中 HAA 相对 base 的静态 y 位置派生 |
| 冻结平面所有权 | 快照完整深拷贝所选 `PlanarRegion`，以 shared ownership 保证 raw `regionPtr` 有效 |
| 摆动障碍层 | 固定读取后处理后的 `elevation`，不回退到启动占位层或 `smooth_planar` |
| 过伸阈值 | `0.62 m` 仅作待标定初值，必须在 Gazebo G0.5 阶段确定最终值 |

## 2. 依据与当前基线

### 2.1 论文与 OCS2 对应关系

| 功能 | 论文 | OCS2 示例 | K20 方案 |
|---|---|---|---|
| Raibert 修正 | 第一即将触地的落脚点 | 世界系基座速度差，只修正当前摆动腿的下次触地 | 世界系 CoM 速度代理，修正当前摆动腿的第一触地点 |
| 平面评分 | 距离加运动学惩罚 | 在接触开始和结束两次评价过伸与内跨 | 相同代价结构，使用 K20 固定规划髋坐标系 |
| 后半程冻结 | 摆动后半程约束不再改变 | 按距触地时间复用历史落脚结果 | 按归一化摆动相位从 50% 起冻结 |
| 摆动净空 | 越过路径最高地形 | 从 `elevation` 查询路径高度剖面 | 从 `elevation` 计算绝对 `maxHeightSequence`，复用现有三次样条 |

### 2.2 当前代码事实

- `ModelSettings::contactNames3DoF` 不从 `task.info` 加载，硬编码默认顺序为 `LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT`。
- 当前 `ConvexRegionSelector` 已计算 Raibert 速度反馈，但名义落脚点返回值未使用该反馈。
- 当前候选平面的附加评分恒为零。
- 当前每个 MPC 周期整体复制 `PlanarTerrain`，`PlanarTerrainProjection::regionPtr` 指向该周期地图容器。
- `PerceptiveLeggedPrecomputation` 直接读取 `regionPtr->transformPlaneToWorld`；冻结恢复不能返回空指针或旧地图指针。
- `SwingTrajectoryPlanner` 已支持三参数 `update(..., maxHeightSequence)`，内部还会加入 `scaling * swingHeight`。
- OCS2 `SegmentedPlanesTerrainModel` 的高度剖面与 SDF 固定读取 `elevation`。

## 3. 目标与非目标

### 3.1 目标

- 四项功能独立开关，新增配置缺失时保持旧行为。
- 开关全部关闭时，落脚点、平面选择、凸区域和摆动高度与当前实现一致。
- 所有跨 MPC 周期状态具有明确所有权和清理边界。
- 平地、缓坡和低台阶使用同一逻辑，不增加地形模式切换。
- 通过单元测试、Gazebo 和 K20 实机低速测试逐级验收。

### 3.2 非目标

- 不引入论文的 48 维 loop-shaping 状态。
- 不修改 MPC 状态/输入维度、centroidal 动力学、SQP/HPIPM、WBC 或状态估计。
- 不加入 MPC 内关节位置、速度或力矩约束。
- 不新增真实基座线速度到 ReferenceManager 的数据通路。
- 不接入实际接触测量；冻结状态按计划接触相和时间边界管理。
- 不替换现有三次摆动样条。
- 不改变 P1 或其他机器人配置的默认行为。

## 4. 总体架构与生命周期不变量

```text
当前 centroidal 状态 + 目标轨迹 + Trot 接触时序
                         |
                         v
        当前摆动腿第一触地点的 Raibert XY 修正
                         |
                         v
       候选平面距离 + K20 规划髋运动学惩罚
                         |
                         v
          生成凸区域并按接触事件提交决策
                         |
                         v
    后半程冻结：projection + polygon + frozenRegion
                         |
                         v
        同一地图快照的 elevation 路径高度剖面
                         |
                         v
            maxHeightSequence -> 现有三次样条
                         |
                         v
                  现有 MPC 足端软约束
```

职责划分：

- `PerceptiveLeggedInterface`：加载统一配置，提供 Pinocchio 模型信息。
- `ConvexRegionSelector`：Raibert 修正、历史滤波、运动学评分、候选选择、冻结快照和地形高度采样。
- `PerceptiveLeggedReferenceManager`：组织接触相的起落脚高度并构造 `maxHeightSequence`。
- `PerceptiveLeggedPrecomputation`：保持现有消费接口，通过有效 `regionPtr` 构造 MPC 足端软约束参数。
- `SwingTrajectoryPlanner`：保持现有实现，继续叠加 `maxHeightSequence + scaling * swingHeight`。

必须保持以下生命周期不变量：

1. `feetProjections_` 中的非空 `regionPtr` 只能指向当前 `planarTerrain_` 或当前快照持有的 `frozenRegion`。
2. 任何 projection 仍引用 `frozenRegion.get()` 时，不得释放对应 shared owner。
3. 一次 `ConvexRegionSelector::update()` 内先构造并验证下一组 projection、polygon 和 owner，再整体提交；不能先释放 owner、后覆盖旧 projection。
4. ReferenceManager 的 `modifyReferences()` 在 solver pre-run 阶段更新选择器；同一次 solver run 的 Precomputation 消费期间不再修改选择器所有权状态。
5. 禁止把冻结平面追加到 `planarTerrain_.planarRegions`，避免候选污染和 vector 扩容导致的指针失效。

## 5. 统一配置

新增可选配置组：

```text
perceptive_foothold_planning
{
  enableRaibertFeedback       false
  enableKinematicPenalty      false
  enableLateSwingFreeze       false
  enableTerrainClearance      false

  invertedPendulumHeight      0.55
  raibertMaxOffset            0.08
  previousFootholdFactor      0.70
  previousFootholdDeadzone    0.01

  nominalLegExtension         0.62
  kinematicPenaltyWeight      5.0

  freezePhase                 0.50
  contactTimeMatchTolerance   0.03

  terrainClearanceMargin      0.03
  maxTerrainAdaptation        0.12

  legRootJointByContact
  {
    LF_FOOT LF_HAA
    RF_FOOT RF_HAA
    LH_FOOT LH_HAA
    RH_FOOT RH_HAA
  }
}
```

兼容和验证规则：

- 配置组或任一开关缺失时，相应功能关闭。
- 所有标量必须有限；高度、距离、裕量和权重不得为负。
- `previousFootholdFactor` 和 `freezePhase` 必须位于 `[0,1]`。
- `contactTimeMatchTolerance >= 0`。
- 运动学功能开启时，遍历运行时 `modelSettings.contactNames3DoF`，按 contact name 查询腿根。
- 每个 3-DoF contact 必须恰有一个映射；缺项、未知 contact、重复 joint 或无法解析的 joint 均启动失败。
- 左右侧由 HAA 相对 base 的静态 y 位置派生；其绝对值必须大于小阈值。
- 地形净空的障碍层在实现中固定为 `elevation`，不新增层名配置。
- 开发和仿真阶段四项默认关闭；逐项验收后才允许在 K20 感知配置中开启。

## 6. Raibert 第一触地点修正

### 6.1 速度语义

K20 当前直接读取：

```text
centroidal_model::getNormalizedMomentum(state, info).head<3>()
```

对 FullCentroidalDynamics，该量是世界系 CoM 速度。本阶段采用：

\[
\Delta p_{xy}=\sqrt{\frac{h}{g}}
(v_{\mathrm{CoM,measured},xy}-v_{\mathrm{CoM,desired},xy})
\]

代码、诊断和测试统一使用 `ComVelocity` 命名。若以后需要严格使用基座速度，必须另行设计状态估计到 ReferenceManager 的数据通路。

### 6.2 适用事件

- 只修正当前已经处于摆动期的腿。
- 只修正该腿第一个计划触地开始事件。
- 当前支撑腿和预测范围内更晚的触地事件不修正。
- 当前支撑足位置保持现有逻辑。
- 没有完整起飞/触地边界时不制造虚假事件。

### 6.3 计算和提交

1. 由现有运动学和坡度补偿得到 `p_kinematic`。
2. 检查实测/目标 CoM 速度有限性；无效时本周期偏移为零。
3. 计算 `delta_xy`，z 固定为零。
4. 按二维模长限制到 `raibertMaxOffset`，不分别裁剪 x/y。
5. 得到 `p_raw = p_kinematic + delta_xy`。
6. 只对 XY 应用同一接触事件的历史死区和低通；z 使用本周期原始值。
7. 用过滤后的启发点执行候选投影和运动学评分。
8. 只有候选完整验证并提交后，才更新历史触地点。

滤波公式：

\[
p_{xy,filtered}=\lambda p_{xy,previous}+(1-\lambda)p_{xy,new}
\]

当新旧 XY 距离小于 `previousFootholdDeadzone` 时保留旧 XY。候选失败不得污染历史状态。

## 7. K20 运动学候选平面惩罚

### 7.1 固定规划髋坐标系

K20 四个 HAA 在 URDF 中均为零旋转，不能依赖原生 Hip frame 自动产生左右相反的内跨方向；`*_hip` link 又会随 HAA 转动，也不是固定 leg-root frame。

每条腿从 Pinocchio 模型读取 HAA 根关节相对 base 的固定变换：

- 左腿：规划髋方向与 base 一致。
- 右腿：在 base 中绕 z 旋转 `pi`，使规划髋的正 x 转动约定仍表示向外。
- 原点为 HAA 根位置。
- 只随期望 base 位姿移动，不随腿部关节角转动。

禁止根据腿数组下标或世界系 y 符号判断左右；只允许使用 URDF 中 HAA 相对 base 的静态横向位置。

### 7.2 惩罚定义

\[
p_{foot}^{hip}=R_{world\rightarrow hip}(p_{foot}^{world}-p_{hip}^{world})
\]

\[
e_{extension}=\max(0,\|p_{foot}^{hip}\|-l_{max})
\]

\[
e_{inward}=\max(0,d_{inward}^{hip}\cdot p_{foot}^{hip})
\]

\[
J_{kin}=w_k(e_{extension}^{2}+e_{inward}^{2})
\]

候选总代价：

\[
J=\|p_{heuristic}-p_{projected}\|^{2}+J_{kin,start}+J_{kin,end}
\]

触地开始和结束的 hip pose 均由对应时刻的期望 base pose 与固定腿根变换得到。运动学输入非有限时，本周期附加惩罚为零，保留距离选择。

### 7.3 `nominalLegExtension` 标定

`nominalLegExtension=0.62 m` 只是保守初值。K20 大腿和小腿各为 `0.35 m`，并存在 `0.093 m`、`0.0495 m` 和 `0.145 m` 偏移，不能用 `0.35+0.35` 直接确定阈值。

几何数值只用于解释，不作为验收值：

- `HFE=0、KFE=0` 时，HAA 根到足端距离约为 `0.732 m`；这是零角参考姿态，不是几何上界，也不是关节限位下的可达阈值。
- 三段变换向量的模长和约为 `0.834 m`，只是不考虑方向关系的三角不等式上界。
- 按 `KFE <= -0.698 rad` 且允许 HFE 调整方向的简化几何估计，最大根足距离约为 `0.776 m`；最终仍以 Pinocchio FK 扫描为准。

Gazebo G0.5 必须在运动学惩罚生效前完成：

1. 令运动学惩罚关闭或权重为零，只发布诊断。
2. 运行站立、原地 Trot、`0~0.20 m/s` 加减速、8° 缓坡和 0.05 m 台阶。
3. 记录四腿在接触开始/结束时的 `||p_foot^hip||` 分布、最大值和关节状态。
4. 使用 K20 关节限位执行 Pinocchio FK 可达域扫描。
5. 选择高于正常工况上界并留有模型/测量裕量、同时低于危险伸展区的阈值。
6. 若 `0.62 m` 在正常步态持续产生惩罚，必须在 G1 前调整，不得推迟到实机。

## 8. 摆动后半程冻结

### 8.1 快照结构与平面所有权

每条腿保存一个按接触事件管理的快照：

```text
FootholdDecisionSnapshot
{
  contactStartTime
  contactEndTime
  filteredHeuristicXY
  projectionPositionInTerrainFrame
  projectionPositionInWorld
  projectionCost
  planeTransformToWorld
  convexPolygon
  touchdownHeight
  frozen
  valid
  frozenRegion: shared_ptr<const PlanarRegion>
}
```

捕获成功候选时执行完整深拷贝：

```cpp
snapshot.frozenRegion =
    std::make_shared<const convex_plane_decomposition::PlanarRegion>(*projection.regionPtr);
```

不得只保存来自地图容器的裸指针，也不得只构造一个字段不完整的临时平面。完整拷贝包含 `boundaryWithInset`、`bbox2d` 和 `transformPlaneToWorld`，减少当前和未来消费方对未初始化字段的依赖。

冻结恢复时重建 `PlanarTerrainProjection`：

```text
regionPtr              = snapshot.frozenRegion.get()
positionInTerrainFrame = snapshot.projectionPositionInTerrainFrame
positionInWorld        = snapshot.projectionPositionInWorld
cost                   = snapshot.projectionCost
```

对应凸多边形直接取快照值。禁止重新在当前地图中寻找“最相似平面”，也禁止把 `frozenRegion` 追加回当前地图候选集合。

### 8.2 owner 提交和清理顺序

- projection、polygon 和 owner 必须作为同一候选事务提交。
- `feetProjections_` 引用 `frozenRegion.get()` 期间，快照 shared owner 必须保留。
- 下一次 update 中，先构造不再引用旧 owner 的新输出，再释放旧快照。
- `getProjection()` 返回 raw pointer 后，同一次 solver run 内不得并发更新选择器。
- 快照清理后，任何已提交 projection 都不得继续指向已释放的 `frozenRegion`。

### 8.3 接触事件匹配

以下条件同时满足时视为同一事件：

```text
abs(newStart - oldStart) <= contactTimeMatchTolerance
abs(newEnd   - oldEnd)   <= contactTimeMatchTolerance
```

接触结束时间暂时超出预测范围时，使用当前扩展值，并在后续周期按相同容差匹配更新。

### 8.4 冻结行为

摆动进度：

\[
s=\operatorname{clip}\left(
\frac{t-t_{liftoff}}{t_{touchdown}-t_{liftoff}},0,1
\right)
\]

- `s < freezePhase`：允许更新候选；成功结果写入快照。
- `s >= freezePhase`：复用最后一个有效快照，不查询新平面或重建新多边形。
- 首次观察事件时已超过阈值：接受第一个有效候选并立即冻结。
- 时序小幅漂移：按容差匹配并保持冻结。
- 后半程完全失配：保持最后结果至缓存的计划触地边界。
- 冻结只固定参考落脚决策，不锁定 MPC 状态、接触力或实际足端轨迹。

清理条件：

- 进入该腿的新计划支撑相并已构造不引用旧 owner 的新输出。
- 识别到新摆动事件。
- 步态切换。
- 当前时间超过缓存触地时间及容差。
- 当前计划相满足 `modeSchedule.modeAtTime(initTime) == ModeNumber::STANCE`，即四腿同时接触。

本阶段没有实际接触测量，因此不声称按实际触地清理。

## 9. 地形感知摆动高度

### 9.1 地图层来源

源码中存在两个同名 `elevation_before_postprocess` 来源：

1. `PerceptiveLeggedInterface` 启动时创建 5×5 m 全零占位地图，只含 `elevation_before_postprocess` 和 `smooth_planar`。
2. 真实 `PlaneDecompositionPipeline` 对输入 `elevation` 完成补洞、去噪和重采样后，在 Postprocessing 中复制出 `elevation_before_postprocess`；随后原 `elevation` 继续执行非平面水平膨胀和高度偏移。

当前 K20 的 `height_layer=elevation`。控制器订阅的 `PlanarTerrain` 包含后处理后的 `elevation`；`elevation_raw` 和 `segmentation` 在发布 `PlanarTerrain` 之后才加入 `filtered_map`，不属于控制输入。

本设计固定读取 `elevation`：

- 不读取或回退到 `elevation_before_postprocess`，避免启动全零占位层误激活功能。
- 不读取或回退到 `smooth_planar`，避免台阶和窄障碍被平滑。
- 当前 `elevation` 已包含非平面 0.03 m 高度偏移和 3 格水平膨胀；额外 `terrainClearanceMargin` 必须在 Gazebo 验证叠加效果。

### 9.2 运行时验证门槛

启动完整感知栈后执行：

```bash
ros2 topic list | rg '^/convex_plane_decomposition_ros/planar_terrain$'
timeout 10s ros2 topic echo /convex_plane_decomposition_ros/planar_terrain \
  --once --field gridmap.layers \
  --filter "'elevation' in m.gridmap.layers and any(v == v and abs(v) != float('inf') for v in m.gridmap.data[m.gridmap.layers.index('elevation')].data)"
```

第二条命令只有收到包含有限 `elevation` 样本的消息才会成功输出 layer 列表；超时、空输出或非零退出均视为验证失败。源码、RViz、参数文件、启动占位地图或接收器 fallback 中存在层名不算运行验证。

该门槛：

- 不阻塞配置、纯函数、mock 地图和生命周期单元测试。
- 阻塞 Gazebo 中地形净空的启用、地形净空验收和后续实机上线。
- 没有 publisher、命令超时、缺层或全无效时，`enableTerrainClearance` 必须保持关闭。

2026-07-30 当前 ROS graph 只有 `/parameter_events` 和 `/rosout`，本项仍未通过。

### 9.3 路径采样

- 沿起落脚 XY 连线遍历相交栅格，采样间距使用地图分辨率。
- 每个栅格只读取 `elevation` 的有限值。
- NaN 样本跳过；没有有限样本时回退固定摆高。
- 起落脚处于同一栅格或 XY 距离小于一个分辨率时，不查询中间障碍。
- 地图外、无有效 index 或查询异常均回退固定摆高。

对有效样本 i：

\[
h_{line,i}=(1-s_i)h_{lift}+s_i h_{touch}
\]

\[
h_{obs}=\max_i(h_{terrain,i}-h_{line,i})
\]

\[
h_{clear}=\begin{cases}
0,&h_{obs}\leq0\\
h_{obs}+h_{margin},&h_{obs}>0
\end{cases}
\]

\[
h_{adapt}=\operatorname{clip}(h_{clear},0,h_{adapt,max})
\]

### 9.4 接入三次样条

传入现有接口：

\[
maxHeightSequence=\max(h_{lift},h_{touch})+h_{adapt}
\]

现有 planner 继续计算：

\[
h_{mid}=maxHeightSequence+scaling\cdot h_{swing}
\]

`maxHeightSequence` 不得预先包含 `swingHeight`。功能关闭或回退时 `h_adapt=0`，与现有二参数 update 的结果一致。

K20 初值：

```text
terrainClearanceMargin  0.03
maxTerrainAdaptation    0.12
swingHeight             0.15  # 现有配置
```

完整摆动的最大中点附加高度为 `0.15 + 0.12 = 0.27 m`。

## 10. 候选事务、回退和诊断

### 10.1 候选事务

每个接触事件按以下顺序处理：

1. 计算原始和过滤后的启发点。
2. 构造非负运动学评分函数。
3. 选择最佳平面。
4. 验证 `regionPtr`、投影、变换、边界和凸多边形。
5. 深拷贝 `frozenRegion`，构造完整快照和本周期输出。
6. 整体提交 projection、polygon、历史滤波点和快照 owner。

任何一步失败均不得部分提交。

### 10.2 回退表

| 异常 | 回退行为 |
|---|---|
| CoM 速度非有限 | 本周期 Raibert 偏移为零 |
| Raibert 偏移过大 | 按 XY 模长限制 |
| 运动学输入非有限 | 本周期运动学附加惩罚为零 |
| 没有有效候选 | 使用同事件上一有效值；无历史时沿用当前错误处理并诊断 |
| 冻结期地图替换 | 使用 `frozenRegion` owner，不访问旧地图指针 |
| 后半程时序小幅漂移 | 容差匹配并继续冻结 |
| 后半程时序完全失配 | 保持缓存结果至计划触地边界 |
| 只有启动占位层 | 固定摆高，`terrainClearanceActive=false` |
| 实时 `elevation` 缺失或无有限值 | 固定摆高，inactive 并输出节流警告 |
| 路径少量 NaN | 跳过无效栅格 |
| 地形尖峰 | 限制到 `maxTerrainAdaptation` |

### 10.3 诊断

每条当前摆动腿记录：

- 实测/目标 CoM 速度及 Raibert 原始、限幅和过滤偏移。
- 历史死区命中和候选是否提交。
- 距离、触地开始运动学和触地结束运动学三项代价。
- 规划髋侧别、腿根位置和内跨方向。
- 被选平面、凸区域、快照有效性、`frozenRegion` owner 状态。
- 摆动进度、事件匹配、冻结状态和清理原因。
- 地图是启动占位还是含有效 `elevation` 的接收地图。
- 有限样本数、最高障碍、增高限幅和最终 `maxHeightSequence`。
- `terrainClearanceActive` 必须表示本周期实际使用了有效 `elevation`，不能只复述配置开关。

高频信息进入调试接口或节流日志，禁止在 MPC 周期持续输出普通日志。

## 11. 测试设计

### 11.1 配置和兼容性

- 配置组缺失和四项开关关闭时输出与当前实现一致。
- contact 顺序使用默认 LF/RF/LH/RH 和打乱顺序时，腿根映射结果相同。
- 缺失 contact、未知 contact、重复 joint 和旧式并行数组均拒绝启用。
- P1 和非感知控制配置不受影响。

### 11.2 Raibert

- 零、正、负 CoM 速度误差。
- 只修正当前摆动腿第一触地点。
- 二维模长限幅，不产生 z 偏移。
- XY 死区/低通，z 不参与历史滤波。
- 非有限速度回退为零偏移。
- 候选未提交时历史值不改变。

### 11.3 运动学惩罚

- 从 K20 URDF 派生四腿侧别和固定规划髋坐标。
- LF/LH 与 RF/RH 的向内候选受罚、向外候选不受罚。
- 四个 HAA 原生旋转均为零时测试仍通过。
- 正常、阈值边界、过伸和非有限输入。
- 接触开始/结束评分累加。
- 几何距离接近时选择运动学更合理的平面。
- Pinocchio FK 扫描输出用于 G0.5 标定，不把解析估计当最终阈值。

### 11.4 冻结和 owner 生命周期

- `freezePhase` 前后和阈值边界。
- 首次观察已超过阈值时选择一次后立即冻结。
- 地图对象整体替换、原 `planarRegions` 清空后，冻结 projection 的 `regionPtr` 仍非空。
- `regionPtr == frozenRegion.get()`，且完整平面变换、边界和 bbox 与冻结时一致。
- 在上述地图替换后实际调用 `PerceptiveLeggedPrecomputation::request()`，足端约束参数有限且与冻结前一致。
- projection 仍引用 owner 时尝试清理快照，测试必须阻止悬空引用。
- 新输出不再引用旧 owner 后清理快照，验证旧对象可释放。
- 接触时间容差匹配、完全失配保持、新事件/步态切换/当前全局 STANCE 清理。

### 11.5 地形摆高

- 平地、上坡、下坡、中间台阶和路径外障碍。
- 启动占位地图虽然包含有限的全零 `elevation_before_postprocess`，功能仍 inactive。
- 只有实时有限 `elevation` 才允许 active。
- 不读取 `smooth_planar` 或 `elevation_before_postprocess`。
- NaN、全无效、地图外、异常尖峰和同栅格起落点。
- 后处理高度偏移/膨胀与 margin 叠加的标定用例。
- `maxHeightSequence` 不包含 `swingHeight`，防止重复加高。

## 12. Gazebo 验收

G0 基线：四项关闭，记录 MPC 频率、求解时间、速度误差、落脚变化、足端高度和安全故障。

G0.5 阈值标定：运动学惩罚不生效，只发布诊断；完成 Pinocchio FK 扫描和站立/Trot/坡地/台阶数据采集，确定 `nominalLegExtension`。

G1 平地：

- 按 Raibert、运动学惩罚、冻结、地形净空顺序逐项开启。
- 地形净空开启前必须通过第 9.2 节实时 `elevation` 验证。
- Trot 从 0 加速至 0.20 m/s，再减速至 0。
- 施加沿机身前后方向 150 N、持续 0.20 s 的推扰。
- 四项全开连续运行不少于 60 s。

G2 缓坡：上下坡各 8°，Trot 速度 0.15 m/s。

G3 低台阶：台阶高度 0.05 m，Trot 速度 0.10 m/s。

通过标准：

- 无摔倒、NaN、MPC 异常退出和新增安全故障。
- 摆动后半程平面、凸区域和触地点高度保持不变。
- 冻结路径的 Precomputation 约束始终有效，无悬空指针。
- 台阶场景摆动参考高于路径障碍。
- 四项关闭时保持基线行为。
- 正常步态不持续触发过伸惩罚。
- 移除 `elevation` 的故障注入使地形净空 inactive 并产生节流警告。
- MPC 实际保持 60 Hz，无持续超时。
- 新增参考规划耗时相对基线增幅不超过 20%。

## 13. K20 实机验收

测试配备急停、保护绳并保留单项关闭能力。

H0 静态检查：

- 地图、CoM 速度和足端数据有限。
- 四腿规划髋方向、contact 映射和内跨惩罚正确。
- 原地抬腿时冻结 owner、`regionPtr` 和摆高诊断正确。

H1 平地：初始 0.10 m/s，连续 Trot 20 s；逐项开启后再提高至 0.20 m/s。

H2 缓坡：初始坡度 5°，Trot 速度 0.10 m/s。

H3 低台阶：初始高度 0.03 m、速度不高于 0.10 m/s；通过后提高至 0.05 m。

立即终止条件：

- MPC/WBC 出现非有限值。
- 触发关节位置、速度或力矩安全保护。
- 摆动后半程发生跨平面切换。
- 足端目标发生超过 0.05 m 的单周期跳变。
- 连续两步拖脚或碰撞台阶。
- MPC 持续低于目标频率。
- 机身姿态或速度误差持续扩大。

## 14. 预计涉及文件

必须修改：

- `legged_perceptive_interface/include/legged_perceptive_interface/ConvexRegionSelector.h`
- `legged_perceptive_interface/src/ConvexRegionSelector.cpp`
- `legged_perceptive_interface/include/legged_perceptive_interface/PerceptiveLeggedReferenceManager.h`
- `legged_perceptive_interface/src/PerceptiveLeggedReferenceManager.cpp`
- `legged_perceptive_interface/src/PerceptiveLeggedInterface.cpp`
- `legged_perceptive_controllers/config/k20/task.info`
- `legged_perceptive_interface/CMakeLists.txt`
- 对应新增单元测试文件

预计不修改：

- `legged_perceptive_interface/include/legged_perceptive_interface/PerceptiveLeggedPrecomputation.h`
- `legged_perceptive_interface/src/PerceptiveLeggedPrecomputation.cpp`
    - 采用完整深拷贝的 `shared_ptr<const PlanarRegion>` 保证现有 `regionPtr` 接口有效。
    - 本文件只作为端到端生命周期测试的真实消费方，不修改生产逻辑。
- `legged_interface/constraint/SwingTrajectoryPlanner.h/.cpp`：现有三参数接口足够。
- `PerceptiveController.cpp` 和 `PlanarTerrainReceiver.cpp`：地形净空从当前地图副本固定读取 `elevation`，不改变 SDF 的 `smooth_planar` 首选策略。
- MPC 动力学、求解器、WBC、状态估计和硬件接口。

若实现发现必须触及“预计不修改”文件，应先更新本文档并说明必要性，不得顺手扩大范围。

## 15. 实施顺序

运行时门槛 R0 可与步骤 1–7 并行：启动完整感知栈并验证实时 `PlanarTerrain.elevation`。R0 失败不阻塞离线实现和单元测试，但阻塞步骤 8 中地形净空的开启与验收。

1. 新增配置结构和加载测试，验证缺失配置保持旧行为、contact 映射不依赖顺序。
2. 建立 Raibert 纯函数测试并实现修正，开关默认关闭。
3. 建立规划髋和运动学惩罚测试并实现候选评分。
4. 建立 `frozenRegion` owner、接触事件匹配和事务提交测试。
5. 实现后半程冻结，并通过真实 Precomputation 消费测试。
6. 建立 `elevation` 层选择和路径高度剖面测试，构造 `maxHeightSequence`。
7. 验证四项关闭或配置缺失时与旧输出一致。
8. R0 通过后，按 G0、G0.5、G1、G2、G3 完成 Gazebo 验收。
9. 按 H0、H1、H2、H3 完成 K20 实机低速验收。
10. 全部通过后，才允许在 K20 感知配置中默认开启已验收功能。

## 16. 成功标准

- 四项功能可独立开启和关闭。
- 配置缺失或全部关闭时不改变当前 K20 行为。
- CoM 速度代理与论文 base velocity 在命名和诊断中明确区分。
- contact 顺序变化不会造成腿根错配。
- 左右内跨方向由 K20 模型正确派生。
- `nominalLegExtension` 在 Gazebo 完成标定。
- 冻结 projection 的 raw `regionPtr` 始终有稳定 owner，地图替换后 Precomputation 约束仍有效。
- 摆动后半程不切换平面、凸区域或触地点高度。
- 地形净空只使用实时有限 `elevation`，启动占位层不得误激活功能。
- `maxHeightSequence` 和现有 `swingHeight` 各加入一次。
- 平地、缓坡和低台阶无需切换规划模式。
- 单元测试、Gazebo 和实机低速验收达到本文门槛。

## 17. 实现前与上线前检查清单

设计已确定：

- [x] 使用世界系 CoM 速度作为 Raibert 代理。
- [x] 使用 `freezePhase=0.5`。
- [x] 使用 contact name 到 HAA joint 的显式映射。
- [x] 冻结快照完整深拷贝 `shared_ptr<const PlanarRegion>`，不保留旧地图裸指针。
- [x] 地形净空固定读取后处理后的 `elevation`。
- [x] 实时地图验证不阻塞离线实现，但阻塞地形净空启用和验收。

实现/运行仍待完成：

- [ ] Pinocchio 中四组 contact → HAA 映射均可解析且唯一。
- [ ] 单元测试覆盖 owner 生命周期，并实际通过 `PerceptiveLeggedPrecomputation::request()` 消费冻结 projection。
- [ ] 完整感知栈的实时 `PlanarTerrain` 包含有限 `elevation`。
- [ ] G0.5 完成 `nominalLegExtension` 和 clearance margin 标定。
- [ ] 四项关闭时回归结果与旧行为一致。
- [ ] Gazebo G0–G3 和实机 H0–H3 达到验收门槛。
