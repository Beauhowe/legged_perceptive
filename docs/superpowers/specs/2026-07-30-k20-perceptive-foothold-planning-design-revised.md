# K20 感知落脚规划增强设计（论文与 OCS2 对齐修订版）

日期：2026-07-30
状态：实现基线（地形净空地图层仍待运行时验证）
目标机器人：K20
主要步态：Trot

## 1. 修订目的

经本轮审查，本文档作为后续实现基线，替代 `2026-07-28-k20-perceptive-foothold-planning-design.md`；原文继续保留供比较。

修订依据：

- 论文《Perceptive Locomotion through Nonlinear Model Predictive Control》的参考生成、足端约束和摆动净空方法。
- 工作区 `ocs2_perceptive_anymal` 中 `SwingTrajectoryPlanner`、`KinematicFootPlacementPenalty`、`FootPhase` 和 `SegmentedPlanesTerrainModel` 的实现。
- K20 当前 centroidal MPC 状态、Pinocchio 模型、感知地图层和三次摆动样条接口。

本次修订及审查解决原设计中的六个实现歧义：

1. 论文使用世界系基座线速度，K20 当前可直接获得的是质量归一化线动量的线性部分，即世界系 CoM 速度。
2. K20 四个 HAA 根关节在 URDF 中均为零旋转，不能直接依赖原生 Hip frame 自动产生左右相反的内跨方向。
3. `PlanarTerrainProjection` 持有指向当前地图副本的 `regionPtr`，冻结状态不能跨地图更新保存该裸指针。
4. 当前 SDF 接收路径首选 `smooth_planar`；OCS2 的路径高度剖面显式读取后处理后的 `elevation`，K20 摆动净空必须沿用该语义，不能把控制器启动时手动创建的 `elevation_before_postprocess` 占位层误认为实时感知地图。
5. `ModelSettings::contactNames3DoF` 不从 `task.info` 加载，当前硬编码顺序为 `LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT`；腿根配置不能使用容易错位的并行数组。
6. 地形净空开关不能仅依赖配置存在；启用前必须从实时 `PlanarTerrain` 消息验证障碍高度层。

## 2. 背景

当前 `legged_perceptive` 已具备：

- 从分割平面选择候选落脚区域并生成局部凸多边形。
- 通过 MPC 足端软约束限制支撑足位置。
- 通过 SDF 对摆动足执行碰撞约束。
- 根据起落脚高度生成固定摆高的三次样条。

当前参考规划仍有四项关键简化：

1. 已计算 Raibert 速度反馈，但名义落脚点返回值没有使用该反馈。
2. 候选平面的附加评分恒为零，没有腿过伸和向身体内侧跨步惩罚。
3. 每个 MPC 周期都会重新选择落脚平面，摆动后半程可能随地图更新跳面。
4. 摆动中点只使用起落脚高度最大值和固定摆高，没有考虑路径中间障碍。

论文的完整系统使用 48 维 loop-shaping 状态、全身关节状态、关节限制、膝部碰撞、multiple-shooting SQP 和专用 WBC。本文只移植与当前 K20 架构兼容的参考规划算法，不宣称完整复现论文控制器。

## 3. 目标

- 对当前处于摆动期的腿，其本次触地点应用带二维限幅、死区和滤波的 Raibert 修正。
- 使用 K20 模型派生的规划髋坐标系评价候选点的腿过伸和内跨风险。
- 从摆动进度 50% 起冻结本次触地决策，避免地图更新导致临近触地跳面。
- 沿起落脚 XY 路径查询障碍高度，通过现有 `maxHeightSequence` 增加摆动净空。
- 四项功能独立开关；新增配置缺失时保持旧行为。
- 保留同一套逻辑处理平地、缓坡和低台阶，不增加地形模式切换。
- 使用单元测试、Gazebo 和 K20 实机低速测试逐级验收。

## 4. 非目标

- 不引入论文的 48 维 loop-shaping 状态。
- 不修改 MPC 状态/输入维度、centroidal 动力学、SQP/HPIPM、WBC 或状态估计。
- 不在本阶段加入 MPC 内关节位置、速度或力矩约束。
- 不移植完整 `ocs2_switched_model_interface`，也不替换当前 ReferenceManager。
- 不把现有三次摆动样条替换为论文/示例中的五次样条体系。
- 不新增真实基座线速度到 ReferenceManager 的数据通路。
- 不在参考规划层接入实际接触测量；冻结状态按计划接触事件清理。
- 不改变其他机器人配置的默认行为。

## 5. 论文、OCS2 与 K20 的对应关系

| 功能 | 论文 | OCS2 示例 | K20 修订方案 |
|---|---|---|---|
| Raibert 修正 | 式 (12)，第一即将触地的落脚点 | 世界系基座速度差，只修正当前摆动腿的下次触地 | 使用世界系 CoM 速度代理，二维限幅后修正当前摆动腿触地点 |
| 平面评分 | 式 (13)，距离加运动学惩罚 | 触地开始/结束两次评估过伸和内跨 | 保持相同代价结构，使用 K20 规划髋坐标系 |
| 后半程冻结 | 摆动后半程约束不再改变 | `previousFootholdTimeDeadzone` 按距触地时间复用 | `freezePhase=0.5`，按归一化摆动相位冻结 |
| 摆动净空 | 五次样条越过路径最高地形 | 地图分辨率高度剖面、相对连线最高障碍、限幅 | 计算绝对 `maxHeightSequence`，继续使用现有三次样条 |

`freezePhase`、Raibert 限幅和障碍净空裕量是面向 K20 实机安全性的适配，不是 OCS2 的逐行复制。

## 6. 总体架构

```text
当前 centroidal 状态 + 目标轨迹 + Trot 接触时序
                         |
                         v
        当前摆动腿的第一触地点 Raibert XY 修正
                         |
                         v
       候选平面距离 + K20 规划髋坐标运动学惩罚
                         |
                         v
                  生成凸落脚区域
                         |
                         v
         按接触事件匹配并在摆动后半程冻结
                         |
                         v
       从同一地图快照的障碍层查询路径高度剖面
                         |
                         v
        maxHeightSequence -> 现有三次摆动样条
                         |
                         v
                 现有 MPC 足端软约束
```

职责划分：

- `PerceptiveLeggedInterface`：加载统一配置；向选择器提供 Pinocchio 模型或预计算的腿根信息。
- `ConvexRegionSelector`：Raibert 修正、历史落脚滤波、K20 运动学评分、凸区域生成、接触事件匹配、值对象冻结和障碍层采样。
- `PerceptiveLeggedReferenceManager`：组织接触相的起落脚高度，向选择器查询地形增高，构造 `maxHeightSequence`。
- `SwingTrajectoryPlanner`：保持现有实现不变；继续将 `maxHeightSequence + scaling * swingHeight` 作为三次样条中点高度。

## 7. 统一配置

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

兼容规则：

- 配置组或任一开关缺失时，相应功能按关闭处理。
- 开关关闭时不得改变当前输出。
- `legRootJointByContact` 仅在运动学惩罚开启时要求存在并有效。
- `ModelSettings::contactNames3DoF` 当前是 `ModelSettings.h` 中的硬编码默认值 `LF_FOOT, RF_FOOT, LH_FOOT, RH_FOOT`；`loadModelSettings()` 不从 `task.info` 加载该列表。
- 加载腿根时遍历运行时的 `modelSettings.contactNames3DoF`，按足端名称查询 `legRootJointByContact`，禁止以两个数组的相同下标建立关联。这样当前顺序或未来顺序变化都不会把 LH/RF 腿根错配。
- `PerceptiveLeggedInterface` 的启动占位地图只有全零的 `elevation_before_postprocess` 和 `smooth_planar`，没有 `elevation`；占位层不得满足地形净空的数据有效条件。
- 地图层在启动时尚不可用，不因缺层阻止控制器启动；但未通过第 11.1 节实时消息验证前，`enableTerrainClearance` 必须保持 `false`。
- 开发和仿真阶段四项默认关闭，逐项验证；全部验收后只在 K20 感知配置中默认开启。

启动参数验证：

- 所有标量必须有限。
- 高度、距离、裕量和权重不得为负。
- `previousFootholdFactor` 与 `freezePhase` 必须位于 `[0,1]`。
- `contactTimeMatchTolerance` 必须大于等于零。
- 运动学功能开启时，每个运行时 3-DoF contact 必须恰有一个映射；四个腿根 joint 必须存在、互不重复，且在 base 坐标系中的横向偏移绝对值大于小阈值。缺项、未知 contact 或重复 joint 均启动失败。

## 8. Raibert 第一触地点修正

### 8.1 速度语义

论文和 OCS2 使用世界系基座线速度。K20 的 centroidal 状态不直接包含基座线速度，当前可直接读取：

```text
centroidal_model::getNormalizedMomentum(state, info).head<3>()
```

对 FullCentroidalDynamics，该量是质量归一化线动量，即世界系 CoM 速度。本阶段明确采用它作为 Raibert 速度代理：

\[
\Delta p_{xy} =
\sqrt{\frac{h}{g}}
(v_{\mathrm{CoM,measured},xy}-v_{\mathrm{CoM,desired},xy})
\]

代码、诊断和测试统一使用 `ComVelocity` 命名，不把该量称为 base velocity。若以后需要严格使用基座速度，必须单独设计状态估计到 ReferenceManager 的数据通路。

### 8.2 适用接触事件

- 当前腿已经处于摆动期时，只修正其第一个有计划触地开始时间的接触事件。
- 当前支撑腿不修正；预测范围内更晚的第二个及以后触地事件不修正。
- 当前支撑足位置始终沿用现有逻辑。
- 新步态刚进入且没有完整起飞/触地边界时，不制造虚假的 Raibert 事件。

这与 OCS2 示例“只给当前摆动腿的下一触地点添加反馈”的实际行为一致。

### 8.3 计算和提交顺序

1. 使用现有期望足端运动学与坡度补偿得到 `p_kinematic`，保持旧基线。
2. 检查实测/目标 CoM 速度有限性；无效时本周期偏移置零。
3. 计算 `delta_xy`，z 固定为零。
4. 按二维模长限制到 `raibertMaxOffset`，不分别裁剪 x/y。
5. 得到 `p_raw = p_kinematic + delta_xy`。
6. 使用同一接触事件上一周期已接受触地点的 XY 做死区和低通；z 保持 `p_raw.z()`，避免跨不同高度平面插值。
7. 以过滤后的启发点执行候选平面投影与运动学评分。
8. 只有候选选择成功并提交后，才更新该事件的历史触地点；失败候选不污染历史。

二维滤波：

\[
p_{xy,filtered}=\lambda p_{xy,previous}+(1-\lambda)p_{xy,new}
\]

当新旧 XY 距离小于 `previousFootholdDeadzone` 时直接保留旧 XY。

只过滤 XY 是 K20 的明确适配；OCS2 示例会对三维参考点整体滤波。

## 9. K20 运动学候选平面惩罚

### 9.1 为什么不能直接使用原生 Hip frame

K20 的 `LF_HAA/LH_HAA/RF_HAA/RH_HAA` 在 URDF 中均以 `rpy="0 0 0"` 安装。若直接使用这些原生坐标系，OCS2 的

```text
inwardDirection = gravityNormalInHip × UnitX
```

会在左右腿得到相同方向，右腿的内跨判断错误。此外，`*_hip` link 位于 HAA 关节之后，会随 HAA 角度旋转，也不是 OCS2 所指的固定 leg-root frame。

### 9.2 K20 规划髋坐标系

每条腿从 Pinocchio 模型读取 HAA 根关节在 base 中的固定位置。根据该位置的 base-y 符号构造固定规划髋坐标：

- 左腿：规划髋方向与 base 方向一致。
- 右腿：在 base 坐标中绕 z 旋转 `pi`，使 OCS2 的正 x 转动约定仍表示向外。
- 规划髋原点位于对应 HAA 根关节位置。
- 只随期望 base 位姿移动，不随 HAA/HFE/KFE 关节角旋转。

禁止通过腿数组下标或世界系 y 符号硬编码左右；左右由 URDF 中腿根相对 base 的静态横向位置派生，并通过单元测试验证。

### 9.3 惩罚定义

候选点转换到规划髋坐标系：

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

候选平面总代价沿用 `convex_plane_decomposition`：

\[
J=\|p_{heuristic}-p_{projected}\|^{2}+J_{kin,start}+J_{kin,end}
\]

其中 `start` 是预计接触开始，`end` 是预计接触结束。两次 hip pose 均由对应时刻的期望 base pose 和固定腿根变换得到。

异常回退：

- 腿根名称无效：运动学功能开启时启动失败。
- 单周期期望 base pose 或候选点非有限：该周期运动学附加惩罚置零，保留距离选择。
- 不运行完整逆运动学，不修改 MPC 内关节约束。

### 9.4 `nominalLegExtension` 的 Gazebo 标定门槛

`nominalLegExtension=0.62 m` 只作为保守初值，不视为已验证阈值。K20 的大腿和小腿各为 `0.35 m`，并且 HAA 到 HFE、HFE 到 KFE 还分别包含 `0.093/0.0495 m` 和 `0.145 m` 偏移；忽略关节限位的 HAA 根到足端直腿几何上界约为 `0.733 m`。由于 KFE 上限为 `-0.698 rad`，该几何上界不是实际可达阈值，最终值必须通过带关节限位的 FK 扫描和 Gazebo 轨迹数据确定。

在开启运动学惩罚前增加 Gazebo 标定步骤：

1. 先令 `enableKinematicPenalty=false`，或令权重为零只发布诊断，执行站立、原地 Trot、`0~0.20 m/s` 加减速、8° 缓坡和 0.05 m 台阶。
2. 记录四腿在接触开始和结束时的 `||p_foot^hip||` 分布、最大值及对应关节状态，并用 K20 关节限位做离线 FK 可达域扫描。
3. 阈值应高于正常工况观测上界并保留测量/模型误差裕量，同时低于关节限位下的危险伸展区；四腿差异超出模型误差时先排查坐标或映射，不用单一阈值掩盖。
4. 若 `0.62 m` 在正常步态持续产生惩罚，则在 G1 前完成调整；不得把该标定推迟到实机阶段。

## 10. 摆动后半程冻结

### 10.1 冻结对象必须值拥有

每条腿保存 `FootholdDecisionSnapshot`：

```text
contactStartTime
contactEndTime
filteredHeuristicXY
projectionPositionInWorld
planeTransformToWorld
convexPolygon
touchdownHeight
frozen
valid
```

禁止在该快照中保存：

- `PlanarTerrainProjection::regionPtr`
- 指向 `planarTerrain_.planarRegions` 的引用或迭代器
- 指向上一 MPC 周期临时容器的地址

地图每周期复制或替换后，旧 `regionPtr` 可能失效。恢复冻结结果时应使用快照中的变换、位置和多边形重建本周期输出。

### 10.2 接触事件匹配

两个事件在以下条件同时满足时视为同一事件：

```text
abs(newStart - oldStart) <= contactTimeMatchTolerance
abs(newEnd   - oldEnd)   <= contactTimeMatchTolerance
```

若接触结束时间在预测范围外，使用当前可用的扩展结束时间，并在后续周期按同一容差更新匹配。

### 10.3 冻结行为

摆动进度：

\[
s=\operatorname{clip}\left(
\frac{t-t_{liftoff}}{t_{touchdown}-t_{liftoff}},0,1
\right)
\]

- `s < freezePhase`：允许更新启发点、候选平面和凸区域；成功结果写入快照。
- `s >= freezePhase`：复用上一成功快照，不再查询新平面或重建新多边形。
- 第一次观察到某事件时若已经超过冻结阈值，则接受本周期第一个有效选择并立即冻结。
- 后半程时序小幅漂移时按容差匹配并保持冻结。
- 后半程完全无法匹配新时序时，保留最后有效快照至缓存的计划触地时间，避免临时跳面。
- 进入计划支撑相、新摆动事件、步态切换、时间超过缓存触地时间容差或 STANCE 后清除旧快照。
- 冻结只固定参考落脚决策，不锁定 MPC 状态、接触力或实际足端轨迹。

ReferenceManager 当前没有实际接触测量，因此本阶段不能声称按“实际触地”清理；清理依据是计划接触相和时间边界。

## 11. 地形感知摆动高度

### 11.1 地图层来源和选择

源码中存在两个 `elevation_before_postprocess` 来源，必须区分：

1. `PerceptiveLeggedInterface::setupOptimalControlProblem()` 在订阅到地图前手动创建 5×5 m 全零占位地图，其中包含 `elevation_before_postprocess` 和 `smooth_planar`。它只用于控制器/SDF 初始化，不是感知结果。
2. 实际 `PlaneDecompositionPipeline::update()` 会先对输入 `elevation` 补洞、去噪和重采样，再由 `Postprocessing::postprocess()` 把此时的数据复制为 `elevation_before_postprocess`。随后原 `elevation` 继续执行非平面水平膨胀和高度偏移，并与 `smooth_planar` 一起通过 `PlanarTerrain` 消息发布。

当前 K20 的 `height_layer` 为 `elevation`。`elevation_raw` 和 `segmentation` 是在 `PlanarTerrain` 发布之后才加入 `filtered_map` 的调试层，控制器订阅的 `PlanarTerrain` 不包含它们。OCS2 `SegmentedPlanesTerrainModel` 的路径高度剖面和 SDF 都直接读取 `elevation`，因此本设计固定使用后处理后的 `elevation`：

- 障碍高度层在实现中固定为 `elevation`，不新增可切换的层名配置。
- 不回退到 `elevation_before_postprocess`，避免把启动占位平面当成实时障碍地图。
- 不回退到 `smooth_planar`，该层用于机身参考且可能平滑掉台阶和窄障碍。
- `elevation` 已包含当前配置的非平面 0.03 m 高度偏移和 3 格水平膨胀；`terrainClearanceMargin` 是额外摆动安全裕量，二者叠加效果必须在 Gazebo 调整。

`enableTerrainClearance` 的启用前硬门槛是从正在运行的感知栈至少采集一条实时消息：

```bash
ros2 topic list | rg '^/convex_plane_decomposition_ros/planar_terrain$'
ros2 topic echo --once /convex_plane_decomposition_ros/planar_terrain \
  | rg -n 'layers:|elevation|elevation_before_postprocess|smooth_planar'
```

必须在消息的 GridMap layer 列表中确认 `elevation` 存在且含有限样本。只在启动占位地图、RViz 配置、接收器回退代码、参数文件或源码的 `Postprocessing` 中看到层名不算运行验证。若没有活跃 publisher、命令超时或 `elevation` 不存在，检查不通过，`enableTerrainClearance` 继续保持 `false`。

2026-07-30 本次检查结果：当前 ROS graph 只有 `/parameter_events` 和 `/rosout`，目标 topic 没有活跃 publisher，因此本项仍为未验证；启动完整感知栈后必须重跑以上命令。

即使 `enableTerrainClearance=true`，当前地图只有启动占位层、缺少 `elevation` 或没有有限样本时，本周期也必须回退固定摆高，将 `terrainClearanceActive=false` 并输出节流警告/诊断。存在 `elevation_before_postprocess` 本身不能使该状态变为 active。

高度查询使用 `ConvexRegionSelector` 当前周期持有的 `planarTerrain_` 地图副本，使候选平面、凸区域和摆高来自同一地图快照。

### 11.2 路径采样

- 沿起落脚 XY 连线遍历所有相交栅格，采样间距由地图分辨率决定，不额外引入固定分辨率参数。
- 每个栅格只使用选定障碍层的有限值。
- NaN 栅格跳过；没有有限样本时回退固定摆高。
- 起落脚点处于同一栅格或 XY 距离小于一个分辨率时，不查询中间障碍，使用起落脚高度最大值。
- 地图外查询、无有效 index 或异常返回均回退固定摆高。

对第 i 个有效样本：

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

### 11.3 接入现有三次样条

传给现有接口的量不是最终摆动中点高度，而是：

\[
maxHeightSequence=\max(h_{lift},h_{touch})+h_{adapt}
\]

现有 `SwingTrajectoryPlanner` 继续计算：

\[
h_{mid}=maxHeightSequence+scaling\cdot h_{swing}
\]

这样不会重复加入 `swingHeight`。功能关闭或回退时 `h_adapt=0`，输出与当前实现一致。

K20 初值：

```text
terrainClearanceMargin  0.03  # m
maxTerrainAdaptation    0.12  # m
swingHeight             0.15  # 现有配置
```

完整摆动时最大中点增高为 `0.15 + 0.12 = 0.27 m`。

## 12. 候选选择的提交与异常回退

每个接触事件按“计算候选 -> 验证 -> 提交”处理：

1. 生成原始和过滤后的启发点。
2. 构造非负运动学评分函数。
3. 调用 `getBestPlanarRegionAtPositionInWorld`。
4. 验证 `regionPtr`、投影位置、平面变换和凸多边形均有效。
5. 转换为本周期输出和值拥有快照。
6. 最后更新历史滤波点和冻结状态。

回退表：

| 异常 | 回退行为 |
|---|---|
| CoM 速度非有限 | 本周期 Raibert 偏移置零 |
| Raibert 偏移过大 | 按 XY 模长限制 |
| 运动学输入非有限 | 本周期运动学惩罚置零 |
| 没有有效候选平面 | 使用同一事件上一有效值；无历史时沿用当前错误处理并给出明确诊断 |
| 冻结期地图更新 | 使用值拥有快照，不访问旧 region 指针 |
| 后半程时序小幅漂移 | 容差匹配并继续冻结 |
| 后半程时序完全失配 | 保持缓存结果至计划触地边界 |
| 只有启动占位的 `elevation_before_postprocess` | 固定摆高，置 `terrainClearanceActive=false`；不得判为实时地图 |
| 启用前未验证实时 `elevation` | 保持 `enableTerrainClearance=false`，禁止进入地形净空功能 |
| 运行中 `elevation` 缺失或无有限值 | 固定摆高，置 `terrainClearanceActive=false` 并输出节流警告/诊断 |
| 路径存在少量 NaN | 跳过无效栅格 |
| 地形尖峰 | 限制到 `maxTerrainAdaptation` |

## 13. 诊断与可观测性

每条腿、每个当前摆动事件记录：

- 实测/目标 CoM 速度，明确标注不是 base velocity。
- 原始、限幅后和过滤后的 Raibert XY 偏移。
- 上一历史触地点是否命中死区、是否提交更新。
- 候选平面的距离代价、触地开始运动学代价和接触结束运动学代价。
- 规划髋坐标系的左右判定、腿根位置和内跨方向。
- 被选平面、投影点、凸区域和快照有效性。
- 摆动进度、事件匹配结果、冻结状态和冻结原因。
- 地图是启动占位还是包含有效 `elevation` 的接收地图，以及有限样本数、最高障碍、增高限幅和最终 `maxHeightSequence`。
- `terrainClearanceActive` 必须反映本周期是否真正使用了有效障碍层，不能仅复述配置开关。
- 每项功能是否进入回退路径。

高频数据进入现有调试发布接口或节流日志，禁止在 MPC 周期持续输出普通日志。

## 14. 测试设计

### 14.1 纯函数和单元测试

Raibert：

- 零、正、负 CoM 速度误差。
- 只修正当前摆动腿的第一触地点；当前支撑腿和更远触地点不变。
- 二维模长限幅，不产生 z 偏移。
- XY 死区、低通和不同地形高度下 z 不参与滤波。
- 非有限实测/目标速度回退为零偏移。
- 候选未提交时历史值不改变。

运动学惩罚：

- 按足端名称解析 `legRootJointByContact`，分别以 `LF, RF, LH, RH` 和打乱后的运行时 contact 顺序测试，得到相同映射。
- 缺失 contact、未知 contact、重复 joint 和旧式并行数组均拒绝启用，防止 LH/RF 错配。
- 从 K20 URDF 派生四条腿的左右侧和固定规划髋坐标。
- LF/LH 向身体内侧跨步产生惩罚，向外不惩罚。
- RF/RH 向身体内侧跨步产生惩罚，向外不惩罚。
- 原生四个 HAA 均为零旋转时测试仍通过，防止误用原生 Hip frame。
- 正常可达、刚好阈值、腿过伸和非有限输入。
- 接触开始与结束评分累加。
- 几何距离相近时选择运动学更合理的平面。

冻结：

- `freezePhase` 前后行为和阈值边界。
- 首次观察已超过阈值时选择一次后立即冻结。
- 地图对象整体替换后，冻结结果不访问旧 `regionPtr`。
- 接触开始/结束时间容差匹配。
- 后半程时序完全失配时保持至缓存触地边界。
- 进入计划支撑相、新事件、步态切换和 STANCE 后清理。

地形摆高：

- 平地保持原摆高。
- 上坡、下坡、中间台阶和路径外障碍。
- 固定使用 `elevation`，不读取 `smooth_planar` 或 `elevation_before_postprocess`。
- 启动占位地图虽包含有限的全零 `elevation_before_postprocess`，地形净空仍必须 inactive。
- 收到含有限 `elevation` 的真实 `PlanarTerrain` 后才允许 active；运行中该层缺失或全无效时显式回退。
- 后处理的非平面高度偏移/水平膨胀与 `terrainClearanceMargin` 叠加后的 Gazebo 标定。
- NaN、全无效、地图外、异常尖峰和同栅格起落点。
- 验证 `maxHeightSequence` 不包含 `swingHeight`，防止重复加高。

兼容性：

- 四项开关关闭时，落脚点、平面、凸区域和摆动高度与当前实现一致。
- 配置组完全缺失时采用旧行为。
- P1 和非感知控制配置不受影响。

### 14.2 Gazebo

G0 基线：四项关闭，记录 MPC 频率、求解时间、速度误差、落脚点变化、足端高度和安全故障。

G0.5 运动学阈值标定：保持运动学惩罚不生效，只发布第 9.4 节诊断；完成带关节限位的 FK 扫描和站立、Trot、缓坡、台阶数据采集，确定 `nominalLegExtension` 后才能进入 G1。

G1 平地：

- 依次开启 Raibert、运动学惩罚、冻结和地形净空。
- Trot 从 0 加速至 0.20 m/s，再减速至 0。
- 加入沿机身前后方向 150 N、持续 0.20 s 的推扰。
- 四项全开连续运行不少于 60 s。

G2 缓坡：上坡和下坡各 8°，Trot 速度 0.15 m/s。

G3 低台阶：台阶高度 0.05 m，Trot 速度 0.10 m/s。

Gazebo 通过标准：

- 无摔倒、NaN、MPC 异常退出和新增安全故障。
- 摆动后半程落脚区域、平面变换和触地点高度保持不变。
- 足端目标不发生跨平面跳变。
- 台阶场景的摆动参考高于检测到的路径障碍。
- 四项关闭时保持基线行为。
- `nominalLegExtension` 在 G0.5 完成标定，正常步态不出现持续误惩罚。
- 地形净空启用前已从实时消息验证 `elevation`；运行中若故障注入移除该层，诊断必须变为 inactive 并产生节流警告。
- MPC 保持实际 60 Hz，无持续超时。
- 新增参考规划耗时相对基线增幅不超过 20%。

### 14.3 K20 实机

测试配备急停、保护绳并保留单项关闭能力。

H0 静态检查：

- 地图、CoM 速度和足端数据均有限。
- 四腿规划髋方向、左右内跨惩罚和腿根位置正确。
- 原地抬腿时冻结和摆高诊断正确。

H1 平地：初始 0.10 m/s，连续 Trot 20 s；按 Raibert、运动学惩罚、冻结、全部开启的顺序验证，通过后提高至 0.20 m/s。

H2 缓坡：初始坡度 5°，Trot 速度 0.10 m/s。

H3 低台阶：初始台阶 0.03 m、速度不高于 0.10 m/s；通过后再提高至 0.05 m。

立即终止条件：

- MPC 或 WBC 出现非有限值。
- 触发关节位置、速度或力矩安全保护。
- 摆动后半程发生跨平面切换。
- 任一足端目标发生超过 0.05 m 的单周期位置跳变。
- 连续两步拖脚或碰撞台阶。
- MPC 持续低于目标频率。
- 机身姿态或速度误差持续扩大。

## 15. 预计涉及文件

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

- `legged_interface/constraint/SwingTrajectoryPlanner.h/.cpp`：已有三参数 `update(..., maxHeightSequence)` 接口足够。
- MPC 动力学、求解器、WBC、状态估计和硬件接口。
- `PerceptiveController.cpp` 与 `PlanarTerrainReceiver.cpp`：地形净空直接从当前 `PlanarTerrain` 地图副本显式选层，不改变现有 SDF 层策略。

如实现过程中发现必须触及“预计不修改”文件，应先说明原因并更新本文档，不能顺手扩大范围。

## 16. 实施顺序

1. 启动完整感知栈，执行第 11.1 节 `ros2 topic echo`，确认实时 `PlanarTerrain` 包含有限的 `elevation`；验证失败时停止进入实现，先修复地图发布链路。
2. 新增配置结构和配置加载测试，验证按 contact name 映射腿根，且缺失配置保持旧行为。
3. 建立 Raibert 纯函数测试，确认 CoM 速度语义、二维限幅、死区和提交顺序。
4. 实现 Raibert 修正，保持开关默认关闭。
5. 建立 K20 规划髋坐标和运动学惩罚测试，再接入候选评分。
6. 建立值拥有快照和接触事件匹配测试，再实现后半程冻结。
7. 建立障碍层选择和高度剖面测试，再构造 `maxHeightSequence`。
8. 验证四项关闭时与旧输出一致。
9. 按 G0、G0.5、G1、G2、G3 完成 Gazebo 验收和运动学阈值标定。
10. 按 H0、H1、H2、H3 完成 K20 实机低速验收。
11. 全部通过后，将 K20 感知配置中的四项开关设为默认开启。

## 17. 成功标准

- 四项功能可独立开启和关闭。
- 配置缺失或全部关闭时不改变当前 K20 行为。
- 诊断明确区分 CoM 速度代理与论文的 base velocity。
- 腿根由 contact name 显式映射，运行时 contact 顺序改变不会造成腿根错配。
- 左右四腿内跨方向由 K20 模型正确派生，不受四个 HAA 零旋转影响。
- `nominalLegExtension` 在 Gazebo 阶段完成标定，不能把正常步幅持续判成过伸。
- 冻结状态不保留任何跨地图周期裸指针。
- 摆动后半程不再切换落脚平面、凸区域或触地点高度。
- 地形净空启用前验证实时 `elevation`，启动占位的 `elevation_before_postprocess` 不得激活功能；不读取 `smooth_planar`，运行中缺层会显式降级，台阶路径能自动提高摆动足净空。
- `maxHeightSequence` 与现有 `swingHeight` 只各加入一次。
- 平地、缓坡和低台阶无需切换规划模式。
- 单元测试、Gazebo 和实机低速验收达到本文门槛。

## 18. 实现前最终检查清单

- [x] 设计决定：使用世界系 CoM 速度作为本阶段 Raibert 代理，并在诊断中明确它不是 base velocity。
- [x] 设计决定：使用 `freezePhase=0.5` 实现论文“摆动后半程冻结”；该值可在 Gazebo 数据表明必要时再标定。
- [ ] K20 Pinocchio 模型中 `LF_FOOT -> LF_HAA`、`RF_FOOT -> RF_HAA`、`LH_FOOT -> LH_HAA`、`RH_FOOT -> RH_HAA` 均可解析且唯一。
- [ ] 启动完整感知栈后，从实时消息确认障碍地图包含有限的 `elevation`；2026-07-30 检查时无目标 topic publisher。
- [ ] 在 G0.5 完成 `nominalLegExtension` 标定，确认正常步态无持续误惩罚。
- [ ] 新测试可在不启动 Gazebo 的情况下覆盖四项核心算法。
- [ ] 原设计文档保留，新实现仅以本修订版为基线。
