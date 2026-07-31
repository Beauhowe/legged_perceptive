# K20 感知落脚规划实现就绪版审核记录

日期：2026-07-31
审核对象：`2026-07-30-k20-perceptive-foothold-planning-design-implementation-ready.md`
审核标准：贴合论文/OCS2 思想、只改必要部分、不破坏原有稳定性

## 1. 总体判断

三个标准里，**贴合论文/OCS2** 和 **不破坏原有稳定性** 均达到。**只改必要部分** 上有三处可收紧。另有一处实质性遗漏和一处实现细节遗漏，建议在实现前补入文档。

上一轮审核（`2026-07-30-k20-perceptive-foothold-planning-design-review.md`）提出的六条建议已全部吸收：

- `frozenRegion shared_ptr<const PlanarRegion>` 方案已进入 §8.1。
- `PerceptiveLeggedPrecomputation` 明确归入"预计不修改"并说明理由（§14）。
- 全局 `STANCE` 清理条件已澄清（§8.4）。
- 运行时验证门槛不阻塞离线实现（§15）。
- 配置顺序问题已由 contact name 显式映射解决（§5）。
- 文件列表完整（§14）。

## 2. OCS2 对齐核实（全部通过）

| 设计条目 | OCS2 代码位置 | 核实结论 |
|---|---|---|
| 只修正当前摆动腿的第一触地点 | `SwingTrajectoryPlanner.cpp:337-342`：`contactCount==0` 才加 `zmpReactiveOffset`；`++contactCount` 在 `hasStartTime` 判断**之外**，支撑腿起步时首个未来接触相的 `contactCount` 已是 1，拿不到反馈 | 与 §6.2 描述一致 ✅ |
| 启发点在接触相中点求值 | `middleContactTime = 0.5*(contactEndTime+contactPhase.start)`（第 328 行） | 与现有 K20 `standMiddleTime` 语义相同，无需改动 ✅ |
| 运动学评分在触地开始和接触结束两次求值 | `basePoseAtTouchdown` 取 `contactPhase.start`，`basePoseAtLiftoff` 取 `contactEndTime`（第 398-403 行） | 与 §7.2 一致 ✅ |
| 倒立摆频率 `sqrt(h/g)` | `SwingTrajectoryPlanner.cpp:308`：`pendulumFrequency = sqrt(settings_.invertedPendulumHeight / 9.81)` | 与 §6.1 公式一致 ✅ |
| OCS2 按"距触地剩余时间"冻结，K20 改为"归一化摆动相位" | `timeTillContact < previousFootholdTimeDeadzone`（第 383 行）vs. §8.4 `s >= freezePhase` | 已在设计中明确标注为 K20 适配而非逐行复制 ✅ |
| `ModeSchedule::modeAtTime()` 存在 | `ModeSchedule.h:67` | §8.4 清理条件可实现 ✅ |
| `PlanarRegion` 只含三个值类型字段，深拷贝安全 | `PlanarRegion.h:32-41`：`boundaryWithInset` / `bbox2d` / `transformPlaneToWorld`，无裸指针无句柄 | `make_shared<const PlanarRegion>(*regionPtr)` 是干净的值语义拷贝 ✅ |
| 摆动净空从 `elevation` 查询高度剖面 | OCS2 `SegmentedPlanesTerrainModel` 直接读取 `elevation` | 与 §9.1 一致 ✅ |

**关于 `frozenRegion` 是否属于过度设计的反向验证：**

替代方案一（冻结时不更新 projection，继续指向当前地图）：因为 `ConvexRegionSelector.cpp:85` 每周期整体拷贝地图，`planarRegions` vector 可能重新分配，旧指针必然可能悬空。

替代方案二（整份 `PlanarTerrain` 双缓冲）：开销远大于单个 region，且四腿冻结时间不同，无法用单一缓冲解决。

结论：给定 `regionPtr` 这个既有接口不变，深拷贝单个 region 就是最小解。

## 3. K20 几何数值核实

KFE→足端偏置：`xyz="0 0 -0.35"`（`k20.urdf`）

独立计算：

| 段 | URDF 偏置向量 | 距离 |
|---|---|---|
| HAA→HFE | `(0.093, 0.0495, 0)` | 0.1054 m |
| HFE→KFE | `(0, 0.145, -0.35)` | 0.3788 m |
| KFE→FOOT | `(0, 0, -0.35)` | 0.3500 m |
| 三段标量和（三角上界） | — | **0.8342 m** |
| 零角构型 HAA→足端实距 | `(0.093, 0.1945, -0.70)` | **0.7324 m** |

§7.3 写 `0.834 m` 和 `0.732 m`，均与计算一致。

## 4. 一处实质性遗漏：`h_lift`/`h_touch` 来源未指定

**影响：** 可能导致三次样条中点低于端点（`h_mid < max(h_lift, h_touch)`），在坡地或台阶上出现摆动轨迹下凹。

现有 `updateSwingTrajectoryPlanner`（`PerceptiveLeggedReferenceManager.cpp:129-142`）的数据流：

```
projections = getProjections(leg)       // 值拷贝
modifyProjections(...)                  // 就地改写，把支撑相 z 替换为 lastLiftoffPos_[leg].z - 0.02
getHeights(contactFlags, projections)   // 从改写后的拷贝取 h_lift / h_touch
```

`modifyProjections` 会覆盖支撑相的 `positionInWorld`，所以样条端点高度来自**改写后**的 projections，而不是选择器存储的原始投影 z 值。

§9.3 写 `maxHeightSequence = max(h_lift, h_touch) + h_adapt`，但没说路径采样的端点 XY 和 `h_lift`/`h_touch` 从哪里取。若取选择器快照的原始值，`maxHeightSequence` 的基准高度与 `liftOffHeightSequence`/`touchDownHeightSequence` 来源不同，会造成三次样条端点和中点基准不一致。

**建议在 §9.3 开头补充：**

> 路径采样的起落脚 XY 端点、`h_lift`/`h_touch` 以及 `maxHeightSequence` 的基准，必须取自 `modifyProjections()` 之后、与同一次 `getHeights()` 调用同源的那份 `projections` 拷贝。`ConvexRegionSelector` 的地形采样调用应在 `PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner` 内 `modifyProjections` 之后执行，或传入已修正的高度值。

## 5. 一处实现细节遗漏：冻结恢复需覆盖全部 phase 下标

`ConvexRegionSelector::update()` 的去重逻辑（`ConvexRegionSelector.cpp:129-145`）：同一接触事件跨多个 phase 下标时，后续下标直接拷贝前一个下标的 projection/polygon，只有 `middleTime` 变化时才重新求解。

§8.4 和 §10 描述了按接触事件匹配和冻结的行为，但没有明确要求冻结恢复时也必须覆盖该事件对应的**全部 phase 下标**。若只填入一个下标，`getProjection(leg, t)` 在其他时刻会拿到空 `regionPtr`，`PerceptiveLeggedPrecomputation` 会把该腿当作摆动腿跳过，冻结期间足端软约束局部消失。

**建议在 §8.1 或 §10.1 补充：**

> 冻结恢复与正常候选提交都必须覆盖该接触事件对应的全部 phase 下标，保持现有 `lastStandMiddleTime` 去重语义。

## 6. 三处可收紧的细节（"只改必要部分"）

### 6.1 `planeTransformToWorld` 字段冗余

`FootholdDecisionSnapshot` 同时含 `planeTransformToWorld` 和 `frozenRegion`。深拷贝后 `frozenRegion->transformPlaneToWorld` 与 `planeTransformToWorld` 应始终相同，两份真值来源在维护时有不一致风险。

建议：删掉 `planeTransformToWorld`，需要时直接访问 `snapshot.frozenRegion->transformPlaneToWorld`。

同理，`valid` 可直接由 `frozenRegion != nullptr` 派生，无需独立维护；`projectionCost` 在当前代码中没有消费方（`getFootPlacements` 和 `PerceptiveLeggedPrecomputation` 均不读 `cost` 字段），若保留应注明"仅结构完整性保留，实现中可为零"。

### 6.2 `ros2 topic echo` 验证命令过于复杂

§9.2 的 `--filter` 表达式有三个脆弱点：

1. `--once`、`--field`、`--filter` 组合在不同 ROS 2 patch 版本行为有差异。
2. 表达式依赖 `gridmap.layers` 与 `gridmap.data` 数组下标严格对应，这是 grid_map_msgs 的当前实现细节，非接口约定。
3. 表达式失败（Python 异常）会静默返回空，与"没有 publisher"无法区分。

字段路径本身是对的（`PlanarTerrain.msg` 含 `gridmap`，`GridMap.msg` 含 `layers` 和 `data`），但建议改为两步：

```bash
# 步骤一：确认有 publisher 且 layer 列表出现 elevation
timeout 10s ros2 topic echo --once --field gridmap.layers \
  /convex_plane_decomposition_ros/planar_terrain

# 人工确认输出中包含 "elevation"，且命令没有超时
# 有限性检查在运行时 terrainClearanceActive 诊断和单元测试中覆盖，不在此命令里做
```

### 6.3 §11.4 有一条测试项描述含义不清

> projection 仍引用 owner 时尝试清理快照，测试必须阻止悬空引用

"测试必须阻止"是无法断言的意图，不是可执行的测试用例。真正要验证的是第 §4 不变量 3 的提交顺序：先构造不引用旧 owner 的新输出，再释放旧快照。

建议改为：

> 用 `weak_ptr` 观测 `frozenRegion` 的引用计数。先构造新输出并替换 `feetProjections_`，验证此时 `weak_ptr.expired() == false`（旧 owner 仍被新输出引用前不应被释放）；释放旧快照后，验证 `weak_ptr.expired() == true`（无引用时可安全析构）。

## 7. 其余章节核实结论

| 章节 | 结论 |
|---|---|
| §1 设计结论表 | 七条决策均有代码依据 |
| §2 OCS2 对应关系表 | 与 OCS2 源码一致 |
| §3 目标/非目标 | 范围划定合理，无意外越界 |
| §4 架构图和生命周期不变量 | 不变量 1–5 准确，能有效防止悬空指针和候选污染 |
| §5 统一配置 | `legRootJointByContact` map 格式可被 Boost ptree info 解析；参数验证规则完整 |
| §6 Raibert | 速度语义、适用条件、提交顺序均正确；XY 专项滤波是已说明的 K20 适配 |
| §7.1–7.2 运动学惩罚 | π 旋转逻辑和惩罚公式均已在上一轮审核中验证 |
| §8.1 快照 | `frozenRegion` 完整深拷贝方案已解决上一轮的关键设计空白 |
| §8.2–8.4 冻结行为 | 状态机覆盖完整；清理条件已澄清 |
| §9.1 地图层 | 占位层分析代码核实准确；固定 `elevation` 的决策与 OCS2 一致 |
| §9.3–9.4 样条接入 | `maxHeightSequence` 语义正确；不重复加 `swingHeight` ✅ |
| §10 候选事务和回退表 | 事务提交顺序合理；回退表覆盖完整 |
| §11 测试设计 | 除 §11.4 一条外均可执行 |
| §12–§13 Gazebo/实机 | G0.5 标定步骤是有效的风险缓解 |
| §14 文件列表 | `PerceptiveLeggedPrecomputation` 已正确归入"预计不修改" |
| §15 实施顺序 | R0 不阻塞离线步骤 1–7，合理 |
| §16–§17 成功标准和检查清单 | 与正文逐项对应，可检验 |

## 8. 建议修改汇总

| # | 位置 | 性质 | 动作 |
|---|---|---|---|
| 1 | §9.3 开头 | 实质性遗漏 | 补充 `h_lift`/`h_touch` 与采样端点必须与 `getHeights()` 同源 |
| 2 | §8.1 或 §10.1 | 实现细节遗漏 | 补充冻结恢复需覆盖该事件的全部 phase 下标 |
| 3 | §8.1 快照字段 | 冗余简化 | 删除 `planeTransformToWorld`；注明 `projectionCost` 用途 |
| 4 | §9.2 验证命令 | 鲁棒性 | 简化为两步命令，去掉脆弱的 `--filter` 表达式 |
| 5 | §11.4 | 可测性 | 把"阻止悬空引用"改为 `weak_ptr` 析构顺序断言 |
