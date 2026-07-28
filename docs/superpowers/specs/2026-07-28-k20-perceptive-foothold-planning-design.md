# K20 感知落脚规划增强设计

日期：2026-07-28
状态：待用户审阅
目标机器人：K20
主要步态：Trot

## 1. 背景

当前 `legged_perceptive` 已能从分割平面生成凸落脚区域，并用软约束限制 MPC 足端落在区域内；摆动足也有基于 SDF 的碰撞约束。但与论文《Perceptive Locomotion through Nonlinear Model Predictive Control》及仓库内 `ocs2_perceptive_anymal` 示例相比，当前落脚参考生成存在四项关键简化：

1. 已计算 Raibert 速度反馈，但实际返回的名义落脚点没有使用该反馈。
2. 候选平面的附加评分恒为零，没有腿过伸和向身体内侧跨步惩罚。
3. 摆动后半程没有明确冻结本次落脚平面和凸区域。
4. 摆动中点高度只取起落脚高度最大值加固定摆高，没有查询起落脚路径上的最高地形。

本设计在不改变现有 centroidal MPC 状态、输入、求解器、WBC 和状态估计的前提下，将 OCS2 示例中的对应算法适配到 K20 感知参考规划层。

## 2. 目标

- 对每条腿的第一个未来落脚点应用带限幅和滤波的 Raibert 速度反馈。
- 使用 K20 髋部坐标系评价候选落脚点的腿过伸与向内跨步风险。
- 在摆动后半程保持已选落脚平面、凸区域和触地点高度不变。
- 根据起落脚路径上的地形高度自动增加摆动足净空。
- 四项功能使用独立配置开关，便于 A/B 对比和实机回退。
- 使用同一套逻辑自动处理平地、缓坡和低台阶，不引入地形模式切换。
- 通过单元测试、Gazebo 和 K20 实机低速测试逐级验收。

## 3. 非目标

- 不引入论文的 48 维 loop-shaping 状态。
- 不修改 MPC 动力学、输入维度、SQP/HPIPM 求解器、WBC 或状态估计。
- 不在本阶段加入 MPC 内关节位置、速度或力矩约束。
- 不移植完整 `ocs2_switched_model_interface`，也不替换当前 ReferenceManager。
- 不在本阶段把现有三次摆动样条替换为完整五次样条体系。
- 不改变其他机器人配置的默认行为。

## 4. 方案选择

采用“在当前规划器中适配 OCS2 算法”的方案。

不直接依赖 `ocs2_switched_model_interface`，因为它的状态布局、运动学接口、TerrainModel 和当前 ROS2 工程不同。直接复用会引入大量适配器，并形成两套足端规划数据结构。整体替换 ReferenceManager 的改动范围更大，超出本阶段目标。

## 5. 总体架构

```text
当前状态 + 目标轨迹 + Trot 接触时序
                    |
                    v
          Raibert 第一落脚点修正
                    |
                    v
      候选平面距离 + K20 运动学惩罚
                    |
                    v
             生成凸落脚区域
                    |
                    v
       按摆动进度更新或冻结落脚决策
                    |
                    v
       查询起落脚路径上的最高地形
                    |
                    v
         生成地形自适应摆动高度
                    |
                    v
          现有 MPC 足端软约束
```

职责划分：

- `ConvexRegionSelector`：Raibert 修正、运动学候选评分、历史落脚决策保存和后半程冻结。
- `PerceptiveLeggedReferenceManager`：组织接触相数据，提取起落脚高度和路径最高地形，将结果传给摆动规划器。
- `SwingTrajectoryPlanner`：继续使用现有三次样条，通过已有 `maxHeightSequence` 接口接收地形自适应中点高度。
- K20 感知配置：保存功能开关和参数；新增配置项缺失时保持旧行为。

## 6. Raibert 落脚修正

只对每条腿第一个尚未触地的未来接触相应用：

\[
\Delta p_{xy} =
\sqrt{\frac{h}{g}}
(v_{\mathrm{measured},xy}-v_{\mathrm{desired},xy})
\]

\[
p_{\mathrm{heuristic}} =
p_{\mathrm{kinematic}}+\Delta p_{xy}
\]

要求：

- 实测速度和目标速度转换到世界坐标系后计算。
- z 分量固定为零。
- 当前支撑足和第二个及以后未来接触相不应用反馈。
- 偏移量按二维模长限幅，不分别裁剪 x、y。
- 新落脚点与上一周期结果低通融合：`filtered = factor * previous + (1-factor) * new`。
- 新旧落脚点距离小于死区时直接保留上一结果。
- 输入速度非有限时，本周期偏移置零并记录诊断。

K20 初始参数：

```text
invertedPendulumHeight    0.55  # m
raibertMaxOffset          0.08  # m
previousFootholdFactor    0.70
previousFootholdDeadzone  0.01  # m
```

## 7. K20 运动学候选平面惩罚

候选落脚点在髋部坐标系中的惩罚为：

\[
J_{\mathrm{kin}} =
w_k(e_{\mathrm{extension}}^2+e_{\mathrm{inward}}^2)
\]

\[
e_{\mathrm{extension}} =
\max(0,\|p_{\mathrm{foot}}^{\mathrm{hip}}\|-l_{\max})
\]

`e_inward` 表示足端相对当前腿髋部向身体内侧越过安全方向的距离。左右腿通过各自髋部坐标系获得相反的内外方向，不硬编码世界 y 符号。

要求：

- 通过 Pinocchio 获得 K20 对应腿根位置和朝向。
- 分别在预计接触开始与接触结束时评价候选点，并将两次结果相加。
- 保留现有候选平面距离代价，运动学惩罚只作为附加评分。
- 本阶段只使用近似腿长和内跨惩罚，不在候选评分中运行完整逆运动学。
- 运动学数据不可用时，本周期附加惩罚置零，保留距离选择逻辑。

K20 初始参数：

```text
nominalLegExtension       0.62  # m
kinematicPenaltyWeight    5.0
```

K20 两段主腿长约为 `0.35 + 0.35 m`。0.62 m 是留有余量的仿真初值，实机启用前通过可达空间测试校准。

## 8. 摆动后半程冻结

每条腿跨 MPC 周期保存：

```text
接触开始时间
接触结束时间
名义落脚点
选中平面
凸多边形
触地点高度
冻结状态
```

摆动进度：

\[
s =
\frac{t-t_{\mathrm{liftoff}}}
{t_{\mathrm{touchdown}}-t_{\mathrm{liftoff}}}
\]

行为：

- `s < freezePhase`：允许更新名义点、候选平面和凸区域。
- `s >= freezePhase`：复用上一周期的平面、凸区域和触地点高度。
- 冻结不停止 MPC 状态和接触力更新，也不锁定实际足端轨迹。
- 接触时间在容差内变化时，仍匹配为同一个未来接触事件。
- 实际触地、进入新接触事件或步态切换后清除过期状态。
- 摆动后半程无法匹配新时序时，保持最后有效结果直到本次触地，避免临时跳面。

初始参数：

```text
freezePhase               0.50
contactTimeMatchTolerance 0.03  # s
```

## 9. 地形感知摆动高度

沿起落脚点在地图 XY 平面中的连线，以地图分辨率采样障碍地形层。每个采样点高出起落脚三维连接线的高度为：

\[
h_{\mathrm{obs}} =
\max_i\left[
h_{\mathrm{terrain},i}-
((1-s_i)h_{\mathrm{lift}}+s_i h_{\mathrm{touch}})
\right]
\]

为保证平地保持原有固定摆高，净空裕量只在检测到正障碍高度时加入：

\[
h_{\mathrm{clear}} =
\begin{cases}
0, & h_{\mathrm{obs}} \le 0 \\
h_{\mathrm{obs}}+h_{\mathrm{margin}}, & h_{\mathrm{obs}} > 0
\end{cases}
\]

\[
h_{\mathrm{mid}} =
\max(h_{\mathrm{lift}},h_{\mathrm{touch}})
+h_{\mathrm{swing}}
+\operatorname{clip}(h_{\mathrm{clear}},0,h_{\mathrm{adapt,max}})
\]

要求：

- 使用障碍地形层，不使用平滑机身参考层。
- 采样间距不小于地图分辨率。
- 少量 NaN 被跳过；有效采样不足时回退当前固定摆高。
- 异常尖峰通过最大增高限幅处理。
- 起落脚点位于同一地图栅格时直接使用起落脚高度最大值。
- 结果通过现有 `maxHeightSequence` 接口交给三次摆动样条。

K20 初始参数：

```text
terrainSampleResolution  0.03  # m
terrainClearanceMargin   0.03  # m
maxTerrainAdaptation     0.12  # m
```

K20 当前固定摆高为 0.15 m，检测到障碍时的初始最大中点增高约为 0.27 m。

## 10. 配置与兼容性

四项独立开关：

```text
enableRaibertFeedback
enableKinematicPenalty
enableLateSwingFreeze
enableTerrainClearance
```

开关只用于开发验证、A/B 对比和实机回退，不代表地形模式。

- 配置项缺失时全部按关闭处理，保持其他机器人和旧配置行为。
- 开发初期 K20 四项默认关闭，每项通过测试后逐项开启。
- 完成全部验收后，K20 感知配置中四项默认开启。
- 非感知控制配置不启用地形净空。

参数加载时验证所有范围、时间和权重。非有限值、负距离、`freezePhase` 不在 `[0,1]` 或滤波因子不在 `[0,1]` 时启动失败，并输出具体参数名。

## 11. 异常回退

| 异常 | 回退行为 |
|---|---|
| 实测或目标速度非有限 | 本周期 Raibert 偏移置零 |
| Raibert 偏移过大 | 限制到 `raibertMaxOffset` |
| 髋部运动学不可用 | 本周期运动学惩罚置零 |
| 没有有效候选平面 | 保持上一有效平面；无历史结果时沿用现有错误处理 |
| 摆动前半程时序改变 | 清除旧匹配并重新规划 |
| 摆动后半程时序小幅漂移 | 按时间容差匹配并保持冻结 |
| 后半程无法匹配接触事件 | 保持最后有效区域直到触地 |
| 地形存在少量 NaN | 跳过无效采样 |
| 有效地形采样不足 | 回退固定摆高 |
| 地形异常尖峰 | 限制最大增高 |

## 12. 诊断与可观测性

每条腿记录：

- 原始、限幅后和滤波后的 Raibert 偏移。
- 候选平面的距离代价与运动学代价。
- 被选平面标识、名义落脚点和凸区域。
- 摆动进度、冻结状态与冻结原因。
- 路径最高障碍、净空裕量和最终摆动高度。
- 每项功能是否进入回退路径。

高频诊断进入现有节流日志或调试发布接口，避免在 MPC 周期内持续输出普通日志。

## 13. 测试设计

### 13.1 单元测试

Raibert：

- 零速度误差、正负速度误差和第一未来接触相限定。
- 当前支撑足不变。
- 二维模长限幅、低通、死区和非有限速度回退。

运动学惩罚：

- 正常可达、腿过伸、左右腿内跨和正常外侧落脚。
- 接触开始与结束评分累加。
- 几何距离接近时选择运动学更合理的平面。

冻结：

- 冻结阈值前后行为。
- 地图更新不改变冻结区域。
- 接触时间容差匹配。
- 触地、步态切换和过期状态清理。

地形摆高：

- 平地保持原摆高。
- 上坡、下坡、中间台阶和路径外障碍。
- NaN、有效采样不足、异常尖峰和同栅格起落点。

兼容性：

- 四项开关关闭时输出与当前实现一致。
- 缺少新增配置项时采用旧行为。

### 13.2 Gazebo

G0 基线：四项关闭，记录 MPC 频率、求解时间、速度误差、落脚点变化、足端高度和安全故障。

G1 平地：

- 依次开启 Raibert、运动学惩罚、冻结和地形净空。
- Trot 从 0 加速至 0.20 m/s，再减速至 0。
- 加入沿机身前后方向的 150 N、持续 0.20 s 的 Gazebo 推扰。
- 四项全开连续运行不少于 60 s。

G2 缓坡：

- 上坡和下坡各 8°。
- Trot 速度 0.15 m/s。

G3 低台阶：

- 台阶高度 0.05 m。
- Trot 速度 0.10 m/s。

Gazebo 通过标准：

- 无摔倒、NaN、MPC 异常退出和新增安全故障。
- 摆动后半程落脚区域不变。
- 足端目标不发生跨平面跳变。
- 台阶场景参考轨迹高于检测到的障碍。
- 四项关闭时保持基线行为。
- MPC 保持实际 60 Hz，无持续超时。
- 新增参考规划耗时相对基线增幅不超过 20%。

### 13.3 K20 实机

测试配备急停、保护绳并保留单项关闭能力。

H0 静态检查：

- 地图、速度和足端数据均有限。
- 四腿内外方向与运动学惩罚正确。
- 原地抬腿时冻结与摆高诊断正确。

H1 平地：

- 初始速度 0.10 m/s，连续 Trot 20 s。
- 按 Raibert、运动学惩罚、冻结、全部开启的顺序验证。
- 全部通过后逐步提高至 0.20 m/s。

H2 缓坡：

- 初始坡度 5°。
- Trot 速度 0.10 m/s。

H3 低台阶：

- 初始台阶高度 0.03 m。
- Trot 速度不高于 0.10 m/s。
- 通过后再提高至 0.05 m。

立即终止条件：

- MPC 或 WBC 出现非有限值。
- 触发关节位置、速度或力矩安全保护。
- 摆动后半程发生跨平面切换。
- 任一足端目标发生超过 0.05 m 的单周期位置跳变。
- 连续两步拖脚或碰撞台阶。
- MPC 持续低于目标频率。
- 机身姿态或速度误差持续扩大。

## 14. 预计涉及文件

- `legged_perceptive_interface/include/legged_perceptive_interface/ConvexRegionSelector.h`
- `legged_perceptive_interface/src/ConvexRegionSelector.cpp`
- `legged_perceptive_interface/include/legged_perceptive_interface/PerceptiveLeggedReferenceManager.h`
- `legged_perceptive_interface/src/PerceptiveLeggedReferenceManager.cpp`
- `../legged_control/legged_interface/include/legged_interface/constraint/SwingTrajectoryPlanner.h`
- `../legged_control/legged_interface/src/constraint/SwingTrajectoryPlanner.cpp`
- `legged_perceptive_controllers/config/k20/task.info`
- 对应的 CMake 测试注册和新增单元测试文件

实现阶段仅触及需求直接关联的文件，不重构无关控制代码。

## 15. 实施顺序

1. 建立纯函数单元测试和配置加载测试。
2. 实现 Raibert 修正及诊断，保持开关默认关闭。
3. 实现 K20 运动学候选评分。
4. 实现跨周期接触匹配与后半程冻结。
5. 实现路径地形采样并接入 `maxHeightSequence`。
6. 验证四项关闭时的兼容性。
7. 按 G0、G1、G2、G3 完成 Gazebo 测试。
8. 按 H0、H1、H2、H3 完成 K20 实机低速验收。
9. 全部通过后将 K20 感知配置中的四项开关设为默认开启。

## 16. 成功标准

- 四项功能均可独立开启和关闭。
- 关闭时不改变当前 K20 行为。
- 全开时平地、缓坡和低台阶不需要切换规划模式。
- Raibert 修正能响应速度误差且不会导致落脚点跳变。
- 候选平面选择避免明显腿过伸和向内跨步。
- 摆动后半程不再切换落脚平面。
- 台阶路径能够自动提高摆动足净空。
- 单元测试、Gazebo 和实机低速验收达到本设计规定的门槛。
