#pragma once

#include <ocs2_core/Types.h>

#include <string>
#include <utility>
#include <vector>

namespace legged {

/**
 * @brief 感知落脚规划各可选功能的统一配置。
 *
 * 该结构体只描述配置契约，不直接执行落脚点修正、运动学代价计算、晚摆动相冻结或
 * 地形净空调整。调用方必须同时检查对应的 enable* 开关和所需运行时数据，才能让
 * 某项功能生效。
 *
 * 四个功能开关默认关闭，因此旧配置文件缺失整个配置组或缺失单个字段时，系统仍
 * 保持原有行为。所有成员的默认值也作为 loadPerceptiveFootholdPlanningSettings()
 * 的逐字段回退值。
 */
struct PerceptiveFootholdPlanningSettings {
  /**
   * @brief 是否启用 Raibert 水平落脚反馈。
   *
   * 启用后，预期只修正当前摆动腿首次规划触地点的世界系 x/y 坐标，并使用世界系
   * CoM 速度代理计算反馈；关闭时不得改变现有落脚点选择结果。
   */
  bool enableRaibertFeedback = false;

  /**
   * @brief 是否在凸区域候选排序中加入腿运动学惩罚。
   *
   * 启用后，预期对候选落脚点在摆动起点和终点的髋足伸展超限及向内跨步进行非负
   * 惩罚；关闭时保留现有的纯距离排序。
   */
  bool enableKinematicPenalty = false;

  /**
   * @brief 是否在摆动后半程冻结最后一个有效落脚点。
   *
   * 启用后，预期当归一化摆动相位达到 freezePhase 时停止更新该腿的目标落脚点；
   * 关闭时继续沿用每个规划周期均可更新目标的行为。
   */
  bool enableLateSwingFreeze = false;

  /**
   * @brief 是否根据 elevation 路径剖面自适应增加摆动净空。
   *
   * 启用后，预期在既有固定摆高基础上加入有上限的地形附加高度；关闭或所需地图
   * 数据不可用时，应回退到既有固定摆高。
   */
  bool enableTerrainClearance = false;

  /**
   * @brief Raibert 倒立摆模型采用的等效质心高度，单位 m。
   *
   * 后续反馈计算可用其构造类似 sqrt(height / gravity) 的时间尺度。必须为有限
   * 非负数；默认值 0.55 m 是 K20 的初始标定值。
   */
  ocs2::scalar_t invertedPendulumHeight = 0.55;

  /**
   * @brief Raibert 水平修正量的最大范数，单位 m。
   *
   * 用于限制一次反馈对目标落脚点 x/y 的总偏移，避免速度误差产生过大的落脚突变。
   * 必须为有限非负数。
   */
  ocs2::scalar_t raibertMaxOffset = 0.08;

  /**
   * @brief 上一周期落脚点在历史平滑中的权重，无量纲。
   *
   * 取值范围为 [0, 1]；越接近 1 越偏向保持上一周期目标，越接近 0 越偏向本周期
   * 新解。
   */
  ocs2::scalar_t previousFootholdFactor = 0.70;

  /**
   * @brief 上一周期目标复用的水平死区，单位 m。
   *
   * 当新旧落脚点的 x/y 差异落在该死区内时，后续实现可保留旧目标以抑制微小抖动。
   * 必须为有限非负数。
   */
  ocs2::scalar_t previousFootholdDeadzone = 0.01;

  /**
   * @brief 运动学惩罚使用的名义腿伸展长度，单位 m。
   *
   * 候选髋足距离超过该值时，后续实现可对超出部分施加惩罚。0.62 m 只是 K20 的
   * 保守初值，并非由 URDF 得到的几何极限；应先在 Gazebo 中结合正常步幅标定。
   * 必须为有限非负数。
   */
  ocs2::scalar_t nominalLegExtension = 0.62;

  /**
   * @brief 运动学惩罚相对于原候选代价的权重。
   *
   * 用于缩放髋足伸展超限和向内跨步等平方误差项，具体量纲随代价归一化方式确定。
   * 必须为有限非负数；设为 0 等价于不增加该惩罚项。
   */
  ocs2::scalar_t kinematicPenaltyWeight = 5.0;

  /**
   * @brief 开始冻结落脚目标的归一化摆动相位。
   *
   * 取值范围为 [0, 1]，其中 0 表示摆动开始、1 表示预计触地。默认 0.50 表示从
   * 摆动中点起进入冻结窗口。
   */
  ocs2::scalar_t freezePhase = 0.50;

  /**
   * @brief 事件与预计触地时刻的最大匹配误差，单位 s。
   *
   * 后续冻结逻辑可用它比较新旧模式时序中的起飞和触地边界，判断二者是否属于
   * 同一摆动事件；完全失配时应走安全回退路径。必须为有限非负数。
   */
  ocs2::scalar_t contactTimeMatchTolerance = 0.03;

  /**
   * @brief 摆动足越过路径最高地形点时保留的额外垂向裕量，单位 m。
   *
   * 该值只用于“地形高于基准摆动轨迹”的附加净空，不替代现有 swingHeight。
   * elevation 数据可能已包含地图膨胀效果，因此需要在仿真和实机上联合标定。
   * 必须为有限非负数。
   */
  ocs2::scalar_t terrainClearanceMargin = 0.03;

  /**
   * @brief 地形自适应部分允许增加的最大摆高，单位 m。
   *
   * 只限制地形附加高度，不包含现有固定 swingHeight；用于避免单个异常地图栅格
   * 生成过高摆动轨迹。必须为有限非负数。
   */
  ocs2::scalar_t maxTerrainAdaptation = 0.12;

  /**
   * @brief 足端 contact 名称到 HAA 腿根关节名称的显式映射。
   *
   * 加载时保留配置文件中的原始顺序和重复项，便于后续严格检测重复 contact；真正
   * 使用时必须按运行时 modelSettings.contactNames3DoF 的名称查找，禁止依赖这里的
   * 顺序或腿数组下标。配置缺失时保持为空，且运动学惩罚关闭时不会触发映射校验。
   */
  std::vector<std::pair<std::string, std::string>> legRootJointByContact;

  /**
   * @brief 校验所有标量配置的数值域。
   *
   * 检查所有标量均为有限数，长度、时间、裕量和权重均非负，并检查
   * previousFootholdFactor 与 freezePhase 位于 [0, 1]。
   *
   * 本函数不校验 contact name 到 HAA 的映射、地图图层可用性或 K20 物理参数的
   * 标定质量，这些属于使用对应功能时的运行时或集成检查。
   *
   * @throws std::invalid_argument 任一标量不满足约束时抛出，异常信息包含配置项名称。
   */
  void validate() const;
};

/**
 * @brief 从 Boost INFO 配置文件加载感知落脚规划参数。
 *
 * 加载器先用结构体默认值初始化结果，再从 fieldName 配置组逐字段覆盖；因此整个
 * 配置组或任一字段缺失都不会报错。完成读取后会自动调用 validate()。该函数只
 * 负责加载和校验配置，不会把任何可选算法接入规划流程。
 *
 * @param fileName Boost INFO 配置文件路径。
 * @param fieldName 配置组名称，默认是 "perceptive_foothold_planning"。
 * @param verbose 为 true 时向标准错误输出四个功能开关的最终状态。
 * @return 已应用默认值、配置覆盖并通过校验的设置。
 *
 * @throws boost::property_tree::info_parser_error 文件无法读取或 INFO 语法无效时抛出。
 * @throws boost::property_tree::ptree_bad_data 配置值无法转换为目标类型时抛出。
 * @throws std::invalid_argument 标量配置未通过 validate() 时抛出。
 */
PerceptiveFootholdPlanningSettings loadPerceptiveFootholdPlanningSettings(
    const std::string& fileName, const std::string& fieldName = "perceptive_foothold_planning", bool verbose = false);

}  // namespace legged
