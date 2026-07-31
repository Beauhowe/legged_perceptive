#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <cstddef>
#include <string>
#include <vector>

#include "legged_perceptive_interface/PerceptiveFootholdPlanningSettings.h"

namespace legged {

/**
 * @brief 由 URDF 中腿根相对 base 的静态横向位置派生出的机身侧别。
 *
 * K20 base 坐标系中 y 为正表示左侧，y 为负表示右侧。侧别只允许由 Pinocchio
 * 模型中的静态几何关系生成，禁止根据 contact 数组下标或 contact 名称前缀猜测。
 */
enum class LegSide { Left, Right };

/**
 * @brief 一个运行时 3-DoF contact 对应的已解析 HAA 腿根信息。
 *
 * 结果数组始终与传入的运行时 contactNames3DoF 顺序一致，配置文件中的映射顺序
 * 不会影响该结构的排列。S2 只保存映射和侧别，尚不构造运动学惩罚使用的规划髋
 * 坐标系。
 */
struct ContactLegRoot {
  /** @brief 运行时 3-DoF 足端 contact 名称。 */
  std::string contactName;

  /** @brief 配置显式指定的 HAA 腿根关节名称。 */
  std::string jointName;

  /** @brief HAA 在 Pinocchio model.joints 中的索引。 */
  std::size_t jointId;

  /** @brief HAA 原点相对 base 的静态 y 坐标，单位 m。 */
  ocs2::scalar_t lateralPositionInBase;

  /** @brief 根据 lateralPositionInBase 的符号派生出的左右侧别。 */
  LegSide side;
};

/**
 * @brief 解析并校验 contact 到 HAA 的映射，同时从 URDF 静态位置派生左右侧别。
 *
 * 当 enableKinematicPenalty 为 false 时直接返回空数组，不读取或校验映射，以保证
 * 旧配置和关闭态继续保持原行为。开关开启时要求：
 *
 * - 每个运行时 3-DoF contact 恰好出现一次，且配置不得包含未知 contact；
 * - 每个 contact 映射到不同、存在且为 1-DoF 的 Pinocchio joint；
 * - 映射 joint 必须是 floating-base 的直接子关节，确保其位姿相对 base 固定；
 * - joint 相对 base 的静态 y 必须有限，且绝对值大于内部小阈值。
 *
 * @param settings 感知落脚配置，包含功能开关和原始 contact→joint 映射。
 * @param contactNames3DoF 运行时 ModelSettings 提供的 3-DoF contact 顺序。
 * @param pinocchioInterface 已从当前机器人 URDF 建立的 Pinocchio 模型。
 * @return 与 contactNames3DoF 同序的已解析腿根信息；功能关闭时为空。
 *
 * @throws std::invalid_argument 功能开启且映射、joint 或静态横向位置不合法时抛出。
 */
std::vector<ContactLegRoot> resolveContactLegRoots(const PerceptiveFootholdPlanningSettings& settings,
                                                   const std::vector<std::string>& contactNames3DoF,
                                                   const ocs2::PinocchioInterface& pinocchioInterface);

}  // namespace legged
