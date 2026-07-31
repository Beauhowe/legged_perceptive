#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>

#include "legged_perceptive_interface/ContactLegRootResolver.h"

#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace legged {
namespace {

/** 小于该横向距离时无法可靠地区分左右腿，单位 m。 */
constexpr ocs2::scalar_t kMinimumLateralPosition = 1e-6;

[[noreturn]] void throwMappingError(const std::string& message) {
  throw std::invalid_argument("perceptive_foothold_planning.legRootJointByContact: " + message);
}

}  // namespace

std::vector<ContactLegRoot> resolveContactLegRoots(const PerceptiveFootholdPlanningSettings& settings,
                                                   const std::vector<std::string>& contactNames3DoF,
                                                   const ocs2::PinocchioInterface& pinocchioInterface) {
  // 关闭态不校验可选映射，保证缺配置的旧 task.info 仍保持原行为。
  if (!settings.enableKinematicPenalty) {
    return {};
  }

  std::unordered_set<std::string> runtimeContacts;
  runtimeContacts.reserve(contactNames3DoF.size());
  for (const auto& contactName : contactNames3DoF) {
    if (!runtimeContacts.insert(contactName).second) {
      throwMappingError("duplicate runtime contact '" + contactName + "'");
    }
  }

  std::unordered_map<std::string, std::string> jointByContact;
  jointByContact.reserve(settings.legRootJointByContact.size());
  for (const auto& [contactName, jointName] : settings.legRootJointByContact) {
    if (runtimeContacts.find(contactName) == runtimeContacts.end()) {
      throwMappingError("unknown contact '" + contactName + "'");
    }
    if (!jointByContact.emplace(contactName, jointName).second) {
      throwMappingError("duplicate contact '" + contactName + "'");
    }
  }

  const auto& model = pinocchioInterface.getModel();
  std::unordered_set<std::string> usedJointNames;
  usedJointNames.reserve(contactNames3DoF.size());

  std::vector<ContactLegRoot> resolved;
  resolved.reserve(contactNames3DoF.size());
  for (const auto& contactName : contactNames3DoF) {
    const auto mapping = jointByContact.find(contactName);
    if (mapping == jointByContact.end()) {
      throwMappingError("missing contact '" + contactName + "'");
    }

    const std::string& jointName = mapping->second;
    if (!usedJointNames.insert(jointName).second) {
      throwMappingError("joint '" + jointName + "' is mapped more than once");
    }
    if (!model.existJointName(jointName)) {
      throwMappingError("joint '" + jointName + "' does not exist");
    }

    const auto jointId = model.getJointId(jointName);
    if (jointId == 0 || jointId >= model.njoints || model.nqs[jointId] != 1 || model.nvs[jointId] != 1) {
      throwMappingError("joint '" + jointName + "' must be a valid 1-DoF joint");
    }

    // HAA 必须直接挂在 floating base 下；否则 jointPlacements 中的位移不是相对 base
    // 固定的腿根位置，会随上游关节运动而失去左右判定意义。
    const auto baseJointId = model.parents[jointId];
    if (baseJointId == 0 || model.parents[baseJointId] != 0 || model.nqs[baseJointId] != 7 ||
        model.nvs[baseJointId] != 6) {
      throwMappingError("joint '" + jointName + "' is not a direct child of the floating base");
    }

    const ocs2::scalar_t lateralPosition = model.jointPlacements[jointId].translation().y();
    if (!std::isfinite(lateralPosition) || std::abs(lateralPosition) <= kMinimumLateralPosition) {
      throwMappingError("joint '" + jointName + "' has an invalid static lateral position");
    }

    resolved.push_back({contactName, jointName, static_cast<std::size_t>(jointId), lateralPosition,
                        lateralPosition > 0.0 ? LegSide::Left : LegSide::Right});
  }

  return resolved;
}

}  // namespace legged
