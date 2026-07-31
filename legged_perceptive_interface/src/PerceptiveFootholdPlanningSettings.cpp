#include "legged_perceptive_interface/PerceptiveFootholdPlanningSettings.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace legged {
namespace {

void validateNonNegative(const char* name, ocs2::scalar_t value) {
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(std::string("perceptive_foothold_planning.") + name +
                                " must be finite and non-negative");
  }
}

void validateUnitInterval(const char* name, ocs2::scalar_t value) {
  if (!std::isfinite(value) || value < 0.0 || value > 1.0) {
    throw std::invalid_argument(std::string("perceptive_foothold_planning.") + name + " must be within [0, 1]");
  }
}

}  // namespace

void PerceptiveFootholdPlanningSettings::validate() const {
  validateNonNegative("invertedPendulumHeight", invertedPendulumHeight);
  validateNonNegative("raibertMaxOffset", raibertMaxOffset);
  validateUnitInterval("previousFootholdFactor", previousFootholdFactor);
  validateNonNegative("previousFootholdDeadzone", previousFootholdDeadzone);
  validateNonNegative("nominalLegExtension", nominalLegExtension);
  validateNonNegative("kinematicPenaltyWeight", kinematicPenaltyWeight);
  validateUnitInterval("freezePhase", freezePhase);
  validateNonNegative("contactTimeMatchTolerance", contactTimeMatchTolerance);
  validateNonNegative("terrainClearanceMargin", terrainClearanceMargin);
  validateNonNegative("maxTerrainAdaptation", maxTerrainAdaptation);
}

PerceptiveFootholdPlanningSettings loadPerceptiveFootholdPlanningSettings(const std::string& fileName,
                                                                          const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree tree;
  boost::property_tree::read_info(fileName, tree);

  PerceptiveFootholdPlanningSettings settings;
  const std::string prefix = fieldName + ".";

  settings.enableRaibertFeedback = tree.get<bool>(prefix + "enableRaibertFeedback", settings.enableRaibertFeedback);
  settings.enableKinematicPenalty = tree.get<bool>(prefix + "enableKinematicPenalty", settings.enableKinematicPenalty);
  settings.enableLateSwingFreeze = tree.get<bool>(prefix + "enableLateSwingFreeze", settings.enableLateSwingFreeze);
  settings.enableTerrainClearance = tree.get<bool>(prefix + "enableTerrainClearance", settings.enableTerrainClearance);

  settings.invertedPendulumHeight =
      tree.get<ocs2::scalar_t>(prefix + "invertedPendulumHeight", settings.invertedPendulumHeight);
  settings.raibertMaxOffset = tree.get<ocs2::scalar_t>(prefix + "raibertMaxOffset", settings.raibertMaxOffset);
  settings.previousFootholdFactor =
      tree.get<ocs2::scalar_t>(prefix + "previousFootholdFactor", settings.previousFootholdFactor);
  settings.previousFootholdDeadzone =
      tree.get<ocs2::scalar_t>(prefix + "previousFootholdDeadzone", settings.previousFootholdDeadzone);
  settings.nominalLegExtension =
      tree.get<ocs2::scalar_t>(prefix + "nominalLegExtension", settings.nominalLegExtension);
  settings.kinematicPenaltyWeight =
      tree.get<ocs2::scalar_t>(prefix + "kinematicPenaltyWeight", settings.kinematicPenaltyWeight);
  settings.freezePhase = tree.get<ocs2::scalar_t>(prefix + "freezePhase", settings.freezePhase);
  settings.contactTimeMatchTolerance =
      tree.get<ocs2::scalar_t>(prefix + "contactTimeMatchTolerance", settings.contactTimeMatchTolerance);
  settings.terrainClearanceMargin =
      tree.get<ocs2::scalar_t>(prefix + "terrainClearanceMargin", settings.terrainClearanceMargin);
  settings.maxTerrainAdaptation =
      tree.get<ocs2::scalar_t>(prefix + "maxTerrainAdaptation", settings.maxTerrainAdaptation);

  if (const auto mappings = tree.get_child_optional(prefix + "legRootJointByContact")) {
    for (const auto& mapping : *mappings) {
      // 使用 vector 保留重复 contact，交由启用运动学功能时的 resolver 明确拒绝。
      settings.legRootJointByContact.emplace_back(mapping.first, mapping.second.get_value<std::string>());
    }
  }

  settings.validate();

  if (verbose) {
    std::cerr << "\n #### Perceptive Foothold Planning Settings:"
              << " raibert=" << settings.enableRaibertFeedback
              << " kinematic=" << settings.enableKinematicPenalty
              << " freeze=" << settings.enableLateSwingFreeze
              << " terrain_clearance=" << settings.enableTerrainClearance << std::endl;
  }

  return settings;
}

}  // namespace legged
