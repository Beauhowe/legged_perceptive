#include <gtest/gtest.h>

#include <pinocchio/multibody/joint/joint-free-flyer.hpp>
#include <pinocchio/multibody/joint/joint-revolute.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/se3.hpp>

#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "legged_perceptive_interface/ContactLegRootResolver.h"
#include "legged_perceptive_interface/PerceptiveFootholdPlanningSettings.h"

namespace {

using ContactJointPairs = std::vector<std::pair<std::string, std::string>>;

pinocchio::Model createQuadrupedModel(double lfY = 0.08) {
  pinocchio::Model model;
  const auto baseJoint =
      model.addJoint(0, pinocchio::JointModelFreeFlyer(), pinocchio::SE3::Identity(), "root_joint");

  const auto addLegRoot = [&](const std::string& name, double x, double y) {
    return model.addJoint(baseJoint, pinocchio::JointModelRX(),
                          pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, 0.0)), name);
  };

  const auto lfJoint = addLegRoot("LF_HAA", 0.30, lfY);
  addLegRoot("RF_HAA", 0.30, -0.08);
  addLegRoot("LH_HAA", -0.30, 0.08);
  addLegRoot("RH_HAA", -0.30, -0.08);

  // HFE 是 HAA 的运动子关节，不能被误当成相对 base 固定的腿根。
  model.addJoint(lfJoint, pinocchio::JointModelRY(),
                 pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.09, 0.05, 0.0)), "LF_HFE");
  return model;
}

ContactJointPairs defaultMappings() {
  return {{"LF_FOOT", "LF_HAA"}, {"RF_FOOT", "RF_HAA"}, {"LH_FOOT", "LH_HAA"}, {"RH_FOOT", "RH_HAA"}};
}

legged::PerceptiveFootholdPlanningSettings enabledSettings(ContactJointPairs mappings = defaultMappings()) {
  legged::PerceptiveFootholdPlanningSettings settings;
  settings.enableKinematicPenalty = true;
  settings.legRootJointByContact = std::move(mappings);
  return settings;
}

TEST(ContactLegRootResolver, DisabledKinematicPenaltySkipsMappingValidation) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel());
  const legged::PerceptiveFootholdPlanningSettings settings;

  EXPECT_TRUE(legged::resolveContactLegRoots(settings, {"LF_FOOT"}, pinocchioInterface).empty());
}

TEST(ContactLegRootResolver, ResolvesDefaultK20MappingAndSides) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel());
  const auto settings = enabledSettings();
  const std::vector<std::string> contacts{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

  const auto resolved = legged::resolveContactLegRoots(settings, contacts, pinocchioInterface);

  ASSERT_EQ(resolved.size(), 4);
  EXPECT_EQ(resolved[0].jointName, "LF_HAA");
  EXPECT_EQ(resolved[0].side, legged::LegSide::Left);
  EXPECT_EQ(resolved[1].jointName, "RF_HAA");
  EXPECT_EQ(resolved[1].side, legged::LegSide::Right);
  EXPECT_EQ(resolved[2].jointName, "LH_HAA");
  EXPECT_EQ(resolved[2].side, legged::LegSide::Left);
  EXPECT_EQ(resolved[3].jointName, "RH_HAA");
  EXPECT_EQ(resolved[3].side, legged::LegSide::Right);
}

TEST(ContactLegRootResolver, FollowsRuntimeContactOrderInsteadOfConfigurationOrder) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel());
  const auto settings = enabledSettings();
  const std::vector<std::string> shuffledContacts{"RH_FOOT", "LF_FOOT", "RF_FOOT", "LH_FOOT"};

  const auto resolved = legged::resolveContactLegRoots(settings, shuffledContacts, pinocchioInterface);

  ASSERT_EQ(resolved.size(), shuffledContacts.size());
  for (size_t i = 0; i < shuffledContacts.size(); ++i) {
    EXPECT_EQ(resolved[i].contactName, shuffledContacts[i]);
  }
  EXPECT_EQ(resolved[0].jointName, "RH_HAA");
  EXPECT_EQ(resolved[1].jointName, "LF_HAA");
  EXPECT_EQ(resolved[2].jointName, "RF_HAA");
  EXPECT_EQ(resolved[3].jointName, "LH_HAA");
}

TEST(ContactLegRootResolver, RejectsMissingAndUnknownContacts) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel());
  const std::vector<std::string> contacts{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

  auto missing = defaultMappings();
  missing.pop_back();
  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(missing), contacts, pinocchioInterface),
               std::invalid_argument);

  auto unknown = defaultMappings();
  unknown.emplace_back("EXTRA_FOOT", "LF_HAA");
  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(unknown), contacts, pinocchioInterface),
               std::invalid_argument);
}

TEST(ContactLegRootResolver, RejectsDuplicateContactAndJointMappings) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel());
  const std::vector<std::string> contacts{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

  auto duplicateContact = defaultMappings();
  duplicateContact.emplace_back("LF_FOOT", "LF_HAA");
  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(duplicateContact), contacts, pinocchioInterface),
               std::invalid_argument);

  auto duplicateJoint = defaultMappings();
  duplicateJoint[1].second = "LF_HAA";
  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(duplicateJoint), contacts, pinocchioInterface),
               std::invalid_argument);
}

TEST(ContactLegRootResolver, RejectsMissingOrNonRootJoint) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel());
  const std::vector<std::string> contacts{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

  auto missingJoint = defaultMappings();
  missingJoint[0].second = "DOES_NOT_EXIST";
  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(missingJoint), contacts, pinocchioInterface),
               std::invalid_argument);

  auto nonRootJoint = defaultMappings();
  nonRootJoint[0].second = "LF_HFE";
  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(nonRootJoint), contacts, pinocchioInterface),
               std::invalid_argument);
}

TEST(ContactLegRootResolver, RejectsNearZeroStaticLateralPosition) {
  const ocs2::PinocchioInterface pinocchioInterface(createQuadrupedModel(1e-9));
  const std::vector<std::string> contacts{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

  EXPECT_THROW(legged::resolveContactLegRoots(enabledSettings(), contacts, pinocchioInterface), std::invalid_argument);
}

}  // namespace
