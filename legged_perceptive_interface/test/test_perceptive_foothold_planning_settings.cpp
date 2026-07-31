#include <gtest/gtest.h>

#include <boost/filesystem.hpp>

#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "legged_perceptive_interface/PerceptiveFootholdPlanningSettings.h"

namespace {

class TemporaryInfoFile {
 public:
  explicit TemporaryInfoFile(const std::string& contents)
      : path_(boost::filesystem::temp_directory_path() /
              boost::filesystem::unique_path("perceptive_foothold_settings-%%%%-%%%%.info")) {
    std::ofstream stream(path_.string());
    stream << contents;
    stream.close();
  }

  ~TemporaryInfoFile() {
    boost::system::error_code error;
    boost::filesystem::remove(path_, error);
  }

  const std::string string() const { return path_.string(); }

 private:
  boost::filesystem::path path_;
};

void expectDisabledDefaults(const legged::PerceptiveFootholdPlanningSettings& settings) {
  EXPECT_FALSE(settings.enableRaibertFeedback);
  EXPECT_FALSE(settings.enableKinematicPenalty);
  EXPECT_FALSE(settings.enableLateSwingFreeze);
  EXPECT_FALSE(settings.enableTerrainClearance);
  EXPECT_DOUBLE_EQ(settings.invertedPendulumHeight, 0.55);
  EXPECT_DOUBLE_EQ(settings.raibertMaxOffset, 0.08);
  EXPECT_DOUBLE_EQ(settings.previousFootholdFactor, 0.70);
  EXPECT_DOUBLE_EQ(settings.previousFootholdDeadzone, 0.01);
  EXPECT_DOUBLE_EQ(settings.nominalLegExtension, 0.62);
  EXPECT_DOUBLE_EQ(settings.kinematicPenaltyWeight, 5.0);
  EXPECT_DOUBLE_EQ(settings.freezePhase, 0.50);
  EXPECT_DOUBLE_EQ(settings.contactTimeMatchTolerance, 0.03);
  EXPECT_DOUBLE_EQ(settings.terrainClearanceMargin, 0.03);
  EXPECT_DOUBLE_EQ(settings.maxTerrainAdaptation, 0.12);
  EXPECT_TRUE(settings.legRootJointByContact.empty());
}

TEST(PerceptiveFootholdPlanningSettings, MissingGroupUsesDisabledDefaults) {
  const TemporaryInfoFile file("unrelated\n{\n  value 1\n}\n");

  const auto settings = legged::loadPerceptiveFootholdPlanningSettings(file.string());

  expectDisabledDefaults(settings);
}

TEST(PerceptiveFootholdPlanningSettings, ExplicitlyDisabledGroupMatchesDefaults) {
  const TemporaryInfoFile file(R"(
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
}
)");

  const auto settings = legged::loadPerceptiveFootholdPlanningSettings(file.string());

  expectDisabledDefaults(settings);
}

TEST(PerceptiveFootholdPlanningSettings, LoadsExplicitValuesWithoutUsingThem) {
  const TemporaryInfoFile file(R"(
perceptive_foothold_planning
{
  enableRaibertFeedback       true
  enableKinematicPenalty      true
  enableLateSwingFreeze       true
  enableTerrainClearance      true
  invertedPendulumHeight      0.60
  raibertMaxOffset            0.09
  previousFootholdFactor      0.25
  previousFootholdDeadzone    0.02
  nominalLegExtension         0.64
  kinematicPenaltyWeight      4.0
  freezePhase                 0.75
  contactTimeMatchTolerance   0.04
  terrainClearanceMargin      0.05
  maxTerrainAdaptation        0.10
  legRootJointByContact
  {
    LF_FOOT LF_HAA
    RF_FOOT RF_HAA
    LH_FOOT LH_HAA
    RH_FOOT RH_HAA
  }
}
)");

  const auto settings = legged::loadPerceptiveFootholdPlanningSettings(file.string());

  EXPECT_TRUE(settings.enableRaibertFeedback);
  EXPECT_TRUE(settings.enableKinematicPenalty);
  EXPECT_TRUE(settings.enableLateSwingFreeze);
  EXPECT_TRUE(settings.enableTerrainClearance);
  EXPECT_DOUBLE_EQ(settings.invertedPendulumHeight, 0.60);
  EXPECT_DOUBLE_EQ(settings.raibertMaxOffset, 0.09);
  EXPECT_DOUBLE_EQ(settings.previousFootholdFactor, 0.25);
  EXPECT_DOUBLE_EQ(settings.previousFootholdDeadzone, 0.02);
  EXPECT_DOUBLE_EQ(settings.nominalLegExtension, 0.64);
  EXPECT_DOUBLE_EQ(settings.kinematicPenaltyWeight, 4.0);
  EXPECT_DOUBLE_EQ(settings.freezePhase, 0.75);
  EXPECT_DOUBLE_EQ(settings.contactTimeMatchTolerance, 0.04);
  EXPECT_DOUBLE_EQ(settings.terrainClearanceMargin, 0.05);
  EXPECT_DOUBLE_EQ(settings.maxTerrainAdaptation, 0.10);
  ASSERT_EQ(settings.legRootJointByContact.size(), 4);
  EXPECT_EQ(settings.legRootJointByContact[0], std::make_pair(std::string("LF_FOOT"), std::string("LF_HAA")));
  EXPECT_EQ(settings.legRootJointByContact[1], std::make_pair(std::string("RF_FOOT"), std::string("RF_HAA")));
  EXPECT_EQ(settings.legRootJointByContact[2], std::make_pair(std::string("LH_FOOT"), std::string("LH_HAA")));
  EXPECT_EQ(settings.legRootJointByContact[3], std::make_pair(std::string("RH_FOOT"), std::string("RH_HAA")));
}

TEST(PerceptiveFootholdPlanningSettings, RejectsNonFiniteAndOutOfRangeScalars) {
  using Settings = legged::PerceptiveFootholdPlanningSettings;
  using ScalarMember = ocs2::scalar_t Settings::*;
  struct ScalarCase {
    const char* name;
    ScalarMember member;
    bool unitInterval;
  };

  const std::vector<ScalarCase> scalarCases{
      {"invertedPendulumHeight", &Settings::invertedPendulumHeight, false},
      {"raibertMaxOffset", &Settings::raibertMaxOffset, false},
      {"previousFootholdFactor", &Settings::previousFootholdFactor, true},
      {"previousFootholdDeadzone", &Settings::previousFootholdDeadzone, false},
      {"nominalLegExtension", &Settings::nominalLegExtension, false},
      {"kinematicPenaltyWeight", &Settings::kinematicPenaltyWeight, false},
      {"freezePhase", &Settings::freezePhase, true},
      {"contactTimeMatchTolerance", &Settings::contactTimeMatchTolerance, false},
      {"terrainClearanceMargin", &Settings::terrainClearanceMargin, false},
      {"maxTerrainAdaptation", &Settings::maxTerrainAdaptation, false},
  };

  for (const auto& scalarCase : scalarCases) {
    SCOPED_TRACE(scalarCase.name);

    Settings notANumber;
    notANumber.*(scalarCase.member) = std::numeric_limits<ocs2::scalar_t>::quiet_NaN();
    EXPECT_THROW(notANumber.validate(), std::invalid_argument);

    Settings infinite;
    infinite.*(scalarCase.member) = std::numeric_limits<ocs2::scalar_t>::infinity();
    EXPECT_THROW(infinite.validate(), std::invalid_argument);

    Settings negative;
    negative.*(scalarCase.member) = -0.01;
    EXPECT_THROW(negative.validate(), std::invalid_argument);

    if (scalarCase.unitInterval) {
      Settings aboveOne;
      aboveOne.*(scalarCase.member) = 1.01;
      EXPECT_THROW(aboveOne.validate(), std::invalid_argument);
    }
  }
}

TEST(PerceptiveFootholdPlanningSettings, LoaderValidatesParsedValues) {
  const TemporaryInfoFile file(R"(
perceptive_foothold_planning
{
  freezePhase 1.01
}
)");

  EXPECT_THROW(legged::loadPerceptiveFootholdPlanningSettings(file.string()), std::invalid_argument);
}

}  // namespace
