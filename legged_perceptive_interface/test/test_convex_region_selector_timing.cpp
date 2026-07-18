#include <gtest/gtest.h>

#include <cmath>
#include <mutex>
#include <vector>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

// Access the existing private helper without adding a test-only production API.
#define private public
#include "legged_perceptive_interface/ConvexRegionSelector.h"
#undef private

namespace legged {
namespace {

TEST(ConvexRegionSelectorTiming, FinalStanceIndexReferencesExistingEventTime) {
  const std::vector<bool> contactFlags{false, true, true};
  const scalar_array_t eventTimes{0.5, 1.0};

  const auto indices = ConvexRegionSelector::findIndex(2, contactFlags);

  ASSERT_GE(indices.second, 0);
  EXPECT_LT(static_cast<size_t>(indices.second), eventTimes.size())
      << "N mode phases have only N-1 event times";
}

#if defined(__SANITIZE_ADDRESS__)
TEST(ConvexRegionSelectorTiming, AddressSanitizerReproducesFinalStanceOutOfBoundsRead) {
  const std::vector<bool> contactFlags{false, true, true};
  const scalar_array_t eventTimes{0.5, 1.0};
  const auto indices = ConvexRegionSelector::findIndex(2, contactFlags);

  const volatile scalar_t finalStanceTime = eventTimes[static_cast<size_t>(indices.second)];
  (void)finalStanceTime;
}
#endif

}  // namespace
}  // namespace legged
