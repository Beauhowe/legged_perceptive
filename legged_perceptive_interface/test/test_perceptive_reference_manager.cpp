#include <gtest/gtest.h>

#include <legged_perceptive_interface/TargetTrajectoryResampler.h>

namespace {

TEST(PerceptiveReferenceManager, ResamplesInputFromInputTrajectory) {
  ocs2::TargetTrajectories source;
  source.timeTrajectory = {0.0, 1.0};
  source.stateTrajectory = {ocs2::vector_t::Zero(2), ocs2::vector_t::Constant(2, 2.0)};
  source.inputTrajectory = {ocs2::vector_t::Constant(2, 10.0), ocs2::vector_t::Constant(2, 20.0)};

  const auto result = legged::resampleTargetTrajectories(source, 0.0, 1.0, 3);

  ASSERT_EQ(result.timeTrajectory.size(), 3u);
  ASSERT_EQ(result.stateTrajectory.size(), 3u);
  ASSERT_EQ(result.inputTrajectory.size(), 3u);
  EXPECT_DOUBLE_EQ(result.stateTrajectory[1](0), 1.0);
  EXPECT_DOUBLE_EQ(result.inputTrajectory[1](0), 15.0);
  EXPECT_NE(result.inputTrajectory[1](0), result.stateTrajectory[1](0));
}

}  // namespace
