#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "legged_perceptive_interface/ConvexRegionSelector.h"

namespace {

using Vector3 = ocs2::legged_robot::vector3_t;

void expectVectorNear(const Vector3& actual, const Vector3& expected, double tolerance = 1e-12) {
  EXPECT_NEAR(actual.x(), expected.x(), tolerance);
  EXPECT_NEAR(actual.y(), expected.y(), tolerance);
  EXPECT_NEAR(actual.z(), expected.z(), tolerance);
}

TEST(RaibertOffset, ZeroVelocityErrorReturnsZero) {
  const Vector3 velocity(0.4, -0.2, 0.1);

  const auto offset = legged::ConvexRegionSelector::computeRaibertOffset(velocity, velocity, 0.55, 0.08);

  expectVectorNear(offset, Vector3::Zero());
}

TEST(RaibertOffset, PositiveVelocityErrorUsesInvertedPendulumTimeScale) {
  const Vector3 measuredComVelocity(1.0, 0.6, 2.0);
  const Vector3 desiredComVelocity(0.2, 0.1, -1.0);
  constexpr double height = 0.55;

  const auto offset =
      legged::ConvexRegionSelector::computeRaibertOffset(measuredComVelocity, desiredComVelocity, height, 1.0);

  const double timeScale = std::sqrt(height / 9.81);
  expectVectorNear(offset, Vector3(0.8 * timeScale, 0.5 * timeScale, 0.0));
}

TEST(RaibertOffset, NegativeVelocityErrorPreservesDirection) {
  const Vector3 measuredComVelocity(-0.4, 0.2, 0.0);
  const Vector3 desiredComVelocity(0.1, 0.5, 0.0);
  constexpr double height = 0.55;

  const auto offset =
      legged::ConvexRegionSelector::computeRaibertOffset(measuredComVelocity, desiredComVelocity, height, 1.0);

  const double timeScale = std::sqrt(height / 9.81);
  expectVectorNear(offset, Vector3(-0.5 * timeScale, -0.3 * timeScale, 0.0));
}

TEST(RaibertOffset, ClampsByTwoDimensionalNormInsteadOfPerAxis) {
  const Vector3 measuredComVelocity(1.0, 1.0, 0.0);
  constexpr double maxOffset = 0.1;

  // height=9.81 使时间尺度为 1；对角偏移应沿原方向缩放到半径 0.1 的圆周。
  const auto offset = legged::ConvexRegionSelector::computeRaibertOffset(
      measuredComVelocity, Vector3::Zero(), 9.81, maxOffset);

  const double expectedComponent = maxOffset / std::sqrt(2.0);
  expectVectorNear(offset, Vector3(expectedComponent, expectedComponent, 0.0));
  EXPECT_NEAR(offset.head<2>().norm(), maxOffset, 1e-12);
}

TEST(RaibertOffset, VerticalVelocityNeverCreatesVerticalOffset) {
  const Vector3 measuredComVelocity(0.2, -0.1, 100.0);
  const Vector3 desiredComVelocity(0.0, 0.0, -100.0);

  const auto offset =
      legged::ConvexRegionSelector::computeRaibertOffset(measuredComVelocity, desiredComVelocity, 0.55, 1.0);

  EXPECT_DOUBLE_EQ(offset.z(), 0.0);
}

TEST(RaibertOffset, NonFiniteVelocityReturnsZero) {
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()}) {
    for (int axis = 0; axis < 3; ++axis) {
      Vector3 measuredComVelocity = Vector3::Zero();
      measuredComVelocity(axis) = invalid;
      expectVectorNear(legged::ConvexRegionSelector::computeRaibertOffset(
                           measuredComVelocity, Vector3::Zero(), 0.55, 0.08),
                       Vector3::Zero());

      Vector3 desiredComVelocity = Vector3::Zero();
      desiredComVelocity(axis) = invalid;
      expectVectorNear(legged::ConvexRegionSelector::computeRaibertOffset(
                           Vector3::Zero(), desiredComVelocity, 0.55, 0.08),
                       Vector3::Zero());
    }
  }
}

TEST(RaibertOffset, InvalidParametersReturnZero) {
  const Vector3 measuredComVelocity(1.0, 1.0, 0.0);
  const Vector3 zero = Vector3::Zero();

  expectVectorNear(legged::ConvexRegionSelector::computeRaibertOffset(measuredComVelocity, zero, -0.1, 0.08), zero);
  expectVectorNear(legged::ConvexRegionSelector::computeRaibertOffset(
                       measuredComVelocity, zero, std::numeric_limits<double>::quiet_NaN(), 0.08),
                   zero);
  expectVectorNear(legged::ConvexRegionSelector::computeRaibertOffset(measuredComVelocity, zero, 0.55, -0.1), zero);
  expectVectorNear(legged::ConvexRegionSelector::computeRaibertOffset(
                       measuredComVelocity, zero, 0.55, std::numeric_limits<double>::infinity()),
                   zero);
}

}  // namespace
