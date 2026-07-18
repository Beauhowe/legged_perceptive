#include "legged_perceptive_interface/TargetTrajectoryResampler.h"

#include <stdexcept>

namespace legged {

ocs2::TargetTrajectories resampleTargetTrajectories(const ocs2::TargetTrajectories& source, ocs2::scalar_t initTime,
                                                    ocs2::scalar_t finalTime, std::size_t nodeNum) {
  if (nodeNum < 2) {
    throw std::invalid_argument("[TargetTrajectoryResampler] nodeNum must be at least 2.");
  }
  if (source.empty() || source.timeTrajectory.size() != source.stateTrajectory.size() ||
      source.timeTrajectory.size() != source.inputTrajectory.size()) {
    throw std::invalid_argument("[TargetTrajectoryResampler] source trajectories must have matching non-empty sizes.");
  }

  ocs2::TargetTrajectories result;
  result.timeTrajectory.reserve(nodeNum);
  result.stateTrajectory.reserve(nodeNum);
  result.inputTrajectory.reserve(nodeNum);
  const auto horizon = finalTime - initTime;
  for (std::size_t i = 0; i < nodeNum; ++i) {
    const auto time = initTime + static_cast<double>(i) * horizon / static_cast<double>(nodeNum - 1);
    result.timeTrajectory.push_back(time);
    result.stateTrajectory.push_back(source.getDesiredState(time));
    result.inputTrajectory.push_back(source.getDesiredInput(time));
  }
  return result;
}

}  // namespace legged
