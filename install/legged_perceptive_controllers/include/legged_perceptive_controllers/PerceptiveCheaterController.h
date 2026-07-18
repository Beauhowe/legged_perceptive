//
// Perceptive controller variant that uses ground-truth odometry for state
// estimation (FromTopicStateEstimate, subscribing /ground_truth/state).
//
// Use this in simulation so the odom frame coincides exactly with the Gazebo
// world origin, keeping the robot aligned with the pre-built terrain/elevation
// map. The plain PerceptiveController uses the drifting KalmanFilterEstimate.
//

#pragma once

#include "legged_perceptive_controllers/PerceptiveController.h"

namespace legged {

class PerceptiveCheaterController : public PerceptiveController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
