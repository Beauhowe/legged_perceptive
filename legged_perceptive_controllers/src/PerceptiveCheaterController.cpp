//
// Perceptive controller variant using ground-truth odometry for state estimation.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_perceptive_controllers/PerceptiveCheaterController.h"

#include <legged_estimation/FromTopiceEstimate.h>

#include <pluginlib/class_list_macros.hpp>

namespace legged {

void PerceptiveCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(rosNode_, leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveCheaterController, controller_interface::ControllerInterface)
