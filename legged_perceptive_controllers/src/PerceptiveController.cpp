//
// Created by qiayuan on 23-1-3.
//

#include <pinocchio/fwd.hpp>

#include "legged_perceptive_controllers/PerceptiveController.h"

#include "legged_perceptive_controllers/synchronized_module/PlanarTerrainReceiver.h"
#include "legged_perceptive_interface/PerceptiveLeggedInterface.h"
#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"

#include <pluginlib/class_list_macros.hpp>

namespace legged {
void PerceptiveController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                                bool verbose) {
  leggedInterface_ = std::make_shared<PerceptiveLeggedInterface>(taskFile, urdfFile, referenceFile, verbose);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

  setupVisualization();
}

void PerceptiveController::setupVisualization() {
  footPlacementVisualizationPtr_ = std::make_shared<FootPlacementVisualization>(
      *dynamic_cast<PerceptiveLeggedReferenceManager&>(*leggedInterface_->getReferenceManagerPtr()).getConvexRegionSelectorPtr(),
      leggedInterface_->getCentroidalModelInfo().numThreeDofContacts, rosNode_);

  sphereVisualizationPtr_ = std::make_shared<SphereVisualization>(
      leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
      *dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPinocchioSphereInterfacePtr(), rosNode_);
}

void PerceptiveController::setupMpc() {
  LeggedController::setupMpc();

  auto planarTerrainReceiver = std::make_shared<PlanarTerrainReceiver>(
      rosNode_, dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getPlanarTerrainPtr(),
      dynamic_cast<PerceptiveLeggedInterface&>(*leggedInterface_).getSignedDistanceFieldPtr(),
      "/convex_plane_decomposition_ros/planar_terrain", "smooth_planar");
  mpc_->getSolverPtr()->addSynchronizedModule(planarTerrainReceiver);
}

controller_interface::return_type PerceptiveController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
  const auto ret = LeggedController::update(time, period);
  footPlacementVisualizationPtr_->update(currentObservation_);
  sphereVisualizationPtr_->update(currentObservation_);
  return ret;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::PerceptiveController, controller_interface::ControllerInterface)
