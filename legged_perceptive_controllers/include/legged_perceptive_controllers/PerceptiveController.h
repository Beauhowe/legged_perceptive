//
// Created by qiayuan on 23-1-3.
//

#pragma once

#include "legged_perceptive_controllers/visualization/FootPlacementVisualization.h"
#include "legged_perceptive_controllers/visualization/SphereVisualization.h"

#include <legged_controllers/LeggedController.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class PerceptiveController : public legged::LeggedController {
 protected:
  void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                            bool verbose) override;

  void setupMpc() override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  void setupVisualization();

 private:
  std::shared_ptr<FootPlacementVisualization> footPlacementVisualizationPtr_;
  std::shared_ptr<SphereVisualization> sphereVisualizationPtr_;
};

}  // namespace legged
