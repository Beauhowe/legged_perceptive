//
// Created by qiayuan on 22-12-30.
//

#pragma once
#include <rclcpp/rclcpp.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>

#include <visualization_msgs/msg/marker_array.hpp>

namespace legged {

using namespace ocs2;

class SphereVisualization {
 public:
  SphereVisualization(PinocchioInterface pinocchioInterface, CentroidalModelInfo centroidalModelInfo,
                      const PinocchioSphereInterface& sphereInterface, rclcpp::Node::SharedPtr node, scalar_t maxUpdateFrequency = 100.0);
  void update(const SystemObservation& observation);

 private:
  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo centroidalModelInfo_;
  const PinocchioSphereInterface& sphereInterface_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;

  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace legged
