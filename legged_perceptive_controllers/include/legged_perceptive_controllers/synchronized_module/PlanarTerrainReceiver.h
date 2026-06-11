//
// Created by qiayuan on 22-12-24.
//

#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

namespace legged {

using namespace ocs2;

class PlanarTerrainReceiver : public SolverSynchronizedModule {
 public:
  PlanarTerrainReceiver(rclcpp::Node::SharedPtr node, std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                        std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr, const std::string& mapTopic,
                        std::string elevationLayer);

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override{};

 private:
  void planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr& msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr subscriber_;
  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  // apt (RSL) grid_map_sdf 2.0.1's SignedDistanceField has a deleted copy-assignment operator
  // (it holds a const member), unlike the ANYbotics master version this code was written against.
  // We therefore stage the freshly computed SDF in a unique_ptr buffer and copy-construct it
  // in place into the shared object (see PlanarTerrainReceiver.cpp) instead of assigning.
  std::unique_ptr<grid_map::SignedDistanceField> stagedSdf_;

  std::string sdfElevationLayer_;

  std::mutex mutex_;
  std::atomic_bool updated_;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
};

}  // namespace legged
