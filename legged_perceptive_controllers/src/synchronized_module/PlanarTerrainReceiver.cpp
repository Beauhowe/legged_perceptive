//
// Created by qiayuan on 22-12-24.
//

#include "legged_perceptive_controllers/synchronized_module/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <utility>

namespace legged {

PlanarTerrainReceiver::PlanarTerrainReceiver(rclcpp::Node::SharedPtr node,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr,
                                             const std::string& mapTopic, std::string sdfElevationLayer)
    : node_(std::move(node)),
      planarTerrainPtr_(std::move(planarTerrainPtr)),
      sdfPtr_(std::move(signedDistanceFieldPtr)),
      sdfElevationLayer_(std::move(sdfElevationLayer)),
      updated_(false) {
  subscriber_ = node_->create_subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>(
      mapTopic, 1, [this](const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr msg) { planarTerrainCallback(msg); });
}

void PlanarTerrainReceiver::preSolverRun(scalar_t /*initTime*/, scalar_t /*finalTime*/, const vector_t& /*currentState*/,
                                         const ReferenceManagerInterface& /*referenceManager*/) {
  if (updated_) {
    std::lock_guard<std::mutex> lock(mutex_);
    updated_ = false;

    *planarTerrainPtr_ = planarTerrain_;
    // The original mutated the shared SDF in place via operator=. apt grid_map_sdf 2.0.1 deletes
    // copy-assignment (const member) but allows copy-construction, so we destroy and copy-construct
    // in place to keep the object's address — every constraint aliases this same shared_ptr.
    // preSolverRun is solver-synchronized, so no constraint reads the SDF concurrently here.
    if (stagedSdf_) {
      sdfPtr_->~SignedDistanceField();
      new (sdfPtr_.get()) grid_map::SignedDistanceField(*stagedSdf_);
    }
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  updated_ = true;

  planarTerrain_ = convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg));

  auto& elevationData = planarTerrain_.gridMap.get(sdfElevationLayer_);
  if (elevationData.hasNaN()) {
    const float inpaint{elevationData.minCoeffOfFinites()};
    RCLCPP_WARN(node_->get_logger(), "[PlanarTerrainReceiver] Map contains NaN values. Will apply inpainting with min value.");
    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
  }
  const float heightMargin{0.1};
  // apt (RSL) grid_map_sdf 2.0.1 API: default-construct then calculate.
  // Original (ANYbotics master) used SignedDistanceField(gridMap, layer, dataMin - heightMargin,
  // dataMax + 3*heightMargin), an explicit absolute [min, max] Z range. The apt version derives the
  // lower bound from the data's min and only takes a clearance above the data's max, so we pass the
  // clearance that reproduces the original upper bound (3*heightMargin above dataMax). The original
  // lower bound extended one heightMargin below dataMin; the apt version starts exactly at dataMin,
  // so the SDF grid is one margin shorter at the bottom. See edit.md "待验证项" for the behavioral note.
  stagedSdf_ = std::make_unique<grid_map::SignedDistanceField>();
  stagedSdf_->calculateSignedDistanceField(planarTerrain_.gridMap, sdfElevationLayer_, 3 * heightMargin);
}

}  // namespace legged
