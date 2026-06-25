//
// Created by qiayuan on 22-12-24.
//

#include "legged_perceptive_controllers/synchronized_module/PlanarTerrainReceiver.h"

#include <convex_plane_decomposition_ros/MessageConversion.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <utility>

namespace legged {

namespace {

constexpr const char* kSmoothPlanarLayer = "smooth_planar";
constexpr const char* kElevationBeforePostprocessLayer = "elevation_before_postprocess";
constexpr const char* kElevationLayer = "elevation";

std::string resolveSdfLayer(const grid_map::GridMap& map, const std::string& preferred) {
  if (!preferred.empty() && map.exists(preferred)) {
    return preferred;
  }
  if (map.exists(kSmoothPlanarLayer)) {
    return kSmoothPlanarLayer;
  }
  if (map.exists(kElevationBeforePostprocessLayer)) {
    return kElevationBeforePostprocessLayer;
  }
  if (map.exists(kElevationLayer)) {
    return kElevationLayer;
  }
  return {};
}

}  // namespace

PlanarTerrainReceiver::PlanarTerrainReceiver(rclcpp::Node::SharedPtr node,
                                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                                             std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr,
                                             const std::string& mapTopic, std::string sdfElevationLayer)
    : node_(std::move(node)),
      signedDistanceField_(*signedDistanceFieldPtr),
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
    *sdfPtr_ = signedDistanceField_;
  }
}

void PlanarTerrainReceiver::planarTerrainCallback(const convex_plane_decomposition_msgs::msg::PlanarTerrain::ConstSharedPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);

  planarTerrain_ = convex_plane_decomposition::PlanarTerrain(convex_plane_decomposition::fromMessage(*msg));

  const std::string sdfLayer = resolveSdfLayer(planarTerrain_.gridMap, sdfElevationLayer_);
  if (sdfLayer.empty()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[PlanarTerrainReceiver] planar_terrain has no usable SDF layer "
                         "(tried preferred='%s', smooth_planar, elevation_before_postprocess, elevation)",
                         sdfElevationLayer_.c_str());
    return;
  }

  auto& elevationData = planarTerrain_.gridMap.get(sdfLayer);
  if (!elevationData.array().isFinite().any()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[PlanarTerrainReceiver] Layer '%s' has no finite values; skipping terrain update", sdfLayer.c_str());
    return;
  }

  if (elevationData.hasNaN()) {
    const float inpaint{elevationData.minCoeffOfFinites()};
    RCLCPP_WARN(node_->get_logger(), "[PlanarTerrainReceiver] Map contains NaN values. Will apply inpainting with min value.");
    elevationData = elevationData.unaryExpr([=](float v) { return std::isfinite(v) ? v : inpaint; });
  }

  const float heightMargin{0.1};
  const float minValue{elevationData.minCoeffOfFinites() - heightMargin};
  const float maxValue{elevationData.maxCoeffOfFinites() + 3 * heightMargin};
  signedDistanceField_ = grid_map::SignedDistanceField(planarTerrain_.gridMap, sdfLayer, minValue, maxValue);
  updated_ = true;
}

}  // namespace legged
