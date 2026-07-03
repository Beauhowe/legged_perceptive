//
// Created by qiayuan on 22-12-30.
//

#include "legged_perceptive_controllers/visualization/FootPlacementVisualization.h"

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

namespace legged {
FootPlacementVisualization::FootPlacementVisualization(const ConvexRegionSelector& convexRegionSelector, size_t numFoot,
                                                       rclcpp::Node::SharedPtr node, scalar_t maxUpdateFrequency)
    : convexRegionSelector_(convexRegionSelector),
      numFoot_(numFoot),
      markerPublisher_(node->create_publisher<visualization_msgs::msg::MarkerArray>("foot_placement", 1)),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {}

void FootPlacementVisualization::update(const SystemObservation& observation) {
  if (observation.time - lastTime_ > minPublishTimeDifference_) {
    lastTime_ = observation.time;

    std_msgs::msg::Header header;
    //    header.stamp.fromNSec(planarTerrainPtr->gridMap.getTimestamp());
    header.frame_id = "odom";

    visualization_msgs::msg::MarkerArray makerArray;

    size_t i = 0;
    for (int leg = 0; leg < numFoot_; ++leg) {
      const auto footPlacements = convexRegionSelector_.getFootPlacements(leg);

      int kStart = 0;
      for (int k = 0; k < footPlacements.size(); ++k) {
        if (footPlacements[k].middleTime < observation.time) {
          kStart = k + 1;
          continue;
        }
        auto color = feetColorMap_[leg];
        float alpha = 1 - static_cast<float>(k - kStart) / static_cast<float>(footPlacements.size() - kStart);
        // Projections
        auto projectionMaker = getArrowAtPointMsg(footPlacements[k].projectionNormal, footPlacements[k].positionInWorld, color);
        projectionMaker.header = header;
        projectionMaker.ns = "Projections";
        projectionMaker.id = i;
        projectionMaker.color.a = alpha;
        makerArray.markers.push_back(projectionMaker);

        // Convex Region
        makerArray.markers.push_back(to3dRosMarker(footPlacements[k].convexRegion, footPlacements[k].transformPlaneToWorld, header, color,
                                                   alpha, i));

        // Nominal Footholds
        auto nominalMarker = getFootMarker(footPlacements[k].nominalFoothold, true, color, footMarkerDiameter_, 1.);
        nominalMarker.header = header;
        nominalMarker.ns = "Nominal Footholds";
        nominalMarker.id = i;
        nominalMarker.color.a = alpha;
        makerArray.markers.push_back(nominalMarker);

        i++;
      }
    }

    markerPublisher_->publish(makerArray);
  }
}

visualization_msgs::msg::Marker FootPlacementVisualization::to3dRosMarker(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                                                          const Eigen::Isometry3d& transformPlaneToWorld,
                                                                          const std_msgs::msg::Header& header, Color color, float alpha,
                                                                          size_t i) {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Convex Regions";
  marker.id = i;
  marker.header = header;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.scale.x = lineWidth_;
  marker.color = getColor(color, alpha);
  if (!polygon.is_empty()) {
    marker.points.reserve(polygon.size() + 1);
    for (const auto& point : polygon) {
      const auto pointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(point, transformPlaneToWorld);
      geometry_msgs::msg::Point point_ros;
      point_ros.x = pointInWorld.x();
      point_ros.y = pointInWorld.y();
      point_ros.z = pointInWorld.z();
      marker.points.push_back(point_ros);
    }
    // repeat the first point to close to polygon
    const auto pointInWorld =
        convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(polygon.vertex(0), transformPlaneToWorld);
    geometry_msgs::msg::Point point_ros;
    point_ros.x = pointInWorld.x();
    point_ros.y = pointInWorld.y();
    point_ros.z = pointInWorld.z();
    marker.points.push_back(point_ros);
  }
  marker.pose.orientation.w = 1.0;
  return marker;
}

}  // namespace legged
