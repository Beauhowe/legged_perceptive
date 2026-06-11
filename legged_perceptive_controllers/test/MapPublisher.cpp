//
// Created by qiayuan on 23-2-4.
//

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

using namespace grid_map;
using namespace std::chrono_literals;

int main(int argc, char** argv) {
  // Initialize node and publisher.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("map_publisher");
  auto gridMapPublisher = node->create_publisher<grid_map_msgs::msg::GridMap>(
      "/elevation_mapping/elevation_map_raw", rclcpp::QoS(1).transient_local());

  tf2_ros::Buffer buffer(node->get_clock());
  tf2_ros::TransformListener transformListener(buffer);

  // Create grid map.
  GridMap map({"elevation"});
  map.setFrameId("odom");
  map.setGeometry(Length(10.0, 10.0), 0.03);
  map["elevation"].setConstant(0);

  grid_map::Polygon polygon;
  polygon.setFrameId(map.getFrameId());
  polygon.addVertex(Position(0.5, 0.3));
  polygon.addVertex(Position(0.6, 0.3));
  polygon.addVertex(Position(0.6, -0.3));
  polygon.addVertex(Position(0.5, -0.3));

  for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd(); ++iterator) {
    map.at("elevation", *iterator) = 0.16;
  }

  rclcpp::Rate rate(3.0);
  while (rclcpp::ok()) {
    // Add data to grid map.
    rclcpp::Time time = node->now();

    try {
      geometry_msgs::msg::TransformStamped tran = buffer.lookupTransform("odom", "base", tf2::TimePointZero);
      map.move(Position(tran.transform.translation.x, tran.transform.translation.y));
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(node->get_logger(), "Failure %s\n", ex.what());
    }

    // Publish grid map.
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> message = GridMapRosConverter::toMessage(map);
    gridMapPublisher->publish(std::move(message));

    rclcpp::spin_some(node);
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
