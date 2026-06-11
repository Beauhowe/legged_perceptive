cmake_minimum_required(VERSION 3.10)
project(legged_perceptive_controllers)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(legged_perceptive_interface REQUIRED)
find_package(legged_controllers REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_sphere_approximation REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)
# grid_map_sdf's public header includes <pcl/point_cloud.h> without propagating PCL include dirs.
find_package(PCL REQUIRED)

set(dependencies
        rclcpp
        controller_interface
        pluginlib
        legged_perceptive_interface
        legged_controllers
        convex_plane_decomposition_msgs
        convex_plane_decomposition_ros
        grid_map_ros
        grid_map_msgs
        grid_map_sdf
        visualization_msgs
        geometry_msgs
        std_msgs
        tf2_ros
        ocs2_ros_interfaces
        ocs2_sphere_approximation
        pinocchio
)

add_library(${PROJECT_NAME} SHARED
        src/synchronized_module/PlanarTerrainReceiver.cpp
        src/visualization/FootPlacementVisualization.cpp
        src/visualization/SphereVisualization.cpp
        src/PerceptiveController.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${EIGEN3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
        pinocchio::pinocchio
        Boost::filesystem
        Boost::system
        ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS} ${OCS2_PINOCCHIO_FLAGS})
pluginlib_export_plugin_description_file(controller_interface legged_perceptive_controllers_plugins.xml)

# Test grid map publisher
add_executable(grid_map_publisher
        test/MapPublisher.cpp)
ament_target_dependencies(grid_map_publisher
        rclcpp
        grid_map_ros
        grid_map_msgs
        tf2_ros
)
target_compile_options(grid_map_publisher PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS grid_map_publisher
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION include/${PROJECT_NAME}
)

install(FILES legged_perceptive_controllers_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies} Eigen3 Boost PCL)

ament_package()
