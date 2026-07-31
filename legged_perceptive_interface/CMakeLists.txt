cmake_minimum_required(VERSION 3.5)
project(legged_perceptive_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(legged_interface REQUIRED)
find_package(convex_plane_decomposition_msgs REQUIRED)
find_package(convex_plane_decomposition_ros REQUIRED)
find_package(grid_map_sdf REQUIRED)
find_package(ocs2_sphere_approximation REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(pinocchio REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem log_setup log)

add_library(${PROJECT_NAME}
  src/constraint/FootCollisionConstraint.cpp
  src/constraint/FootPlacementConstraint.cpp
  src/constraint/SphereSdfConstraint.cpp
  src/ContactLegRootResolver.cpp
  src/ConvexRegionSelector.cpp
  src/TargetTrajectoryResampler.cpp
  src/PerceptiveFootholdPlanningSettings.cpp
  src/PerceptiveLeggedReferenceManager.cpp
  src/PerceptiveLeggedPrecomputation.cpp
  src/PerceptiveLeggedInterface.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  Boost::filesystem
  Boost::system
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  legged_interface
  convex_plane_decomposition_msgs
  convex_plane_decomposition_ros
  grid_map_sdf
  ocs2_sphere_approximation
)
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_PINOCCHIO_FLAGS})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(
  EXPORT export_${PROJECT_NAME}
  DESTINATION share/${PROJECT_NAME}/cmake
)

ament_export_include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  legged_interface
  convex_plane_decomposition_msgs
  convex_plane_decomposition_ros
  grid_map_sdf
  ocs2_sphere_approximation
  pinocchio
  Eigen3
  Boost
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_convex_region_selector_timing
    test/test_convex_region_selector_timing.cpp
  )
  target_link_libraries(test_convex_region_selector_timing
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_perceptive_reference_manager
    test/test_perceptive_reference_manager.cpp
  )
  target_link_libraries(test_perceptive_reference_manager
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_perceptive_foothold_planning_settings
    test/test_perceptive_foothold_planning_settings.cpp
  )
  target_link_libraries(test_perceptive_foothold_planning_settings
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_contact_leg_root_resolver
    test/test_contact_leg_root_resolver.cpp
  )
  target_link_libraries(test_contact_leg_root_resolver
    ${PROJECT_NAME}
  )
  target_compile_options(test_perceptive_reference_manager PRIVATE ${OCS2_PINOCCHIO_FLAGS})
  target_compile_options(test_convex_region_selector_timing PRIVATE ${OCS2_PINOCCHIO_FLAGS})
  target_compile_options(test_perceptive_foothold_planning_settings PRIVATE ${OCS2_PINOCCHIO_FLAGS})
  target_compile_options(test_contact_leg_root_resolver PRIVATE ${OCS2_PINOCCHIO_FLAGS})
endif()

ament_package()
