//
// Created by qiayuan on 23-1-2.
//

#pragma once

#include <mutex>

#include <ocs2_core/reference/ModeSchedule.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class ConvexRegionSelector {
 public:
  struct FootPlacement {
    scalar_t middleTime{0.0};
    vector3_t projectionNormal{vector3_t::Zero()};
    vector3_t positionInWorld{vector3_t::Zero()};
    Eigen::Isometry3d transformPlaneToWorld{Eigen::Isometry3d::Identity()};
    convex_plane_decomposition::CgalPolygon2d convexRegion;
    vector3_t nominalFoothold{vector3_t::Zero()};
  };

  ConvexRegionSelector(CentroidalModelInfo info, std::shared_ptr<convex_plane_decomposition::PlanarTerrain> PlanarTerrainPtr,
                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numVertices);

  void update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState, TargetTrajectories& targetTrajectories);

  convex_plane_decomposition::PlanarTerrainProjection getProjection(size_t leg, scalar_t time) const;

  convex_plane_decomposition::CgalPolygon2d getConvexPolygon(size_t leg, scalar_t time) const;

  vector3_t getNominalFootholds(size_t leg, scalar_t time) const;

  std::vector<scalar_t> getMiddleTimes(size_t leg) const;

  std::vector<convex_plane_decomposition::PlanarTerrainProjection> getProjections(size_t leg) const;

  std::vector<FootPlacement> getFootPlacements(size_t leg) const;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() { return planarTerrainPtr_; }

  feet_array_t<scalar_t> getInitStandFinalTimes() const;

  feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

 private:
  static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

  vector3_t getNominalFoothold(size_t leg, scalar_t time, const vector_t& initState, TargetTrajectories& targetTrajectories);

  feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
  feet_array_t<std::vector<convex_plane_decomposition::CgalPolygon2d>> convexPolygons_;

  feet_array_t<std::vector<vector3_t>> nominalFootholds_;
  feet_array_t<std::vector<scalar_t>> middleTimes_;

  feet_array_t<scalar_t> initStandFinalTime_;

  feet_array_t<std::vector<scalar_t>> timeEvents_;

  const CentroidalModelInfo info_;
  size_t numVertices_;

  convex_plane_decomposition::PlanarTerrain planarTerrain_;
  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

  mutable std::mutex mutex_;
};
}  // namespace legged
