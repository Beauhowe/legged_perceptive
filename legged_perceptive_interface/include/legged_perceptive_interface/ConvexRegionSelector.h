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

  /**
   * @brief 根据世界系 CoM 速度误差计算 Raibert 水平落脚修正量。
   *
   * 使用倒立摆时间尺度 sqrt(invertedPendulumHeight / 9.81)，将实测速度减期望速度
   * 的 x/y 分量转换为水平落脚偏移；随后按二维欧氏模长限制到 raibertMaxOffset，
   * 不分别裁剪 x、y。输出 z 始终为零。
   *
   * 任一速度分量、参数或中间结果非有限，或者参数为负时，返回零向量作为安全
   * 回退。该纯函数不读取开关、不保存历史，也不修改候选落脚点。
   *
   * @param measuredComVelocity 世界系实测 CoM 速度，单位 m/s。
   * @param desiredComVelocity 世界系期望 CoM 速度，单位 m/s。
   * @param invertedPendulumHeight 倒立摆等效高度，单位 m。
   * @param raibertMaxOffset 允许的最大水平偏移模长，单位 m。
   * @return 世界系 Raibert 落脚修正量，单位 m，且 z 恒为零。
   */
  static vector3_t computeRaibertOffset(const vector3_t& measuredComVelocity, const vector3_t& desiredComVelocity,
                                        scalar_t invertedPendulumHeight, scalar_t raibertMaxOffset);

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
