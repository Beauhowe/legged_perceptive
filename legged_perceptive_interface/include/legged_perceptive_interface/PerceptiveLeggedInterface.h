//
// Created by qiayuan on 22-12-27.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <legged_interface/LeggedInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <grid_map_sdf/SignedDistanceField.hpp>

#include "legged_perceptive_interface/ContactLegRootResolver.h"
#include "legged_perceptive_interface/PerceptiveFootholdPlanningSettings.h"

namespace legged {

class PerceptiveLeggedInterface : public legged::LeggedInterface {
 public:
  using LeggedInterface::LeggedInterface;

  void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                  bool verbose) override;

  void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                             bool verbose) override;

  void setupPreComputation(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                           bool verbose) override;

  std::shared_ptr<grid_map::SignedDistanceField> getSignedDistanceFieldPtr() const { return signedDistanceFieldPtr_; }

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const { return planarTerrainPtr_; }

  std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const { return pinocchioSphereInterfacePtr_; }

  size_t getNumVertices() const { return numVertices_; }

 protected:
  size_t numVertices_ = 16;

  PerceptiveFootholdPlanningSettings perceptiveFootholdPlanningSettings_;

  /**
   * @brief 与运行时 contactNames3DoF 同序的 HAA 映射和左右侧别。
   *
   * 运动学惩罚关闭时为空；S2 只解析并保存，尚未把结果接入候选代价。
   */
  std::vector<ContactLegRoot> contactLegRoots_;

  std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
  std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr_;
  std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;
};

}  // namespace legged
