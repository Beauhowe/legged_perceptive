//
// Created by qiayuan on 23-1-3.
//
#include <utility>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

#include "legged_perceptive_interface/PerceptiveLeggedReferenceManager.h"
#include "legged_perceptive_interface/TargetTrajectoryResampler.h"

namespace legged {

PerceptiveLeggedReferenceManager::PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                                   std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                                   std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   scalar_t comHeight)
    : info_(std::move(info)),
      SwitchedModelReferenceManager(std::move(gaitSchedulePtr), std::move(swingTrajectoryPtr)),
      convexRegionSelectorPtr_(std::move(convexRegionSelectorPtr)),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      comHeight_(comHeight) {}

void PerceptiveLeggedReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                        TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = getGaitSchedule()->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  bool hasDynamicPhase = false;
  for (const auto mode : modeSchedule.modeSequence) {
    hasDynamicPhase = hasDynamicPhase || mode != ModeNumber::STANCE;
  }
  if (!hasDynamicPhase) {
    dynamicGaitDiagnosticLogged_ = false;
  }

  TargetTrajectories newTargetTrajectories =
      resampleTargetTrajectories(targetTrajectories, initTime, finalTime, 11);
  const size_t nodeNum = newTargetTrajectories.timeTrajectory.size();
  for (size_t i = 0; i < nodeNum; ++i) {
    const scalar_t time = newTargetTrajectories.timeTrajectory[i];
    vector_t& state = newTargetTrajectories.stateTrajectory[i];

    const auto& map = convexRegionSelectorPtr_->getPlanarTerrainPtr()->gridMap;
    const grid_map::Position pos = centroidal_model::getBasePose(state, info_).head<2>();

    // Base Orientation
    scalar_t step = 0.3;
    grid_map::Vector3 normalVector;
    normalVector(0) = (map.atPosition("smooth_planar", pos + grid_map::Position(-step, 0)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(step, 0))) /
                      (2 * step);
    normalVector(1) = (map.atPosition("smooth_planar", pos + grid_map::Position(0, -step)) -
                       map.atPosition("smooth_planar", pos + grid_map::Position(0, step))) /
                      (2 * step);
    normalVector(2) = 1;
    normalVector.normalize();
    matrix3_t R;
    scalar_t z = centroidal_model::getBasePose(state, info_)(3);
    R << cos(z), -sin(z), 0,  // clang-format off
             sin(z), cos(z), 0,
             0, 0, 1;  // clang-format on
    vector_t v = R.transpose() * normalVector;
    centroidal_model::getBasePose(state, info_)(4) = atan(v.x() / v.z());

    // Base Z Position
    centroidal_model::getBasePose(state, info_)(2) =
        map.atPosition("smooth_planar", pos) + comHeight_ ;/// cos(centroidal_model::getBasePose(state, info_)(4));
  }
  targetTrajectories = newTargetTrajectories;

  // Footstep
  convexRegionSelectorPtr_->update(modeSchedule, initTime, initState, targetTrajectories);

  if (hasDynamicPhase && !dynamicGaitDiagnosticLogged_) {
    std::ostringstream log;
    log << "[PerceptiveDiag] dynamic gait snapshot: init=" << initTime << " final=" << finalTime << " modes=[";
    for (size_t i = 0; i < modeSchedule.modeSequence.size(); ++i) {
      log << (i == 0 ? "" : ",") << modeSchedule.modeSequence[i];
    }
    log << "] events=[";
    for (size_t i = 0; i < modeSchedule.eventTimes.size(); ++i) {
      log << (i == 0 ? "" : ",") << modeSchedule.eventTimes[i];
    }
    log << "] init_state_finite=" << initState.allFinite();

    if (!targetTrajectories.stateTrajectory.empty()) {
      const auto& firstState = targetTrajectories.stateTrajectory.front();
      const auto& lastState = targetTrajectories.stateTrajectory.back();
      log << " target_state_finite=" << (firstState.allFinite() && lastState.allFinite())
          << " first_base=" << centroidal_model::getBasePose(firstState, info_).transpose()
          << " last_base=" << centroidal_model::getBasePose(lastState, info_).transpose();
    }
    if (!targetTrajectories.inputTrajectory.empty()) {
      const auto& firstInput = targetTrajectories.inputTrajectory.front();
      const auto& lastInput = targetTrajectories.inputTrajectory.back();
      log << " input_dims=" << firstInput.size() << "," << lastInput.size()
          << " input_finite=" << (firstInput.allFinite() && lastInput.allFinite())
          << " input_norms=" << firstInput.norm() << "," << lastInput.norm();
    }

    const auto contactFlags = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
    for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg) {
      const auto projections = convexRegionSelectorPtr_->getProjections(leg);
      log << "\n  leg=" << leg << " flags=";
      for (const auto flag : contactFlags[leg]) {
        log << (flag ? '1' : '0');
      }
      log << " projections=" << projections.size();
      for (size_t phase = 0; phase < projections.size(); ++phase) {
        log << " p" << phase << "=" << projections[phase].positionInWorld.transpose();
      }
    }
    std::cerr << log.str() << std::endl;
    dynamicGaitDiagnosticLogged_ = true;
  }

  // Swing trajectory
  updateSwingTrajectoryPlanner(initTime, initState, modeSchedule);
}

void PerceptiveLeggedReferenceManager::updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                                    ModeSchedule& modeSchedule) {
  const auto contactFlagStocks = convexRegionSelectorPtr_->extractContactFlags(modeSchedule.modeSequence);
  feet_array_t<scalar_array_t> liftOffHeightSequence, touchDownHeightSequence;

  for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++) {
    size_t initIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime);

    auto projections = convexRegionSelectorPtr_->getProjections(leg);
    if (contactFlagStocks[leg].size() != projections.size() || projections.empty() || initIndex >= projections.size()) {
      throw std::invalid_argument("[PerceptiveLeggedReferenceManager] Invalid contact/projection phase data.");
    }
    modifyProjections(initTime, initState, leg, initIndex, contactFlagStocks[leg], projections);

    scalar_array_t liftOffHeights, touchDownHeights;
    std::tie(liftOffHeights, touchDownHeights) = getHeights(contactFlagStocks[leg], projections);
    liftOffHeightSequence[leg] = liftOffHeights;
    touchDownHeightSequence[leg] = touchDownHeights;
  }
  swingTrajectoryPtr_->update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

void PerceptiveLeggedReferenceManager::modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                                                         const std::vector<bool>& contactFlagStocks,
                                                         std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections) {
  if (initIndex >= contactFlagStocks.size() || contactFlagStocks.size() != projections.size()) {
    throw std::invalid_argument("[PerceptiveLeggedReferenceManager] Invalid projection phase index.");
  }
  if (contactFlagStocks[initIndex]) {
    lastLiftoffPos_[leg] = endEffectorKinematicsPtr_->getPosition(initState)[leg];
    lastLiftoffPos_[leg].z() -= 0.02;
    lastLiftoffPosValid_[leg] = true;
    for (int i = initIndex; i < projections.size(); ++i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
    for (int i = initIndex; i >= 0; --i) {
      if (!contactFlagStocks[i]) {
        break;
      }
      projections[i].positionInWorld = lastLiftoffPos_[leg];
    }
  }
  if (initTime > convexRegionSelectorPtr_->getInitStandFinalTimes()[leg]) {
    for (int i = initIndex; i >= 0; --i) {
      if (contactFlagStocks[i] && lastLiftoffPosValid_[leg]) {
        projections[i].positionInWorld = lastLiftoffPos_[leg];
      }
      if (!contactFlagStocks[i] && (i + 1 >= static_cast<int>(contactFlagStocks.size()) || !contactFlagStocks[i + 1])) {
        break;
      }
    }
  }
  //    for (int i = 0; i < numPhases; ++i) {
  //      if (leg == 1) std::cerr << std::setprecision(3) << projections[i].positionInWorld.z() << "\t";
  //    }
  //    std::cerr << std::endl;
}

std::pair<scalar_array_t, scalar_array_t> PerceptiveLeggedReferenceManager::getHeights(
    const std::vector<bool>& contactFlagStocks, const std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections) {
  scalar_array_t liftOffHeights, touchDownHeights;
  const size_t numPhases = projections.size();

  liftOffHeights.clear();
  liftOffHeights.resize(numPhases);
  touchDownHeights.clear();
  touchDownHeights.resize(numPhases);

  for (size_t i = 1; i < numPhases; ++i) {
    if (!contactFlagStocks[i]) {
      liftOffHeights[i] = contactFlagStocks[i - 1] ? projections[i - 1].positionInWorld.z() : liftOffHeights[i - 1];
    }
  }
  for (int i = numPhases - 2; i >= 0; --i) {
    if (!contactFlagStocks[i]) {
      touchDownHeights[i] = contactFlagStocks[i + 1] ? projections[i + 1].positionInWorld.z() : touchDownHeights[i + 1];
    }
  }

  //  for (int i = 0; i < numPhases; ++i) {
  //    std::cerr << std::setprecision(3) << liftOffHeights[i] << "\t";
  //  }
  //  std::cerr << std::endl;
  //  for (int i = 0; i < numPhases; ++i) {
  //    std::cerr << std::setprecision(3) << contactFlagStocks[i] << "\t";
  //  }
  //  std::cerr << std::endl;

  return {liftOffHeights, touchDownHeights};
}

contact_flag_t PerceptiveLeggedReferenceManager::getFootPlacementFlags(scalar_t time) const {
  contact_flag_t flag;
  const auto finalTime = convexRegionSelectorPtr_->getInitStandFinalTimes();
  for (int i = 0; i < flag.size(); ++i) {
    flag[i] = getContactFlags(time)[i] && time >= finalTime[i];
  }
  return flag;
}

}  // namespace legged
