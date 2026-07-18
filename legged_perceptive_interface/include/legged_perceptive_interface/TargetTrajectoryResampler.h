#pragma once

#include <cstddef>

#include <ocs2_core/reference/TargetTrajectories.h>

namespace legged {

ocs2::TargetTrajectories resampleTargetTrajectories(const ocs2::TargetTrajectories& source, ocs2::scalar_t initTime,
                                                    ocs2::scalar_t finalTime, std::size_t nodeNum);

}  // namespace legged
