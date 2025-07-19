//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalTrunkController.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalTrunkController::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode) {
  TrunkControllerBase::update(stateDesired, inputDesired, rbdStateMeasured, mode);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateSwingLegTask(stateDesired);
  Task task2 = formulateBaseAccelTask(stateDesired);
  Task task3 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task3, std::make_shared<HoQp>(task2, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0))));

  return hoQp.getSolutions();
}

}  // namespace legged
