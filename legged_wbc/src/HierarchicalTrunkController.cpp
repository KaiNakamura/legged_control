//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/HierarchicalTrunkController.h"

#include "legged_wbc/HoQp.h"

namespace legged {
vector_t HierarchicalTrunkController::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode, vector_t typeFlag) {
  TrunkControllerBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, typeFlag);

  Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() + formulateNoContactMotionTask();
  Task task1 = formulateSwingLegTask(stateDesired);

  Task accelTask = formulateBaseAccelTask(stateDesired);
  Task xTask = Task({accelTask.a_.row(0), accelTask.b_.row(0), matrix_t(), vector_t()});
  Task yTask = Task({accelTask.a_.row(1), accelTask.b_.row(1), matrix_t(), vector_t()});
  Task zTask = Task({accelTask.a_.row(2), accelTask.b_.row(2), matrix_t(), vector_t()});
  Task oTask = Task({accelTask.a_.block(3, 0, 3, accelTask.a_.rows()), accelTask.b_.block(3, 0, 3, accelTask.a_.rows()), matrix_t(), vector_t()});

  Task task3 = formulateContactForceTask(inputDesired);
  HoQp hoQp(task3, std::make_shared<HoQp>(oTask, std::make_shared<HoQp>(zTask, std::make_shared<HoQp>(xTask, std::make_shared<HoQp>(yTask, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)))))));

  return hoQp.getSolutions();
}

}  // namespace legged
