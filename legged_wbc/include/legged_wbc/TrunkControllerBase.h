//
// Created by qiayuan on 2022/7/1.
//

#pragma once

#include "legged_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace legged {
using namespace ocs2;
using namespace legged_robot;

// Decision Variables: x = [\dot u^T, F^T, \tau^T]^T
class TrunkControllerBase {
  using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
  using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

 public:
  TrunkControllerBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics);

  virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

  virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode, vector_t typeFlag);

 protected:
  void updateMeasured(const vector_t& rbdStateMeasured);
  void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired);

  size_t getNumDecisionVars() const { return numDecisionVars_; }
 
  Task formulateFloatingBaseEomTask();
  Task formulateNoContactMotionTask();
  Task formulateRollingTask(const vector_t& stateDesired);
  Task formulateFrictionConeTask();
  Task formulateBaseAccelTask(const vector_t& stateDesired);
  Task formulateSwingLegTask(const vector_t& stateDesired);
  Task formulateContactForceTask(const vector_t& inputDesired) const;
  Task formulateTorqueLimitsTask();

  size_t numDecisionVars_;
  PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
  CentroidalModelInfo info_;

  std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;
  CentroidalModelPinocchioMapping mapping_;

  vector_t qMeasured_, vMeasured_, inputLast_;
  matrix_t j_, dj_, jst_;
  contact_flag_t contactFlag_{};
  vector_t typeFlag_{};
  size_t numContacts_{};

  // Task Parameters:
  vector_t torqueLimits_;
  scalar_t frictionCoeff_{}, frictionWheelCoeff_{}, swingKp_{}, swingKd_{}, rollingKp_{}, rollingKd_{}, linStanceKp_{}, linStanceKd_{}, angStanceKp_{}, angStanceKd_{};
};

}  // namespace legged
