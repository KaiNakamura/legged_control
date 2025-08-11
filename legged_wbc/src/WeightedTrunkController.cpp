//
// Created by qiayuan on 22-12-23.
//

#include "legged_wbc/WeightedTrunkController.h"

#include <qpOASES.hpp>

namespace legged {

vector_t WeightedTrunkController::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode, vector_t typeFlag) {
  TrunkControllerBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, typeFlag);

  // Constraints
  Task constraints = formulateConstraints(stateDesired);
  size_t numConstraints = constraints.b_.size() + constraints.f_.size();

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
  vector_t lbA(numConstraints), ubA(numConstraints);  // clang-format off
  A << constraints.a_,
       constraints.d_;

  lbA << constraints.b_,
         -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
  ubA << constraints.b_,
         constraints.f_;  // clang-format on

  // Cost
  Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired);
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H = weighedTask.a_.transpose() * weighedTask.a_;
  vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

  // Solve
  auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
  qpOASES::Options options;
  options.setToMPC();
  options.printLevel = qpOASES::PL_LOW;
  options.enableEqualities = qpOASES::BT_TRUE;
  qpProblem.setOptions(options);
  int nWsr = 20;

  qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
  vector_t qpSol(getNumDecisionVars());

  qpProblem.getPrimalSolution(qpSol.data());
  return qpSol;
}

Task WeightedTrunkController::formulateConstraints(const vector_t& stateDesired) {
  return formulateFloatingBaseEomTask() + formulateFrictionConeTask() + formulateNoContactMotionTask() + formulateTorqueLimitsTask();
}

Task WeightedTrunkController::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired) {
  return formulateSwingLegTask(stateDesired) * weightSwingLeg_ + formulateBaseAccelTask(stateDesired) * weightBaseAccel_ +
         formulateContactForceTask(inputDesired) * weightContactForce_ + formulateRollingTask(stateDesired);
}

void WeightedTrunkController::loadTasksSetting(const std::string& taskFile, bool verbose) {
  TrunkControllerBase::loadTasksSetting(taskFile, verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "weight.";
  if (verbose) {
    std::cerr << "\n #### TrunkController weight:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
  loadData::loadPtreeValue(pt, weightRollingLeg_, prefix + "rollingLeg", verbose);
  loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
  loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
}

}  // namespace legged
