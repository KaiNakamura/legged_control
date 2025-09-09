//
// Created by qiayuan on 2022/7/1.
//
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_wbc/TrunkControllerBase.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>

// State vector takes form {q_b, q_j, lambda} 
namespace legged {
TrunkControllerBase::TrunkControllerBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info, const PinocchioEndEffectorKinematics& eeKinematics)
    : pinocchioInterfaceMeasured_(pinocchioInterface),
      pinocchioInterfaceDesired_(pinocchioInterface),
      info_(std::move(info)),
      mapping_(info_),
      inputLast_(vector_t::Zero(info_.inputDim)),
      eeKinematics_(eeKinematics.clone()) {
  numDecisionVars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
  qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
  vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
}

vector_t TrunkControllerBase::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode, vector_t typeFlag) {
  contactFlag_ = modeNumber2StanceLeg(mode);
  typeFlag_ = typeFlag;
  numContacts_ = 0;
  std::cout << "contact flag: ";
  for (bool flag : contactFlag_) {
    if (flag) {
      numContacts_++;
    }
    std::cout << flag << " ";
  }
  std::cout << std::endl;

  updateMeasured(rbdStateMeasured);

  return {};
}

void TrunkControllerBase::updateMeasured(const vector_t& rbdStateMeasured) {
  qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
  qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();
  qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
  vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
  vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
  vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

  const auto& model = pinocchioInterfaceMeasured_.getModel();
  auto& data = pinocchioInterfaceMeasured_.getData();

  // For floating base EoM task
  pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
  pinocchio::computeJointJacobians(model, data);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::crba(model, data, qMeasured_);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
  j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // For not contact motion task
  pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);
  dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
    jac.setZero(6, info_.generalizedCoordinatesNum);
    pinocchio::getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
    dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
  }

  // Stance jacobian
  jst_ = matrix_t::Zero(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      jst_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
    }
  }
}

Task TrunkControllerBase::formulateFloatingBaseEomTask() {
  auto& data = pinocchioInterfaceMeasured_.getData();

  matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
  s.block(0, 0, info_.actuatedDofNum, 6).setZero();
  s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();

  matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, numDecisionVars_) << data.M, -j_.transpose(), -s.transpose()).finished();
  vector_t b = -data.nle;

  return {a, b, matrix_t(), vector_t()};
}

Task TrunkControllerBase::formulateTorqueLimitsTask() {
  matrix_t d(2 * info_.actuatedDofNum, numDecisionVars_);
  d.setZero();
  matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
  d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum, info_.actuatedDofNum) = i;
  d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
          info_.actuatedDofNum) = -i;
  vector_t f(2 * info_.actuatedDofNum);
  for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l) {
    f.segment<3>(3 * l) = torqueLimits_;
  }

  return {matrix_t(), vector_t(), d, f};
}

Task TrunkControllerBase::formulateNoContactMotionTask() {
  matrix_t a(3 * numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if (contactFlag_[i]) {
      if(typeFlag_[i] == 0){
        a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
        b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      }
      else if(typeFlag_[i] == 1){
        a.block(3 * j + 1, 0, 2, info_.generalizedCoordinatesNum) = j_.block(3 * i + 1, 0, 2, info_.generalizedCoordinatesNum);
        b.segment(3 * j + 1, 2) = -dj_.block(3 * i + 1, 0, 2, info_.generalizedCoordinatesNum) * vMeasured_;
      }
      j++;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task TrunkControllerBase::formulateRollingTask(const vector_t& stateDesired) {
  matrix_t a(numContacts_, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;

  for (size_t i = 0; i < info_.numThreeDofContacts; i++) {
    if(contactFlag_[i] && typeFlag_[i] == 1){
      eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
      std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
      std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());

      vector_t posDesired = stateDesired.segment(18 + 3*i, 3);
      vector_t velDesired = stateDesired.segment(3, 3);

      vector_t relativePosDesired = qMeasured_.segment(0, 3) - (stateDesired.segment(0, 3) - stateDesired.segment(18 + 3*i, 3));
      vector_t relativeVelDesired = vMeasured_.segment(0, 3);

      double accel = rollingKp_*(posDesired(0) - posMeasured[i](0)) + rollingKd_*(relativeVelDesired(0) - velMeasured[i](0));

      a.block(j, 0, 1, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 1, info_.generalizedCoordinatesNum);
      b.segment(j, 1) = -dj_.block(3 * i, 0, 1, info_.generalizedCoordinatesNum) * vMeasured_ + (vector_t(1) << accel).finished();
      j++;

      // std::cout << "error here!!!!" << std::endl;
      std::cout << i << " roll accel: " << accel << " dpos: " << posDesired(0) << " mpos: " << posMeasured[i](0) << " dvel: " << vMeasured_(0) << " mvel: " << velMeasured[i](0) << std::endl;
    }
  }

  // std::cout << "rolling matrix a: " << a << std::endl;
  // std::cout << "rolling matrix b: " << b << std::endl;
  return {a, b, matrix_t(), vector_t()};
}
// From https://arxiv.org/pdf/1904.04595
Task TrunkControllerBase::formulateBaseAccelTask(const vector_t& stateDesired) {
  auto& data = pinocchioInterfaceMeasured_.getData();

  vector_t linPosDesired = stateDesired.segment(0, 3);
  vector_t linVelDesired = stateDesired.segment(3, 3);
  vector_t linAccDesired = stateDesired.segment(6, 3);

  vector_t angPosDesired = stateDesired.segment(9, 3);
  vector_t angVelDesired = stateDesired.segment(12, 3);
  vector_t angAccDesired = stateDesired.segment(15, 3);

  vector_t linPosMeasured = qMeasured_.segment(0, 3);
  vector_t linVelMeasured = vMeasured_.segment(0, 3);

  angPosDesired(0) = -angPosDesired(0);
  angVelDesired(0) = -angVelDesired(0);
  angAccDesired(0) = -angAccDesired(0);

  vector_t angPosMeasured = qMeasured_.segment(3, 3);
  vector_t angVelMeasured = vMeasured_.segment(3, 3);

  matrix_t I = matrix_t::Identity(3, 3);

  std::cout << "ad: " << linAccDesired.transpose() << " pd: " << linPosDesired.transpose() << " pm: " << linPosMeasured.transpose() << " vd: " << linVelDesired.transpose() << " vm: " << linVelMeasured.transpose() << std::endl;
  std::cout << "odd: " << angAccDesired.transpose() << " td: " << angPosDesired.transpose() << " tm: " << angPosMeasured.transpose() << " od: " << angVelDesired.transpose() << " om: " << angVelMeasured.transpose() << std::endl;

  matrix_t angStanceKp = (matrix_t(3,3) << angStanceKp_ , 0, 0,
                                            0, angStanceKp_, 0,
                                            0, 0, angStanceKp_).finished();
  matrix_t linStanceKp = (matrix_t(3,3) << linStanceKp_ , 0, 0,
                                            0, linStanceKp_, 0,
                                            0, 0, linStanceKp_).finished();
  matrix_t angStanceKd = (matrix_t(3,3) << angStanceKd_ , 0, 0,
                                            0, angStanceKd_, 0,
                                            0, 0, angStanceKd_).finished();
  matrix_t linStanceKd = (matrix_t(3,3) << linStanceKd_ , 0, 0,
                                            0, linStanceKd_, 0,
                                            0, 0, linStanceKd_).finished();

  vector_t ddotRRef = linAccDesired + linStanceKp * (linPosDesired - linPosMeasured) + linStanceKd * (linVelDesired - linVelMeasured);
  vector_t dotOmegaRef = angAccDesired + angStanceKp * (angPosDesired - angPosMeasured) + angStanceKd * (angVelDesired - angVelMeasured); //Note no omegas here it's all derivatives of euler angles

  vector_t ref = vector_t(6);
  ref << ddotRRef, dotOmegaRef;
  std::cout << "Ref: " << ref.transpose() << std::endl;

  matrix_t G = matrix_t::Zero(6, numDecisionVars_);
  vector_t g = vector_t(G.rows());
  matrix_t I6 = matrix_t::Identity(6, 6);

  G.block(0, 0, 6, 6) = I6;
  g = ref;

  G.row(2) *= 2;
  g(2) *= 2;

  G.row(1) *= 2;
  g(1) *= 2;

  G.row(0) *= 0.8;  
  g(0) *= 0.8;

  return {G, g, matrix_t(), vector_t()};
}

Task TrunkControllerBase::formulateFrictionConeTask() {
  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  a.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
    }
  }
  vector_t b(a.rows());
  b.setZero();

  matrix_t frictionPyramic(5, 3);  // clang-format off
  frictionPyramic << 0, 0, -1,
                     1, 0, -frictionCoeff_,
                    -1, 0, -frictionCoeff_,
                     0, 1, -frictionCoeff_,
                     0,-1, -frictionCoeff_;  // clang-format on

  matrix_t frictionPyramicWheel(5, 3);  // clang-format off
  frictionPyramicWheel << 0, 0, -1,
                          1, 0, -frictionWheelCoeff_,
                          -1, 0, -frictionWheelCoeff_,
                          0, 1, -frictionCoeff_,
                          0,-1, -frictionCoeff_;  // clang-format on

  matrix_t d(5 * numContacts_ + 3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  d.setZero();
  j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (contactFlag_[i]) {
      if(typeFlag_[i] == 0){
        d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
      }
      else{
        d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramicWheel;
      }
    }
  }
  vector_t f = Eigen::VectorXd::Zero(d.rows());

  return {a, b, d, f};
}


Task TrunkControllerBase::formulateSwingLegTask(const vector_t& stateDesired) {
  eeKinematics_->setPinocchioInterface(pinocchioInterfaceMeasured_);
  std::vector<vector3_t> posMeasured = eeKinematics_->getPosition(vector_t());
  std::vector<vector3_t> velMeasured = eeKinematics_->getVelocity(vector_t(), vector_t());
  for(int i = 0; i < info_.numThreeDofContacts; i++){
    posMeasured[i](2) -= 0.02;
  }

  vector_t posDesired = stateDesired.segment(18, 3*info_.numThreeDofContacts);
  vector_t velDesired = stateDesired.segment(18 + 3*info_.numThreeDofContacts, 3*info_.numThreeDofContacts);

  // std::cout << "ees desired: " << posDesired.transpose() << std::endl; 
  // std::cout << "ees measured: " << posMeasured[0].transpose() << posMeasured[1].transpose() << posMeasured[2].transpose() << posMeasured[3].transpose() << std::endl; 
  // std::cout << "evs desired: " << velDesired.transpose() << std::endl;
  // std::cout << "evs measured: " << velMeasured[0].transpose() << velMeasured[1].transpose() << velMeasured[2].transpose() << velMeasured[3].transpose() << std::endl; 

  matrix_t a(3 * (info_.numThreeDofContacts - numContacts_), numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();
  b.setZero();
  size_t j = 0;
  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    if (!contactFlag_[i]) {
      matrix3_t kp(3, 3); matrix3_t kd(3, 3);
      kp.setZero(); kd.setZero();
      kp(0,0) = swingKp_; kp(1,1) = swingKp_; kp(2,2) = 5*swingKp_; 
      kd(0,0) = swingKd_; kd(1,1) = swingKd_; kd(2,2) = 8*swingKd_; 

      vector3_t accel = kp*(posDesired.segment<3>(3*i) - posMeasured[i]) + kd*(velDesired.segment<3>(3*i) - velMeasured[i]);
      a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum);
      b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * vMeasured_;
      j++;

      std::cout << i << "th position: " << posMeasured[i].transpose() << std::endl;
      std::cout << i << "th vel: " << velMeasured[i].transpose() << std::endl;
      std::cout << i << "th desired: " << posDesired.segment<3>(3*i).transpose() << std::endl;
      std::cout << i << "th desired vel: " << velDesired.segment<3>(3*i).transpose() << std::endl;
      std::cout << i << "th leg acceleration: " << accel.transpose() << std::endl;
    }
  }

  return {a, b, matrix_t(), vector_t()};
}

Task TrunkControllerBase::formulateContactForceTask(const vector_t& inputDesired) const {
  matrix_t a(3 * info_.numThreeDofContacts, numDecisionVars_);
  vector_t b(a.rows());
  a.setZero();

  for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
    a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
  }
  b = inputDesired.head(a.rows());

  return {a, b, matrix_t(), vector_t()};
}

void TrunkControllerBase::loadTasksSetting(const std::string& taskFile, bool verbose) {
  // Load task file
  torqueLimits_ = vector_t(info_.actuatedDofNum / 4);
  loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torqueLimits_);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::string prefix = "frictionConeTask.";
  if (verbose) {
    std::cerr << "\n #### Friction Cone Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoeff_, prefix + "frictionCoefficient", verbose);
  loadData::loadPtreeValue(pt, frictionWheelCoeff_, prefix + "frictionWheelCoefficient", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  prefix = "swingLegTask.";
  if (verbose) {
    std::cerr << "\n #### Swing Leg Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, swingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, swingKd_, prefix + "kd", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  prefix = "rollingTask.";
  if (verbose) {
    std::cerr << "\n #### Rolling Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, rollingKp_, prefix + "kp", verbose);
  loadData::loadPtreeValue(pt, rollingKd_, prefix + "kd", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  prefix = "movementTask.";
  if (verbose) {
    std::cerr << "\n #### Movement Task:";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, linStanceKp_, prefix + "linkp", verbose);
  loadData::loadPtreeValue(pt, linStanceKd_, prefix + "linkd", verbose);
  loadData::loadPtreeValue(pt, angStanceKp_, prefix + "angkp", verbose);
  loadData::loadPtreeValue(pt, angStanceKd_, prefix + "angkd", verbose);
  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }
}

}  // namespace legged
