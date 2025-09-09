//
// Created by qiayuan on 2022/6/24.
//

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "legged_controllers/LeggedController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_legged_robot_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>

#include <angles/angles.h>
#include <legged_estimation/FromTopiceEstimate.h>
#include <legged_estimation/LinearKalmanFilter.h>
#include <legged_wbc/HierarchicalWbc.h>
#include <legged_wbc/WeightedWbc.h>
#include <legged_wbc/WeightedTrunkController.h>
#include <legged_wbc/HierarchicalTrunkController.h>
#include <pluginlib/class_list_macros.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace legged {
bool LeggedController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) {
  // Initialize OCS2
  std::string urdfFile;
  std::string taskFile;
  std::string referenceFile;
  controller_nh.getParam("/urdfFile", urdfFile);
  controller_nh.getParam("/taskFile", taskFile);
  controller_nh.getParam("/referenceFile", referenceFile);

  controller_nh.getParam("/eePos", eePosFile);
  controller_nh.getParam("/eeRegions", eeRegionsFile);
  controller_nh.getParam("/eeTypes", eeTypesFile);
  controller_nh.getParam("/forces", forcesFile);
  controller_nh.getParam("/states", statesFile);
  controller_nh.getParam("/swings", swingsFile);
  controller_nh.getParam("/time", timeFile);

  bool verbose = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  setupLeggedInterface(taskFile, urdfFile, referenceFile, verbose);
  setupMpc();
  setupMrt();
  // Visualization
  ros::NodeHandle nh;
  CentroidalModelPinocchioMapping pinocchioMapping(leggedInterface_->getCentroidalModelInfo());
  eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(leggedInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                      leggedInterface_->modelSettings().contactNames3DoF);
  robotVisualizer_ = std::make_shared<LeggedRobotVisualizer>(leggedInterface_->getPinocchioInterface(),
                                                             leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, nh);
  selfCollisionVisualization_.reset(new LeggedSelfCollisionVisualization(leggedInterface_->getPinocchioInterface(),
                                                                         leggedInterface_->getGeometryInterface(), pinocchioMapping, nh));

  // Hardware interface
  auto* hybridJointInterface = robot_hw->get<HybridJointInterface>();
  std::vector<std::string> joint_names{"LF_HAA", "LF_HFE", "LF_KFE", "LH_HAA", "LH_HFE", "LH_KFE",
                                       "RF_HAA", "RF_HFE", "RF_KFE", "RH_HAA", "RH_HFE", "RH_KFE"};
  for (const auto& joint_name : joint_names) {
    hybridJointHandles_.push_back(hybridJointInterface->getHandle(joint_name));
  }

  for (const auto& switcher_name : switcher_names) {
    switcherHandles_.push_back(hybridJointInterface->getHandle(switcher_name));
  }

  for (const auto& roller_name : roller_names) {
    rollerHandles_.push_back(hybridJointInterface->getHandle(roller_name));
  }

  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
    for (const auto& name : wheel_names) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }

  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  tc_ = std::make_shared<WeightedTrunkController>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  tc_->loadTasksSetting(taskFile, verbose);


  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());
  CSVFormat = Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ",", "", ",");

  outputFile.open("solution.csv", std::ios::out);
  if (!outputFile.is_open()) {
    std::cerr << "Error opening file!" << std::endl;
  }
  std::cout << "output file open: " << outputFile.is_open() << std::endl;
  outputFile << "Time,x,y,z,vx,vy,vz,tx,ty,tz,ox,oy,oz,dx,dy,dz,dvx,dvy,dvz,dax,day,daz,dtx,dty,dtz,dox,doy,doz,dodx,dody,dodz,f1x,f1y,f1z,f2x,f2y,f2z,f3x,f3y,f3z,f4x,f4y,f4z,df1x,df1y,df1z,df2x,df2y,df2z,df3x,df3y,df3z,df4x,df4y,df4z,ee1x,ee1y,ee1z,ee2x,ee2y,ee2z,ee3x,ee3y,ee3z,ee4x,ee4y,ee4z,dee1x,dee1y,dee1z,dee2x,dee2y,dee2z,dee3x,dee3y,dee3z,dee4x,dee4y,dee4z,cf1,cf2,cf3,cf4,cw1,cw2,cw3,cw4,dee1vx,dee1vy,dee1vz,dee2vx,dee2vy,dee2vz,dee3vx,dee3vy,dee3vz,dee4vx,dee4vy,dee4vz,ee1vx,ee1vy,ee1vz,ee2vx,ee2vy,ee2vz,ee3vx,ee3vy,ee3vz,ee4vx,ee4vy,ee4vz,\n";

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
  stateEstimate_->updateType((vector_t(4) << 0, 0, 0, 0).finished());
  updateStateEstimation(time, ros::Duration(0.002));
  currentObservation_.input.setZero(leggedInterface_->getCentroidalModelInfo().inputDim);
  currentObservation_.mode = ModeNumber::STANCE;
  updatedMode = currentObservation_.mode;

  TargetTrajectories target_trajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});

  // Set the first observation and command and wait for optimization to finish
  mpcMrtInterface_->setCurrentObservation(currentObservation_);
  mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
  ROS_INFO_STREAM("Waiting for the initial policy ...");
  while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok()) {
    mpcMrtInterface_->advanceMpc();
    ros::WallRate(leggedInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
  }
  ROS_INFO_STREAM("Initial policy has been received.");

  mpcRunning_ = true;

}

void LeggedController::update(const ros::Time& time, const ros::Duration& period) {
  // State Estimate
  updateStateEstimation(time, period);

  vector_t torque;
  vector_t optimizedState, optimizedInput;
  vector_t x;

  vector_t posDes, velDes;

  ros::Duration elapsedTime = time - mipStartTime;
  std::cout << "Elapsed: " << elapsedTime.sec + elapsedTime.nsec/1.0e9 << std::endl;
  // std::cout << "Mip activated:" << mipActivated << std::endl;

  if(!mipActivated || elapsedTime.sec + elapsedTime.nsec/1.0e9 <= 0.03){

    if(measuredRbdState_(5) > 0.15){
      currentObservation_.mode = updatedMode;
    }

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

    wbcTimer_.startTimer();
    if(measuredRbdState_(5) > 0.15){
      x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, updatedMode, period.toSec());
    }
    else{
      x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    }
    wbcTimer_.endTimer();

    CentroidalModelInfo info = leggedInterface_->getCentroidalModelInfo();
    eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

    vector_t qMeasured = vector_t(info.generalizedCoordinatesNum);
    vector_t vMeasured = vector_t(info.generalizedCoordinatesNum);

    qMeasured.head<3>() = measuredRbdState_.segment<3>(3);
    qMeasured.segment<3>(3) = measuredRbdState_.head<3>();
    qMeasured.tail(info.actuatedDofNum) = measuredRbdState_.segment(6, info.actuatedDofNum);
    vMeasured.head<3>() = measuredRbdState_.segment<3>(info.generalizedCoordinatesNum + 3);
    vMeasured.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured.segment<3>(3), measuredRbdState_.segment<3>(info.generalizedCoordinatesNum));
    vMeasured.tail(info.actuatedDofNum) = measuredRbdState_.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

    const auto& model = leggedInterface_->getPinocchioInterface().getModel();
    auto& data = leggedInterface_->getPinocchioInterface().getData();

    pinocchio::forwardKinematics(model, data, qMeasured, vMeasured);
    pinocchio::updateFramePlacements(model, data);
    std::vector<vector3_t> footPos = eeKinematicsPtr_->getPosition(vector_t());
    for(int i = 0; i < info.numThreeDofContacts; i++){
      footPos[i](2) -= 0.02;
    }

    // std::cout << "optimized state: " << optimizedState.transpose() << std::endl;
    // std::cout << "optimized input: " << optimizedInput.transpose() << std::endl;
    // std::cout << "x: " << x.transpose() << std::endl;
    // std::cout << "measured state: " << measuredRbdState_.segment<3>(3).transpose() << measuredRbdState_.head<3>().transpose() << footPos[0].transpose() << footPos[1].transpose() << footPos[2].transpose() << footPos[3].transpose() << std::endl;

    torque = x.tail(12);

    posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());
    // std::cout << "torque: " << torque.transpose() << std::endl;

    for(int i = 0; i < switcherHandles_.size(); i++){
      switcherHandles_[i].setCommand(0.05, 0, 10000, 0, 0);
    }
  }
  else{    
    double knotTime = times[1] - times[0];

    int idx = times.size()-2;
    if(elapsedTime.sec + elapsedTime.nsec/1.0e9 < times[times.size()-2]){
      idx = (int)((elapsedTime.sec + elapsedTime.nsec/1.0e9)/knotTime);
    }    
    int swingIdx = idx / (times.size()/swings[0].size());

    // std::cout << "Idx: " << idx << std::endl;
    // std::cout << "Swing Idx: " << swingIdx << std::endl;

    CentroidalModelInfo info = leggedInterface_->getCentroidalModelInfo();
    eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

    optimizedState = Eigen::VectorXd::Zero(18 + 3*info.numThreeDofContacts + 3*info.numThreeDofContacts);
    optimizedInput = Eigen::VectorXd::Zero(3*info.numThreeDofContacts);

    // Load csv data into optimized state vector
    // Change from xyz format to zyx
    Eigen::VectorXd hqb = Eigen::VectorXd::Zero(info.generalizedCoordinatesNum);
    hqb << states[0][idx]+0.00, states[1][0], states[2][idx], states[3][idx], 0, states[5][idx], states[6][idx], 0, states[8][idx], 
            0, 0, 0, 0, 0, 0, 0, 0, 0;
    // hqb << states[0][0], states[1][0], 0.3, states[3][0], states[4][0], states[5][0], states[6][0], states[7][0], states[8][0], 
    //         states[11][0], states[10][0], states[9][0], states[14][0], states[13][0], states[12][0], states[17][0], states[16][0], states[15][0];

    Eigen::VectorXd hqbNext = Eigen::VectorXd::Zero(info.generalizedCoordinatesNum);
    hqbNext << states[0][idx + 1]+0.00, states[1][0], states[2][idx + 1], states[3][idx + 1], 0, states[5][idx + 1], states[6][idx + 1], 0, states[8][idx + 1], 
            0, 0, 0, 0, 0, 0, 0, 0, 0;

    double swingTime = times[times.size() - 1]/swings[0].size();
    double swingHeight = 0.05;
    double t = (elapsedTime.sec + elapsedTime.nsec/1.0e9 - 0.006)/swingTime - (int)((elapsedTime.sec + elapsedTime.nsec/1.0e9 - 0.006)/swingTime);

    for(int i = 0; i < 18; i++){
      hqb(i) = (1-t)*hqb(i) + t*hqbNext(i);
    }
    hqb(1) = 0.9-hqb(1); hqb(4) = -hqb(4); hqb(7) = -hqb(7);
    hqb(11) = -hqb(11); hqb(14) = -hqb(14); hqb(17) = -hqb(17);
    optimizedState.segment<18>(0) = hqb;

    int swapIdx[info.numThreeDofContacts] {2, 0, 1, 3};
    size_t currMode = 0;
    for(int i = 0; i < info.numThreeDofContacts; i++){
      if(abs(swings[i][swingIdx]) <= 0.001){
        currMode += (int) pow(2, info.numThreeDofContacts-1-swapIdx[i]);
      }
    }

    vector_t types = vector_t(info.numThreeDofContacts);
    std::cout << "switcher goal poses: ";
    for(int i = 0; i < switcherHandles_.size(); i++){
      types(swapIdx[i]) = eeTypes[i][swingIdx];
      double goalPos = switcherUpper;

      if(swingIdx >= 1){
        bool typeConstant = eeTypes[i][swingIdx] == eeTypes[i][swingIdx - 1];
        // std::cout << "Type constant: " << typeConstant << " t: " << t << " type: " << eeTypes[swapIdx[i]][swingIdx] << std::endl;
        if(typeConstant || t >= 0.9){
          if(types(swapIdx[i]) == 0){
            goalPos = switcherUpper;
          }
          else if(types(swapIdx[i]) == 1){
            goalPos = switcherLower;
          }
        }
        else{
          if(types(swapIdx[i]) == 0){
            goalPos = switcherLower;
          }
          else if(types(swapIdx[i]) == 1){
            goalPos = switcherUpper;
          }
        }
        std::cout << goalPos << " ";
        switcherHandles_[swapIdx[i]].setCommand(goalPos, 0, 17500, 0, 0);
      }
      else{
        if(types(swapIdx[i]) == 0){
          goalPos = switcherUpper;
        }
        else if(types(swapIdx[i]) == 1){
          goalPos = switcherLower;
        }
        std::cout << goalPos << " ";

        switcherHandles_[swapIdx[i]].setCommand(goalPos, 0, 10000, 0, 0);
      } 
    }
    std::cout << std::endl;
    // currMode = 15;
    vector_t qMeasured = vector_t(info.generalizedCoordinatesNum);
    vector_t vMeasured = vector_t(info.generalizedCoordinatesNum);

    const auto& model = leggedInterface_->getPinocchioInterface().getModel();
    auto& data = leggedInterface_->getPinocchioInterface().getData();

    qMeasured.head<3>() = measuredRbdState_.segment<3>(3);
    qMeasured.segment<3>(3) = measuredRbdState_.head<3>();
    qMeasured.tail(info.actuatedDofNum) = measuredRbdState_.segment(6, info.actuatedDofNum);
    vMeasured.head<3>() = measuredRbdState_.segment<3>(info.generalizedCoordinatesNum + 3);
    vMeasured.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
      qMeasured.segment<3>(3), measuredRbdState_.segment<3>(info.generalizedCoordinatesNum));
    vMeasured.tail(info.actuatedDofNum) = measuredRbdState_.segment(info.generalizedCoordinatesNum + 6, info.actuatedDofNum);

    pinocchio::forwardKinematics(model, data, qMeasured, vMeasured);
    pinocchio::updateFramePlacements(model, data);
    std::vector<vector3_t> footPos = eeKinematicsPtr_->getPosition(vector_t());
    std::vector<vector3_t> footVel = eeKinematicsPtr_->getVelocity(vector_t(), vector_t());

    for(int i = 0; i < info.numThreeDofContacts; i++){
      footPos[i](2) -= 0.02;
    }

    Eigen::VectorXd ees = Eigen::VectorXd::Zero(3*info.numThreeDofContacts);
    Eigen::VectorXd evs = Eigen::VectorXd::Zero(3*info.numThreeDofContacts);

    for(int i = 0; i < info.numThreeDofContacts; i++){
      vector_t footstep(3);
      vector_t evel(3);
      vector_t eVec(3);
      eVec << eePos[0 + 3*i][idx], eePos[1 + 3*i][idx], eePos[2 + 3*i][idx];

      int liftIdx = swingIdx * times.size() / swings[0].size();
      int landIdx = (swingIdx + 1) * times.size() / swings[0].size() - 1;

      vector_t eVecLift(3);
      vector_t eVecLand(3);
      eVecLift << eePos[0 + 3*i][liftIdx], eePos[1 + 3*i][liftIdx], eePos[2 + 3*i][liftIdx];
      eVecLand << eePos[0 + 3*i][landIdx], eePos[1 + 3*i][landIdx], eePos[2 + 3*i][landIdx];

      vector_t relativeEEPos(3);
      vector_t relativeEELift(3);
      vector_t relativeEELand(3);
      if(abs(optimizedState(0) - qMeasured(0)) > 0.3 || types(swapIdx[i]) == 1){
        relativeEEPos = qMeasured.segment(0, 3) - (optimizedState.segment(0, 3) - eVec);
        relativeEELift = qMeasured.segment(0, 3) - (optimizedState.segment(0, 3) - eVecLift);
        relativeEELand = qMeasured.segment(0, 3) - (optimizedState.segment(0, 3) - eVecLand);
      }
      else{
        relativeEEPos = eVec;
        relativeEELift = eVecLift;
        relativeEELand = eVecLand;
      }

      if(abs(swings[i][swingIdx]) <= 0.001){
        footstep = relativeEEPos;
        evel << 0, 0, 0;
      }
      else{
        vector_t P1 = vector_t(3); vector_t P2 = vector_t(3); vector_t P3 = vector_t(3); vector_t V1 = vector_t(3); vector_t V3 = vector_t(3); vector_t A1 = vector_t(3); vector_t A3 = vector_t(3); 
        matrix_t Mq = (matrix_t(6,6) << 1, 0, 0, 0, 0, 0,
                                        0, 0, 1, 0, 0, 0,
                                        0, 0, 0, 0, 0.5, 0,
                                        -10, 10, -6, -4, -1.5, 0.5,
                                        15, -15, 8, 7, 1.5, -1,
                                        -6, 6, -3, -3, -0.5, 0.5).finished();
        matrix_t Ms = (matrix_t(8,3) << 1, 0, 0,
                                        0, 0, 0,
                                        0, 0, 0,
                                        -102, 64, 38,
                                        411, -192, -219,
                                        -642, 192, 450,
                                        452, -64, -388,
                                        -120, 0, 120).finished();

        P1 << relativeEELift;
        P3 << relativeEELand;     
        P2 << (P1(0) + P3(0))/2.0, (P1(1) + P3(1))/2.0, std::max(P1(2), P2(2)) + swingHeight;

        V1 << (eePos[0 + 3*i][liftIdx]-eePos[0 + 3*i][liftIdx-1])/knotTime, (eePos[1 + 3*i][liftIdx]-eePos[1 + 3*i][liftIdx-1])/knotTime, (eePos[2 + 3*i][liftIdx]-eePos[2 + 3*i][liftIdx-1])/knotTime;
        if(eeTypes[i][swingIdx] == 0){
          V3 << 0, 0, 0;
        }
        else {
          V3 << states[3][landIdx], 0, 0;
        }
        A1 << 0, 0, 0;
        A3 << 0, 0, 0;

        // std::cout << "leg " << i << " type " << eeTypes[i][swingIdx] << " v: " << V3.transpose() << std::endl;

        vector_t vx = (vector_t(6) << P1(0), P3(0), V1(0), V3(0), A1(0), A3(0)).finished();
        vector_t vy = (vector_t(6) << P1(1), P3(1), V1(1), V3(1), A1(1), A3(1)).finished();
        vector_t vz = (vector_t(3) << P1(2), P2(2), P3(2)).finished();

        // std::cout << "vx: " << vx.transpose() << std::endl;
        // std::cout << "vy: " << vy.transpose() << std::endl;
        // std::cout << "vz: " << vz.transpose() << std::endl;

        vector_t px = Mq * vx;
        vector_t py = Mq * vy;
        vector_t pz = Ms * vz;

        // std::cout << "px: " << px.transpose() << std::endl;
        // std::cout << "py: " << py.transpose() << std::endl;
        // std::cout << "pz: " << pz.transpose() << std::endl;

        footstep(0) = px(0) + px(1)*t + px(2)*pow(t,2) + px(3)*pow(t,3) + px(4)*pow(t,4) + px(5)*pow(t,5);
        footstep(1) = py(0) + py(1)*t + py(2)*pow(t,2) + py(3)*pow(t,3) + py(4)*pow(t,4) + py(5)*pow(t,5);
        footstep(2) = pz(0) + pz(1)*t + pz(2)*pow(t,2) + pz(3)*pow(t,3) + pz(4)*pow(t,4) + pz(5)*pow(t,5) + pz(6)*pow(t,6) + pz(7)*pow(t,7);

        // std::cout << "liftIdx: " << liftIdx << " landIdx: " << landIdx << " P1: " << P1.transpose() << " P3: " << P3.transpose() << " P2: " << P2.transpose() << " footstep: " << footstep.transpose() << std::endl;
        // std::cout << " V1: " << V1.transpose() << " V3: " << V3.transpose() << " A1: " << A1.transpose() << " A3: " << A3.transpose() << std::endl;

        evel(0) = px(1) + 2*px(2)*pow(t,1) + 3*px(3)*pow(t,2) + 4*px(4)*pow(t,3) + 5*px(5)*pow(t,4);
        evel(1) = py(1) + 2*py(2)*pow(t,1) + 3*py(3)*pow(t,2) + 4*py(4)*pow(t,3) + 5*py(5)*pow(t,4);
        evel(2) = pz(1) + 2*pz(2)*pow(t,1) + 3*pz(3)*pow(t,2) + 4*pz(4)*pow(t,3) + 5*pz(5)*pow(t,4) + 6*pz(6)*pow(t,5) + 7*pz(7)*pow(t,6);

        // std::cout << "evel: " << evel.transpose() << std::endl;
      }
      // footstep << eePos[0 + 3*i][idx], eePos[1 + 3*i][idx], eePos[2 + 3*i][idx];
      footstep[1] = 0.9 - footstep[1];
      evel[1] = -evel[1];

      ees.segment<3>(3*swapIdx[i]) = footstep;
      evs.segment<3>(3*swapIdx[i]) = evel;
      
    }
    optimizedState.segment(18, 3*info.numThreeDofContacts) = ees;
    optimizedState.segment(18 + 3*info.numThreeDofContacts, 3*info.numThreeDofContacts) = evs;

    for(int i = 0; i < 3*info.numThreeDofContacts; i++){
      optimizedInput(i) = forces[3*swapIdx[i/3] + i%3][idx];
      // optimizedInput(i) = forces[i][0];
    }

    stateEstimate_->updateType(types);

    x = tc_->update(optimizedState, optimizedInput, measuredRbdState_, currMode, types);

    torque = x.tail(info.actuatedDofNum);

    outputFile << elapsedTime.sec + elapsedTime.nsec/1.0e9 << ",";
    outputFile << qMeasured.head<3>().transpose().format(CSVFormat) << vMeasured.head<3>().transpose().format(CSVFormat) << qMeasured.segment<3>(3).transpose().format(CSVFormat) << vMeasured.segment<3>(3).transpose().format(CSVFormat);
    outputFile << hqb.transpose().format(CSVFormat);
    outputFile << x.segment(info.generalizedCoordinatesNum, 3*info.numThreeDofContacts).transpose().format(CSVFormat);
    outputFile << optimizedInput.transpose().format(CSVFormat);
    outputFile << footPos[0].transpose().format(CSVFormat) << footPos[1].transpose().format(CSVFormat) << footPos[2].transpose().format(CSVFormat) << footPos[3].transpose().format(CSVFormat);
    outputFile << optimizedState.segment(18, 3*info.numThreeDofContacts).transpose().format(CSVFormat);

    for(int i = 0; i < contactHandles_.size(); i++){
      if(i<4){
        outputFile << contactHandles_[i].isContact() << ",";
      }
      else{
        outputFile << contactHandles_[i].isContact() << ",";
      }
    }

    outputFile << optimizedState.segment(18 + 3*info.numThreeDofContacts, 3*info.numThreeDofContacts).transpose().format(CSVFormat);
    outputFile << footVel[0].transpose().format(CSVFormat) << footVel[1].transpose().format(CSVFormat) << footVel[2].transpose().format(CSVFormat) << footVel[3].transpose().format(CSVFormat);
    outputFile <<"\n";
    
    // std::cout << "optimized state: " << optimizedState.transpose() << std::endl;
    // std::cout << "measured state: " << measuredRbdState_.segment<3>(3).transpose() << measuredRbdState_.head<3>().transpose() << footPos[0].transpose() << footPos[1].transpose() << footPos[2].transpose() << footPos[3].transpose() << std::endl;
    // std::cout << "optimized input: " << optimizedInput.transpose() << std::endl;
    // std::cout << "x: " << x.transpose() << std::endl;
    std::cout << "forces: " << x.segment(info.generalizedCoordinatesNum, 3*info.numThreeDofContacts).transpose() << std::endl;
    
    std::cout << "optimized feet: " << optimizedState.segment(18, 3*info.numThreeDofContacts).transpose() << std::endl;
    std::cout << "measured feet: " << footPos[0].transpose() << footPos[1].transpose() << footPos[2].transpose() << footPos[3].transpose() << std::endl;
    // std::cout << "foot vel: " << footVel[0].transpose() << footVel[1].transpose() << footVel[2].transpose() << footVel[3].transpose() << std::endl;
    std::cout << "torque: " << torque.transpose() << std::endl;
    std::cout << "wheels: ";
    for(int i = 0; i < info.numThreeDofContacts; i++){
      std::cout << rollerHandles_[i].getPosition() << " ";
    }
    std::cout << std::endl;

    std::cout << "switchers: ";
    for(int i = 0; i < info.numThreeDofContacts; i++){
      std::cout << switcherHandles_[i].getPosition() << " ";
    }
    std::cout << std::endl;
    std::cout << "types: " << types.transpose() << std::endl;

    std::cout << "swings: ";
    for(int i = 0; i < info.numThreeDofContacts; i++){
      std::cout << swings[swapIdx[i]][swingIdx] << " ";
    }
    std::cout << std::endl;
    std::cout << "currMode: " << currMode << std::endl;

    std::cout << "Contacts: ";
    for(int i = 0; i < contactHandles_.size(); i++){
      std::cout << contactHandles_[i].isContact() << " ";
    }
    std::cout << std::endl;

    if (!outputFile.is_open()) {
      std::cerr << "Error opening file!" << std::endl;
      return; // Handle error appropriately
    }

    // if(elapsedTime.sec + elapsedTime.nsec/1.0e9 > 0.01){
    //   throw std::exception();
    // }

    // if(elapsedTime.sec + elapsedTime.nsec/1.0e9 > 1.208){
    //   throw std::exception();
    // }

    // if(elapsedTime.sec + elapsedTime.nsec/1.0e9 > 0.758){
    //   outputFile.close();
    //   throw std::exception();
    // }

    // if(elapsedTime.sec + elapsedTime.nsec/1.0e9 > 1.858){
    //   throw std::exception();
    // }

    // if(elapsedTime.sec + elapsedTime.nsec/1.0e9 > 2.48){
    //   throw std::exception();
    // }

    velDes = vMeasured.tail(info.actuatedDofNum) + (period.sec + period.nsec/1.0e9) * x.segment(6, info.actuatedDofNum);
    posDes = qMeasured.tail(info.actuatedDofNum) + (period.sec + period.nsec/1.0e9) * velDes;

    // // file pointer
    // std::fstream fout;
    // if(!fout){
    //   std::cout << "OPEN FAILED" << std::endl;
    // }
    // // opens an existing csv file or creates a new file.
    // fout.open("pos.csv", std::ios::out | std::ios::app);
    // for(int i = 0; i < 3; i++){
    //   fout << hqb(i) << "," << qMeasured(i) << "," << hqb(i + 3) << "," << vMeasured(i) << "," << hqb(i+9) << "," << qMeasured(i+3) << "," << hqb(i+12) << "," << vMeasured(i+3)<< "\n";
    // }
    // fout.close();
  }

  // std::cout << "PosDes: " << posDes.transpose() << std::endl;
  // std::cout << "velDes: " << velDes.transpose() << std::endl;

  updatedMode = contactEstimate_->update(currentObservation_.time, period, optimizedInput, measuredRbdState_, torque, contactFlag, mpcMrtInterface_->activePrimalSolutionPtr_->modeSchedule_);
  
  leg1_contact_force.data = contactFlag[0];
  leg2_contact_force.data = contactFlag[1];
  leg3_contact_force.data = contactFlag[2];
  leg4_contact_force.data = contactFlag[3];

  leg1_contact_force_pub.publish(leg1_contact_force);
  leg2_contact_force_pub.publish(leg2_contact_force);
  leg3_contact_force_pub.publish(leg3_contact_force);
  leg4_contact_force_pub.publish(leg4_contact_force);

  height.data = measuredRbdState_(5);
  height_pub.publish(height);

  // std::cout << "planned mode: " << plannedMode << std::endl;
  // std::cout << "force sensors mode: " << currentObservation_.mode << std::endl;
  // std::cout << "updated mode: " << updatedMode << std::endl;
  // std::cout << "actual mode: " << mpcMrtInterface_->activePrimalSolutionPtr_->modeSchedule_.modeAtTime(currentObservation_.time) << std::endl;

  // Safety check, if failed, stop the controller
  if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
    ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
    stopRequest(time);
  }

  for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
    hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 3, 3, torque(j));
  }

  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
  if (robotVisualizer_->robotStatePublisherPtr_ != nullptr) {
    // std::cout << "switcher Pos: ";
    std::map<std::string, scalar_t> jointPositions;
    for(int i = 0; i < switcher_names.size(); i++){
      // std::cout << " " << switcherHandles_[i].getPosition();
      jointPositions[switcher_names[i]] = switcherHandles_[i].getPosition();
    }
    for(int i = 0; i < roller_names.size(); i++){
      jointPositions[roller_names[i]] = rollerHandles_[i].getPosition();
      
    }
    // std::cout<<std::endl;
    robotVisualizer_->robotStatePublisherPtr_->publishTransforms(jointPositions, ros::Time::now());
  }
  selfCollisionVisualization_->update(currentObservation_);

  // Publish the observation. Only needed for the command interface
  observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
}

void LeggedController::updateStateEstimation(const ros::Time& time, const ros::Duration& period) {
  vector_t jointPos(hybridJointHandles_.size()), jointVel(hybridJointHandles_.size());
  contact_flag_t contacts;
  Eigen::Quaternion<scalar_t> quat;
  vector3_t angularVel, linearAccel;
  matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;

  for (size_t i = 0; i < hybridJointHandles_.size(); ++i) {
    jointPos(i) = hybridJointHandles_[i].getPosition();
    jointVel(i) = hybridJointHandles_[i].getVelocity();
  }
  for (size_t i = 0; i < contacts.size(); ++i) {
    contactFlag[i] = contactHandles_[i].isContact();
  }
  for (size_t i = 0; i < 4; ++i) {
    quat.coeffs()(i) = imuSensorHandle_.getOrientation()[i];
  }
  for (size_t i = 0; i < 3; ++i) {
    angularVel(i) = imuSensorHandle_.getAngularVelocity()[i];
    linearAccel(i) = imuSensorHandle_.getLinearAcceleration()[i];
  }
  for (size_t i = 0; i < 9; ++i) {
    orientationCovariance(i) = imuSensorHandle_.getOrientationCovariance()[i];
    angularVelCovariance(i) = imuSensorHandle_.getAngularVelocityCovariance()[i];
    linearAccelCovariance(i) = imuSensorHandle_.getLinearAccelerationCovariance()[i];
  }

  stateEstimate_->updateJointStates(jointPos, jointVel);
  stateEstimate_->updateContact(contactFlag);
  stateEstimate_->updateImu(quat, angularVel, linearAccel, orientationCovariance, angularVelCovariance, linearAccelCovariance);
  measuredRbdState_ = stateEstimate_->update(time, period);
  currentObservation_.time += period.toSec();
  scalar_t yawLast = currentObservation_.state(9);
  currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
  currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
  currentObservation_.mode = stateEstimate_->getMode();
}

LeggedController::~LeggedController() {
  controllerRunning_ = false;
  if (mpcThread_.joinable()) {
    mpcThread_.join();
  }
  std::cerr << "########################################################################";
  std::cerr << "\n### MPC Benchmarking";
  std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
  std::cerr << "########################################################################";
  std::cerr << "\n### WBC Benchmarking";
  std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
  std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
}

void LeggedController::setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                            bool verbose) {
  leggedInterface_ = std::make_shared<LeggedInterface>(taskFile, urdfFile, referenceFile);
  leggedInterface_->setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);
}

void LeggedController::setupMpc() {
  mpc_ = std::make_shared<SqpMpc>(leggedInterface_->mpcSettings(), leggedInterface_->sqpSettings(),
                                  leggedInterface_->getOptimalControlProblem(), leggedInterface_->getInitializer());
  rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(leggedInterface_->getPinocchioInterface(),
                                                                    leggedInterface_->getCentroidalModelInfo());

  const std::string robotName = "legged_robot";
  ros::NodeHandle nh;
  // Gait receiver
  auto gaitReceiverPtr =
      std::make_shared<GaitReceiver>(nh, leggedInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nh);
  mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
  mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
  observationPublisher_ = nh.advertise<ocs2_msgs::mpc_observation>(robotName + "_mpc_observation", 1);

  leg1_contact_force_pub = nh.advertise<std_msgs::Int16>("contact_estimation/leg1_contact_force", 10);
  leg2_contact_force_pub = nh.advertise<std_msgs::Int16>("contact_estimation/leg2_contact_force", 10);
  leg3_contact_force_pub = nh.advertise<std_msgs::Int16>("contact_estimation/leg3_contact_force", 10);
  leg4_contact_force_pub = nh.advertise<std_msgs::Int16>("contact_estimation/leg4_contact_force", 10);

  height_pub = nh.advertise<std_msgs::Float64>("contact_estimation/height", 10);
  mipActivationSub = nh.subscribe("activate_MIP_file", 1, &LeggedController::mipActivationCallback, this);
}

void LeggedController::setupMrt() {
  mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
  mpcMrtInterface_->initRollout(&leggedInterface_->getRollout());
  mpcTimer_.reset();

  controllerRunning_ = true;
  mpcThread_ = std::thread([&]() {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            leggedInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        stopRequest(ros::Time());
      }
    }
  });
  setThreadPriority(leggedInterface_->sqpSettings().threadPriority, mpcThread_);
}

void LeggedController::setupStateEstimate(const std::string& taskFile, bool verbose) {
  stateEstimate_ = std::make_shared<KalmanFilterEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  dynamic_cast<KalmanFilterEstimate&>(*stateEstimate_).loadSettings(taskFile, verbose);

  contactEstimate_ = std::make_shared<ContactEstimate>(leggedInterface_->getPinocchioInterface(),
                                                          leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
  currentObservation_.time = 0;
}

void LeggedCheaterController::setupStateEstimate(const std::string& /*taskFile*/, bool /*verbose*/) {
  stateEstimate_ = std::make_shared<FromTopicStateEstimate>(leggedInterface_->getPinocchioInterface(),
                                                            leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);

  contactEstimate_ = std::make_shared<ContactEstimate>(leggedInterface_->getPinocchioInterface(),
                                                        leggedInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
}

void LeggedController::mipActivationCallback(std_msgs::Time msg){
  mipActivated = true;
  mipStartTime = msg.data;

  readCSVDouble(eePosFile, eePos);
  readCSVDouble(forcesFile, forces);
  readCSVDouble(statesFile, states);
  readCSVDouble(timeFile, times);

  readCSVInt(eeRegionsFile, eeRegions);
  readCSVInt(eeTypesFile, eeTypes);
  readCSVInt(swingsFile, swings);
}

void LeggedController::readCSVDouble(std::string file, std::vector<std::vector<double>> &vec){
  std::ifstream fin;
  fin.open(file, std::ios::in);

  std::vector<double> row;
  std::string line, word, temp;

  if(!fin){
    std::cout << "OPEN FAILED" << std::endl;
  }
  // std::cout << "Reading csv at " << file << std::endl;
  while (getline(fin, line)){
    row.clear();
    std::stringstream s(line);
    // read every column data of a row and
    // store it in a string variable, 'word'
    while (getline(s, word, ','))
    {
        // add all the column data
        // of a row to a vector
        row.push_back(std::stod(word));
    }

    vec.push_back(row);
  }
  fin.close();
}

void LeggedController::readCSVInt(std::string file, std::vector<std::vector<int>> &vec){
  std::ifstream fin;
  fin.open(file, std::ios::in);

  std::vector<int> row;
  std::string line, word, temp;

  if(!fin){
    std::cout << "OPEN FAILED" << std::endl;
  }
  std::cout << "Reading csv at " << file << std::endl;
  while (getline(fin, line)){
    row.clear();
    std::stringstream s(line);
    // read every column data of a row and
    // store it in a string variable, 'word'
    while (getline(s, word, ','))
    {
        // add all the column data
        // of a row to a vector
        row.push_back((int) std::stod(word));
    }

    vec.push_back(row);
  }
  fin.close();
}

void LeggedController::readCSVDouble(std::string file, std::vector<double> &vec){
  std::ifstream fin;
  fin.open(file, std::ios::in);

  std::vector<double> row;
  std::string line, word, temp;

  if(!fin){
    std::cout << "OPEN FAILED" << std::endl;
  }
  std::cout << "Reading csv at " << file << std::endl;
  while (getline(fin, line)){
    std::stringstream s(line);
    // read every column data of a row and
    // store it in a string variable, 'word'
    while (getline(s, word, ','))
    {
        // add all the column data
        // of a row to a vector
        row.push_back(std::stod(word));
    }
  }

  vec = row;

  fin.close();
}

// Todo: Probably delete
Eigen::VectorXd LeggedController::ik(vector_t q0, int jointID, vector_t xd){
  CentroidalModelInfo info_ = leggedInterface_->getCentroidalModelInfo();

  const auto& model = leggedInterface_->getPinocchioInterface().getModel();
  auto& data = leggedInterface_->getPinocchioInterface().getData();

  const pinocchio::SE3 oMdes(Eigen::Matrix3d::Identity(), Eigen::Vector3d(xd(1), xd(2), xd(3)));

  Eigen::VectorXd q = q0;
  const double eps  = 1e-4;
  const int IT_MAX  = 1000;
  const double DT   = 1e-1;
  const double damp = 1e-6;

  pinocchio::Data::Matrix6x J(6, info_.numThreeDofContacts);
  J.setZero();

  bool success = false;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  Vector6d err;
  Eigen::VectorXd v(info_.numThreeDofContacts);
  for (int i=0;;i++)
  {
    pinocchio::forwardKinematics(model,data,q);
    const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[jointID]);
    err = pinocchio::log6(dMi).toVector();
    if(err.norm() < eps)
    {
      success = true;
      break;
    }
    if (i >= IT_MAX)
    {
      success = false;
      break;
    }
    pinocchio::computeJointJacobian(model,data,q, jointID,J);
    pinocchio::Data::Matrix6 JJt;
    JJt.noalias() = J * J.transpose();
    JJt.diagonal().array() += damp;
    v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
    q = pinocchio::integrate(model,q,v*DT);
    if(!(i%10))
      std::cout << i << ": error = " << err.transpose() << std::endl;
  }

  if(success) 
  {
    std::cout << "Convergence achieved!" << std::endl;
  }
  else 
  {
    std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
  }

  return q;
}

}  // namespace legged

PLUGINLIB_EXPORT_CLASS(legged::LeggedController, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(legged::LeggedCheaterController, controller_interface::ControllerBase)
