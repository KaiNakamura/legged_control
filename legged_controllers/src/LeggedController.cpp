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
#include <pluginlib/class_list_macros.hpp>

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/kinematics.hpp"

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
  auto* contactInterface = robot_hw->get<ContactSensorInterface>();
  for (const auto& name : leggedInterface_->modelSettings().contactNames3DoF) {
    contactHandles_.push_back(contactInterface->getHandle(name));
  }
  imuSensorHandle_ = robot_hw->get<hardware_interface::ImuSensorInterface>()->getHandle("unitree_imu");

  // State estimation
  setupStateEstimate(taskFile, verbose);

  // Whole body control
  wbc_ = std::make_shared<WeightedWbc>(leggedInterface_->getPinocchioInterface(), leggedInterface_->getCentroidalModelInfo(),
                                       *eeKinematicsPtr_);
  wbc_->loadTasksSetting(taskFile, verbose);

  // Safety Checker
  safetyChecker_ = std::make_shared<SafetyChecker>(leggedInterface_->getCentroidalModelInfo());

  return true;
}

void LeggedController::starting(const ros::Time& time) {
  // Initial state
  currentObservation_.state.setZero(leggedInterface_->getCentroidalModelInfo().stateDim);
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

  if(!mipActivated){
    if(measuredRbdState_(5) > 0.15){
      currentObservation_.mode = updatedMode;
    }

    // Update the current state of the system
    mpcMrtInterface_->setCurrentObservation(currentObservation_);

    // Load the latest MPC policy
    mpcMrtInterface_->updatePolicy();

    // Evaluate the current policy
    vector_t optimizedState, optimizedInput;
    size_t plannedMode = 0;  // The mode that is active at the time the policy is evaluated at.
    mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState, optimizedInput, plannedMode);

    // Whole body control
    currentObservation_.input = optimizedInput;

    wbcTimer_.startTimer();
    vector_t x;
    if(measuredRbdState_(5) > 0.15){
      x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, updatedMode, period.toSec());
    }
    else{
      x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, plannedMode, period.toSec());
    }
    wbcTimer_.endTimer();

    vector_t torque = x.tail(12);
    updatedMode = contactEstimate_->update(currentObservation_.time, period, optimizedInput, measuredRbdState_, torque, contactFlag, mpcMrtInterface_->activePrimalSolutionPtr_->modeSchedule_);
    
    // std::cout << measuredRbdState_(5) << std::endl;

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

    vector_t posDes = centroidal_model::getJointAngles(optimizedState, leggedInterface_->getCentroidalModelInfo());
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput, leggedInterface_->getCentroidalModelInfo());

    // Safety check, if failed, stop the controller
    if (!safetyChecker_->check(currentObservation_, optimizedState, optimizedInput)) {
      ROS_ERROR_STREAM("[Legged Controller] Safety check failed, stopping the controller.");
      stopRequest(time);
    }

    for (size_t j = 0; j < leggedInterface_->getCentroidalModelInfo().actuatedDofNum; ++j) {
      hybridJointHandles_[j].setCommand(posDes(j), velDes(j), 0, 3, torque(j));
    }

    // std::cout << "Optimized state: " << std::endl << optimizedState << std::endl;
    // std::cout << "Optimized input: " << std::endl << optimizedInput << std::endl;
    // std::cout << "rbdState: " << std::endl << measuredRbdState_ << std::endl;
  }
  else{
    vector_t x;
    std::cout << "Time: " << (time.nsec - mipStartTime.nsec)/1.0e9 << std::endl;
    
    int idx = 0;
    for(int i = 1; i < times.size(); i++){
      // std::cout << times[i] << " ";
      if((time.nsec - mipStartTime.nsec)/1.0e9 < times[i]){
        idx = i - 1;
        break;
      }
    }
    // std::cout << std::endl;

    if((time.nsec - mipStartTime.nsec)/1.0e9 > 0.03){
      std::cout << "Idx: " << idx << std::endl;

      // std::cout << "States: " << std::endl;
      // for(int i = 0; i < states.size(); i++){
      //   for(int j = 0; j < states[0].size(); j++){
      //     std::cout << i << ", " << j << " " << states[i][j] << std::endl;
      //   }
      //   std::cout << std::endl;
      // }

      // std::cout << "Forces: " << forces.size();
      // std::cout << " Forces[0]: " << forces[0].size() << std::endl;
      // Rearrange csv order into acceptable form
      Eigen::VectorXd optimizedState = Eigen::VectorXd::Zero(24);
      Eigen::VectorXd optimizedInput = Eigen::VectorXd::Zero(18);

      Eigen::VectorXd hqb = Eigen::VectorXd::Zero(12);
      hqb << states[6][idx], states[7][idx], states[8][idx], states[15][idx], states[16][idx], states[17][idx], states[0][idx], states[1][idx], states[2][idx], states[9][idx], states[10][idx], states[11][idx];
      optimizedState.segment<12>(0) = hqb;
      // optimizedState(0) = states[7][idx]; optimizedState(1) = states[8][idx]; optimizedState(2) = states[9][idx];
      // optimizedState(3) = states[15][idx]; optimizedState(4) = states[16][idx]; optimizedState(5) = states[17][idx];
      // optimizedState(6) = states[0][idx]; optimizedState(7) = states[1][idx]; optimizedState(8) = states[2][idx];
      // optimizedState(9) = states[10][idx]; optimizedState(10) = states[11][idx]; optimizedState(11) = states[12][idx];

      CentroidalModelInfo info_ = leggedInterface_->getCentroidalModelInfo();
      eeKinematicsPtr_->setPinocchioInterface(leggedInterface_->getPinocchioInterface());

      std::vector<vector3_t> footPos = eeKinematicsPtr_->getPosition(vector_t());

      Eigen::VectorXd qMeasured_ = Eigen::VectorXd(info_.generalizedCoordinatesNum);
      qMeasured_.head<3>() = measuredRbdState_.segment<3>(3);
      qMeasured_.segment<3>(3) = measuredRbdState_.head<3>();
      qMeasured_.tail(info_.actuatedDofNum) = measuredRbdState_.segment(6, info_.actuatedDofNum);

      // std::cout << "Optimized State pre-ik: " << optimizedState << std::endl;
      for(int i = 0; i < info_.numThreeDofContacts; i++){
        Eigen::VectorXd footstep = Eigen::VectorXd(3);
        footstep << eePos[0 + 3*i][idx], eePos[1 + 3*i][idx], eePos[2 + 3*i][idx];

        // Eigen::VectorXd q = ik(qMeasured_.segment<3>(6 + 3*i), info_.endEffectorFrameIndices[i], footstep);
        // std::cout << "q" << i << ": " << q << std::endl <<  "segmented:" << std::endl << q.segment<3>(0) << std::endl << "qMeasured: " << qMeasured_.segment<3>(6 + 3*i) << std::endl;

        optimizedState.segment(12 + 3*i, 3) = qMeasured_.segment(6 + 3*i, 3);
      }

      std::cout << "Optimized State: " << optimizedState << std::endl;

      for(int i = 0; i < 3*info_.numThreeDofContacts; i++){
        optimizedInput(i) = forces[i][idx];
      }
      // std::cout << "Optimized Input: " << optimizedInput << std::endl;

      size_t currMode = 0;
      for(int i = 0; i < info_.numThreeDofContacts; i++){
        if(swings[i][idx / (times.size()/swings[0].size())]){
          currMode += (int) pow(2, i);
          // mean_zg[i] = footPos[i](2) + 0.05;
        }
      }
      x = wbc_->update(optimizedState, optimizedInput, measuredRbdState_, currMode, period.toSec());
      
      // updatedMode = contactEstimate_->update(currentObservation_.time, period, optimizedInput, measuredRbdState_, torque, contactFlag, mpcMrtInterface_->activePrimalSolutionPtr_->modeSchedule_);

    }
  }
  // Visualization
  robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());
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
