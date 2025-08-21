//
// Created by qiayuan on 2022/6/24.
//

#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <legged_common/hardware_interface/ContactSensorInterface.h>

#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_legged_robot_ros/visualization/LeggedRobotVisualizer.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>

#include <legged_estimation/StateEstimateBase.h>
#include <legged_interface/LeggedInterface.h>
#include <legged_wbc/WbcBase.h>
#include <legged_wbc/TrunkControllerBase.h>

#include "legged_controllers/SafetyChecker.h"
#include "legged_controllers/visualization/LeggedSelfCollisionVisualization.h"

#include <legged_estimation/ContactEstimate.h>
#include "std_msgs/Time.h"

namespace legged {
using namespace ocs2;
using namespace legged_robot;

class LeggedController : public controller_interface::MultiInterfaceController<HybridJointInterface, hardware_interface::ImuSensorInterface,
                                                                               ContactSensorInterface> {
 public:
  LeggedController() = default;
  ~LeggedController() override;
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& /*time*/) override { mpcRunning_ = false; }

 protected:
  virtual void updateStateEstimation(const ros::Time& time, const ros::Duration& period);

  virtual void setupLeggedInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile,
                                    bool verbose);
  virtual void setupMpc();
  virtual void setupMrt();
  virtual void setupStateEstimate(const std::string& taskFile, bool verbose);
  void mipActivationCallback(std_msgs::Time msg);
  void readCSVDouble(std::string file, std::vector<std::vector<double>> &vec);
  void readCSVInt(std::string file, std::vector<std::vector<int>> &vec);
  void readCSVDouble(std::string file, std::vector<double> &vec);
  Eigen::VectorXd ik(vector_t q0, int jointID, vector_t xd);

  // Interface
  std::shared_ptr<LeggedInterface> leggedInterface_;
  std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;
  std::vector<HybridJointHandle> hybridJointHandles_, switcherHandles_, rollerHandles_;
  std::vector<ContactSensorHandle> contactHandles_;
  hardware_interface::ImuSensorHandle imuSensorHandle_;

  ros::Publisher leg1_contact_force_pub;
  ros::Publisher leg2_contact_force_pub;
  ros::Publisher leg3_contact_force_pub;
  ros::Publisher leg4_contact_force_pub;

  ros::Publisher leg1_force_sensor_pub;

  ros::Publisher height_pub;

  std_msgs::Int16 leg1_contact_force;
  std_msgs::Int16 leg2_contact_force;
  std_msgs::Int16 leg3_contact_force;
  std_msgs::Int16 leg4_contact_force;

  std_msgs::Int16 leg1_force_sensor;
  
  std_msgs::Float64 height;

  // State Estimation
  SystemObservation currentObservation_;
  vector_t measuredRbdState_;
  std::shared_ptr<StateEstimateBase> stateEstimate_;
  std::shared_ptr<ContactEstimate> contactEstimate_;
  std::shared_ptr<CentroidalModelRbdConversions> rbdConversions_;
  contact_flag_t contactFlag;

  // Whole Body Control
  std::shared_ptr<WbcBase> wbc_;
  std::shared_ptr<TrunkControllerBase> tc_;
  std::shared_ptr<SafetyChecker> safetyChecker_;

  // Nonlinear MPC
  std::shared_ptr<MPC_BASE> mpc_;
  std::shared_ptr<MPC_MRT_Interface> mpcMrtInterface_;

  // Visualization
  std::shared_ptr<LeggedRobotVisualizer> robotVisualizer_;
  std::shared_ptr<LeggedSelfCollisionVisualization> selfCollisionVisualization_;
  ros::Publisher observationPublisher_;

  size_t updatedMode;

  double contactTime;

  bool mipActivated = false;
  ros::Time mipStartTime;
  ros::Time time_;
  ros::Subscriber mipActivationSub;

  std::string eePosFile;
  std::string eeRegionsFile;
  std::string eeTypesFile;
  std::string forcesFile;
  std::string statesFile;
  std::string swingsFile;
  std::string timeFile;

  std::vector<std::vector<double>> eePos;
  std::vector<std::vector<int>> eeRegions;
  std::vector<std::vector<int>> eeTypes;
  std::vector<std::vector<double>> forces;
  std::vector<std::vector<double>> states;
  std::vector<std::vector<int>> swings;
  std::vector<double> times;

 private:
  std::thread mpcThread_;
  std::atomic_bool controllerRunning_{}, mpcRunning_{};
  benchmark::RepeatedTimer mpcTimer_;
  benchmark::RepeatedTimer wbcTimer_;

  std::vector<std::string> switcher_names{"LF_switcher", "RF_switcher", "LH_switcher", "RH_switcher"};
  std::vector<std::string> roller_names{"LF_roller", "LH_roller", "RF_roller", "RH_roller"};

  double switcherUpper = 0.05;
  double switcherLower = -0.07;
};

class LeggedCheaterController : public LeggedController {
 protected:
  void setupStateEstimate(const std::string& taskFile, bool verbose) override;
};

}  // namespace legged
