#include "kinova_driver/kinova_api.h"

#include <signal.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <iostream>

/* Define Gravity and Zero-Torque Pose */
#define ZERO_TORQUE_POSE {180, 180, 180, 0, 180, 180}
#define GRAVITY_CONSTANT {0, -9.80665, 0}
using namespace kinova;

/* Define Signal Handler */
static volatile bool run = true;
void intHandler(int signal) {
  run = false;
}

class SAIDriver
{
public:
  
  SAIDriver(int dof)
  {
    /* Initialize State */
    robot_dof_ = dof;
    sai_started_ = false;

    /* Initialize API */
    api.initializeKinovaAPIFunctions(USB);
    assert(api.initAPI() == NO_ERROR_KINOVA);
  }

  ~SAIDriver()
  {
    api.closeAPI();
  }

  void SetModeTorqueControl()
  {
    assert(api.switchTrajectoryTorque(TORQUE) == NO_ERROR_KINOVA);
    assert(api.setTorqueControlType(DIRECTTORQUE) == NO_ERROR_KINOVA);
    assert(api.setTorqueSafetyFactor(1) == NO_ERROR_KINOVA);
    assert(api.setTorqueVibrationController(0) == NO_ERROR_KINOVA);
    std::cout << "Robot set to torque control mode." << std::endl;
  }
  
  void SetModePositionControl()
  {
    assert(api.switchTrajectoryTorque(POSITION) == NO_ERROR_KINOVA);
    std::cout << "Robot set to position control mode." << std::endl;
  }
  
  void CalibrateTorques()
  { 
    /* Define Joint Angle Trajectory */
    TrajectoryPoint trajectory;
    trajectory.InitStruct();
    trajectory.Position.Type = ANGULAR_POSITION;

    /* Natural Zero-Torque Position */
    float zt_pose[6] = ZERO_TORQUE_POSE;
    trajectory.Position.Actuators.Actuator1 = zt_pose[0];
    trajectory.Position.Actuators.Actuator2 = zt_pose[1];
    trajectory.Position.Actuators.Actuator3 = zt_pose[2];
    trajectory.Position.Actuators.Actuator4 = zt_pose[3];
    trajectory.Position.Actuators.Actuator5 = zt_pose[4];
    trajectory.Position.Actuators.Actuator6 = zt_pose[5];

    
    /* Send Trajectory and Wait for Arrival */
    assert(api.sendAdvanceTrajectory(trajectory) == NO_ERROR_KINOVA);
    std::cout << "Sending robot to zero-torque configuration." << std::endl;
    usleep(10000000);
    
    /* Calibrate the Torque Sensors to Zero */
    int actuator_address[] = {16,17,18,19,20,21,25};
    for (int i= 0; i < robot_dof_; i++) {
      assert(api.setTorqueZero(actuator_address[i]) == NO_ERROR_KINOVA);
      std::cout << "Joint " << i << " calibrated to zero torque." << std::endl;
    }
  }
  
  void SetGravity()
  {
    float gravity[3] = GRAVITY_CONSTANT;
    assert(api.setGravityVector(gravity) == NO_ERROR_KINOVA);
    std::cout << "Set Gravity Vector." << std::endl;
  }
  
  void RunTorqueControl()
  {
    float command_torques[6];
    while(run) {
      if (!sai_started_) {
	for(int i = 0; i < robot_dof_; i++) {command_torques[i] = 0;}
      } else {
	// TODO: Read Joint Torques from REDIS
      }
      assert(api.sendAngularTorqueCommand(command_torques) == NO_ERROR_KINOVA);
      // TODO: Get and Publish Joint Angles to REDIS
    }
  }
  
private:

  KinovaAPI api;
  int robot_dof_;
  bool sai_started_;
  
};

int main()
{
  /* Install Signal Handler */
  signal(SIGINT, intHandler);
  
  /* Instantiate Driver */
  SAIDriver driver(6);
  
  /* Calibrate Torques */
  driver.SetModePositionControl();
  driver.CalibrateTorques();

  /* Torque Control */
  driver.SetModeTorqueControl();
  driver.SetGravity();
  driver.RunTorqueControl();
  return 0;
}
