#include "kinova_driver/kinova_api.h"
#include <unistd.h>
#include <string>
#include <vector>
#include <iostream>

using namespace kinova;

int main()
{
  /* Initialize API */
  KinovaAPI kinova_api_;
  int robot_dof_ = 6;
  kinova_api_.initializeKinovaAPIFunctions(USB);
  assert(kinova_api_.initAPI() == NO_ERROR_KINOVA);
  std::cout << "Initialized Kinova API." << std::endl;

  /* Define Joint Angle Trajectory */
  TrajectoryPoint trajectory;
  trajectory.InitStruct();
  trajectory.Position.Type = ANGULAR_POSITION;

  /* Natural Zero-Torque Position */
  float zt_pose[6] = {180, 180, 180, 0, 180, 180};
  trajectory.Position.Actuators.Actuator1 = zt_pose[0];
  trajectory.Position.Actuators.Actuator2 = zt_pose[1];
  trajectory.Position.Actuators.Actuator3 = zt_pose[2];
  trajectory.Position.Actuators.Actuator4 = zt_pose[3];
  trajectory.Position.Actuators.Actuator5 = zt_pose[4];
  trajectory.Position.Actuators.Actuator6 = zt_pose[5];

  /* Switch to Position Control Mode */
  assert(kinova_api_.switchTrajectoryTorque(POSITION) == NO_ERROR_KINOVA);

  /* Send Trajectory and Wait for Arrival */
  assert(kinova_api_.sendAdvanceTrajectory(trajectory) == NO_ERROR_KINOVA);
  std::cout << "Sent Zero-Torque Position." << std::endl;
  std::cout << "Allowing the Robot 10 Seconds for Motion ..." << std::endl;
  usleep(10000000);
  std::cout << "10-Second Waiting Period Over." << std::endl;
  
  /* Calibrate the Torque Sensors to Zero */
  int actuator_address[] = {16,17,18,19,20,21,25};
  for (int i= 0; i < robot_dof_; i++) {
    assert(kinova_api_.setTorqueZero(actuator_address[i]) == NO_ERROR_KINOVA);
    std::cout << "Calibrated Gravity on Joint: " << (i+1) << "." << std::endl;
  }
  
  /* Switch to Torque Control Mode */
  assert(kinova_api_.switchTrajectoryTorque(TORQUE) == NO_ERROR_KINOVA);
  assert(kinova_api_.setTorqueSafetyFactor(1) == NO_ERROR_KINOVA);
  
  /* Set Gravity Vector to Zero */
  float gravity[3] = {0.0, -9.8, 0.0};
  assert(kinova_api_.setGravityVector(gravity) == NO_ERROR_KINOVA);
  std::cout << "Set Gravity Vector." << std::endl;

  /* Command Robot to Zero Torques */
  float command_torques[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  while(true) {
    std::cout << "Sending Torques: {0, 0, 0, 0, 0, 0}" << std::endl;
    assert(kinova_api_.sendAngularTorqueCommand(command_torques) == NO_ERROR_KINOVA);
  }

  kinova_api_.closeAPI();
  return 0;
}
