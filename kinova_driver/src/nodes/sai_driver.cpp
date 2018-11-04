#include "kinova_driver/kinova_api.h"

#include <hiredis/hiredis.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>

#include <string>
#include <vector>
#include <iostream>

using namespace kinova;
using namespace std;

/* Define Robot Interaction Parameters */
#define ZERO_TORQUE_POSE {180, 180, 180, 0, 180, 180}
#define GRAVITY_CONSTANT {0, -9.80665, 0}
#define KINOVA_MICO_DOF 6

/* Define REDIS Server Parameters */
#define REDIS_HOST "127.0.0.1"
#define REDIS_PORT 6379

/* Define REDIS SAI Keys */
static const string J_T = "cs225a::robot::mico::actuators::fgc";
static const string J_V = "cs225a::robot::mico::sensors::dq";
static const string J_A = "cs225a::robot::mico::sensors::q";

/* Define Signal Handler */
static volatile bool run = true;
void intHandler(int signal) {
  run = false;
}

class SAIDriver
{
public:
  
  SAIDriver()
  {
    /* Connect to REDIS */
    redis_reply_ = NULL;
    redis_context_ = redisConnect(REDIS_HOST, REDIS_PORT);
    assert(redis_context_->err == 0);
    freeReplyObject(redisCommand(redis_context_, "SET USE_SAI_TORQUES FALSE"));

    /* Initialize API */
    api.initializeKinovaAPIFunctions(USB);
    assert(api.initAPI() == NO_ERROR_KINOVA);
  }

  ~SAIDriver()
  {
    redisFree(redis_context_);
    api.closeAPI();
  }
  
  void setModeTorqueControl()
  {
    assert(api.switchTrajectoryTorque(TORQUE) == NO_ERROR_KINOVA);
    assert(api.setTorqueControlType(DIRECTTORQUE) == NO_ERROR_KINOVA);
    assert(api.setTorqueSafetyFactor(1) == NO_ERROR_KINOVA);
    assert(api.setTorqueVibrationController(0) == NO_ERROR_KINOVA);
    cout << "Robot set to torque control mode." << endl;
  }
  
  void setModePositionControl()
  {
    assert(api.switchTrajectoryTorque(POSITION) == NO_ERROR_KINOVA);
    cout << "Robot set to position control mode." << endl;
  }
  
  void calibrateTorques()
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
    cout << "Sending robot to zero-torque configuration." << endl;
    usleep(10000000);
    
    /* Calibrate the Torque Sensors to Zero */
    int actuator_address[] = {16,17,18,19,20,21,25};
    for (int i= 0; i < KINOVA_MICO_DOF; i++) {
      assert(api.setTorqueZero(actuator_address[i]) == NO_ERROR_KINOVA);
      cout << "Joint " << i << " calibrated to zero torque." << endl;
    }
  }
  
  void setGravity()
  {
    float gravity[3] = GRAVITY_CONSTANT;
    assert(api.setGravityVector(gravity) == NO_ERROR_KINOVA);
    cout << "Set Gravity Vector." << endl;
  }
  
  void runTorqueControl()
  {
    float command_torques[6];
    while(run) {      
      if (!useSAITorques()) {
	for(int i = 0; i < KINOVA_MICO_DOF; i++) {command_torques[i] = 0;}
      } else {
	queryRedis(J_T, command_torques);
	for(int i = 0; i < KINOVA_MICO_DOF; i++) {cout << command_torques[i] << " ";}
	cout << endl;
      }
      assert(api.sendAngularTorqueCommand(command_torques) == NO_ERROR_KINOVA);
      AngularPosition angles, velocities;
      assert(api.getAngularPosition(angles) == NO_ERROR_KINOVA);
      assert(api.getAngularVelocity(velocities) == NO_ERROR_KINOVA);
      setJointStates(angles.Actuators, velocities.Actuators);
    }
  }

  void runPositionControl()
  {
    TrajectoryPoint trajectory;
    trajectory.InitStruct();
    trajectory.Position.Type = ANGULAR_POSITION;
    while(run) {
      getJointAngles(trajectory.Position.Actuators);
      api.sendAdvanceTrajectory(trajectory);
      api.eraseAllTrajectories();
    }
  }
  
private:

  /* Fields */
  KinovaAPI api;
  redisContext *redis_context_;
  redisReply *redis_reply_;

  bool useSAITorques()
  {
    redis_reply_ = (redisReply *) redisCommand(redis_context_, "GET USE_SAI_TORQUES");
    return string(redis_reply_->str) == "TRUE";
    freeReplyObject(redis_reply_);
  }
  
  void setJointStates(AngularInfo &angles, AngularInfo &velocities)
  {
    /* Set Up Joint Angles String */
    ostringstream oss_a;
    oss_a << (angles.Actuator1 * M_PI) / 180.0 << " ";
    oss_a << (angles.Actuator2 * M_PI) / 180.0 << " ";
    oss_a << (angles.Actuator3 * M_PI) / 180.0 << " ";
    oss_a << (angles.Actuator4 * M_PI) / 180.0 << " ";
    oss_a << (angles.Actuator5 * M_PI) / 180.0 << " ";
    oss_a << (angles.Actuator6 * M_PI) / 180.0;
    
    /* Set up Joint Velocities String */
    ostringstream oss_v;
    oss_v << (velocities.Actuator1 * M_PI) / 180.0 << " ";
    oss_v << (velocities.Actuator2 * M_PI) / 180.0 << " ";
    oss_v << (velocities.Actuator3 * M_PI) / 180.0 << " ";
    oss_v << (velocities.Actuator4 * M_PI) / 180.0 << " ";
    oss_v << (velocities.Actuator5 * M_PI) / 180.0 << " ";
    oss_v << (velocities.Actuator6 * M_PI) / 180.0;
  
    /* Publish Angles and Velocities */
    freeReplyObject(redisCommand(redis_context_, "SET %s %s", J_A.c_str(), oss_a.str().c_str()));
    freeReplyObject(redisCommand(redis_context_, "SET %s %s", J_V.c_str(), oss_v.str().c_str()));
  }
  
  void queryRedis(string key, float *response_array)
  {
    /* Get REDIS Key Value */
    redis_reply_ = (redisReply *) redisCommand(redis_context_, "GET %s", key.c_str());
    
    /* Parse Response */
    char *iter_pointer = redis_reply_->str;
    for(int i = 0; i < KINOVA_MICO_DOF; i++) {
      response_array[i] = strtod(iter_pointer, &iter_pointer);
    }

    /* Deallocate REDIS Reply Object */
    freeReplyObject(redis_reply_);
  }
  
  void getJointAngles(AngularInfo &actuators)
  {
    float angles[6];
    queryRedis(J_A, angles);
    actuators.Actuator1 = (180.0 / M_PI) * angles[0];
    actuators.Actuator2 = (180.0 / M_PI) * angles[1];
    actuators.Actuator3 = (180.0 / M_PI) * angles[2];
    actuators.Actuator4 = (180.0 / M_PI) * angles[3];
    actuators.Actuator5 = (180.0 / M_PI) * angles[4];
    actuators.Actuator6 = (180.0 / M_PI) * angles[5];
  }
  
};

void print_usage()
{
  cout << "Usage: ./sai_driver <task_option>" << endl;
  cout << " <task_option> must be one of:" << endl;
  cout << "  'calibrate' - The robot moves to a zero-torque configuration and calibrates." << endl;
  cout << "  'angle-ctrl' - The robot follows the joint angles published by SAI using a simulator." << endl;
  cout << "  'torque-ctrl' - The robot does torque-control based on SAI controller-computed torques." << endl;
}

string parseArgs(int argc, char** argv)
{
  if (argc != 2) {
    print_usage();
    exit(0);
  }
  string option = string(argv[1]);
  if(option != "calibrate"
     && option != "angle-ctrl"
     && option != "torque-ctrl") {
    print_usage();
    exit(0);
  }
  return option;
}


int main(int argc, char *argv[])
{
  string task = parseArgs(argc, argv);
  signal(SIGINT, intHandler);
  SAIDriver driver;
  
  if(task == "calibrate") {
    cout << "Running Torque Calibration." << endl;
    driver.setModePositionControl();
    driver.calibrateTorques();
  } else if(task == "angle-ctrl") {
    cout << "Running Position Control." << endl;
    driver.setModePositionControl();
    driver.runPositionControl();
  } else {
    cout << "Running Torque Control." << endl;
    driver.setModeTorqueControl();
    driver.setGravity();
    driver.runTorqueControl();
  }
  
  return 0;
}
