#include <webots/robot.h>
#include <webots/motor.h>

#define TIME_STEP 64
#define MAX_SPEED 6.28
static int time_step = -1;

// 头左右旋转(-2.09~2.09)
static WbDeviceTag HeadYaw;

// 头前后旋转(-0.67~0.51)
static WbDeviceTag HeadPitch;

// 肩膀节点控制胳膊前后旋转(-2.09~2.09)
static WbDeviceTag RShoulderPitch;  	
static WbDeviceTag LShoulderPitch;

// 肩膀节点控制胳膊左右旋转(-1.33~0.31)
static WbDeviceTag RShoulderRoll;  	
static WbDeviceTag LShoulderRoll;

// 小臂挥动(0~1.54)
static WbDeviceTag RElbowRoll;
static WbDeviceTag LElbowRoll;

// 小臂旋转(-2.09~2.09)
static WbDeviceTag RElbowYaw;
static WbDeviceTag LElbowYaw;

// 大腿节点控制腿前后旋转(-1.77~0.48)
static WbDeviceTag RHipPitch;
static WbDeviceTag LHipPitch;

// 大腿节点控制腿左右旋转(-0.74~0.45)
static WbDeviceTag RHipRoll;
static WbDeviceTag LHipRoll;

// 腿膝关节控制小腿前后旋转(-0.09~2.11)
static WbDeviceTag RKneePitch;
static WbDeviceTag LKneePitch;

// 腿向内外侧旋转(-1.15~0.74)
static WbDeviceTag RHipYawPitch;
static WbDeviceTag LHipYawPitch;

// 脚腕前后旋转(-1.19~0.92)
static WbDeviceTag RAnklePitch;
static WbDeviceTag LAnklePitch;

static WbDeviceTag LAnkleRoll;  	
static WbDeviceTag RAnkleRoll;

static void find_and_enable_devices() 
{
   RElbowRoll = wb_robot_get_device("RElbowRoll");
   
   LHipYawPitch = wb_robot_get_device("LHipYawPitch");
   RHipYawPitch = wb_robot_get_device("RHipYawPitch");
   
   LHipRoll = wb_robot_get_device("LHipRoll");
   RHipRoll = wb_robot_get_device("RHipRoll");
   
   LHipPitch = wb_robot_get_device("LHipPitch");
   RHipPitch = wb_robot_get_device("RHipPitch");
   
   LKneePitch = wb_robot_get_device("LKneePitch");
   RKneePitch = wb_robot_get_device("RKneePitch");
   
   LAnklePitch = wb_robot_get_device("LAnklePitch");
   RAnklePitch = wb_robot_get_device("RAnklePitch");
   
   LAnkleRoll = wb_robot_get_device("LAnkleRoll");
   RAnkleRoll = wb_robot_get_device("RAnkleRoll");
   
   LShoulderPitch = wb_robot_get_device("LShoulderPitch");
   RShoulderPitch = wb_robot_get_device("RShoulderPitch");
}

static void terminate() {
  // add you cleanup code here: write results, close files, free memory, etc.
  // ...

  wb_robot_cleanup();
}


static void simulation_step() {
  if (wb_robot_step(time_step) == -1)
    terminate();
}


int main() {

  wb_robot_init();
  // simulation step in milliseconds
  time_step = wb_robot_get_basic_time_step();
  
  find_and_enable_devices();
  
  // wb_motor_set_position(RElbowRoll, 1);
  // wb_motor_set_velocity(RElbowRoll, 3);
  // wb_motor_set_position(RElbowRoll, 0);
  // wb_motor_set_velocity(RElbowRoll, 5);
  // wb_robot_step(4000);

  // wb_motor_set_position(RElbowRoll, 1);
  // wb_motor_set_velocity(RElbowRoll, 3);
  // wb_robot_step(2000);
  
  // wb_motor_set_position(RElbowRoll, 0);
  // wb_motor_set_velocity(RElbowRoll, 5);
  
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
      
    // Main control loop
      // wb_motor_set_position(RElbowRoll, 1);
      // wb_motor_set_velocity(RElbowRoll, 3);
      // wb_motor_set_position(LShoulderPitch, 1);
      // wb_motor_set_velocity(LShoulderPitch, 3);
      // wb_motor_set_position(RShoulderPitch, -1);
      // wb_motor_set_velocity(RShoulderPitch, 3);
      // wb_motor_set_position(LHipRoll, 0.5);
      // wb_motor_set_velocity(LHipRoll, 1);
      // wb_motor_set_position(RHipRoll, -0.5);
      // wb_motor_set_velocity(RHipRoll, 1);
      
      // wb_motor_set_position(LAnkleRoll, -0.5);
      // wb_motor_set_velocity(LAnkleRoll, 1);
      // wb_motor_set_position(RAnkleRoll, 0.5);
      // wb_motor_set_velocity(RAnkleRoll, 1);
      

      
      wb_motor_set_position(LHipPitch, -0.8);
      wb_motor_set_velocity(LHipPitch, 4);
      wb_motor_set_position(RHipPitch, -0.8);
      wb_motor_set_velocity(RHipPitch, 4);
      
      wb_motor_set_position(LAnklePitch, -0.5);
      wb_motor_set_velocity(LAnklePitch, 1);
      wb_motor_set_position(RAnklePitch, -0.5);
      wb_motor_set_velocity(RAnklePitch, 1);
      
      wb_motor_set_position(LKneePitch, 1);
      wb_motor_set_velocity(LKneePitch, 3);
      wb_motor_set_position(RKneePitch, 1);
      wb_motor_set_velocity(RKneePitch, 3);
      
 
      
      wb_robot_step(1500);
      
      // wb_motor_set_position(RElbowRoll, 0);
      // wb_motor_set_velocity(RElbowRoll, 5);
      // wb_motor_set_position(LShoulderPitch, 0);
      // wb_motor_set_velocity(LShoulderPitch, 3);
      // wb_motor_set_position(RShoulderPitch, 0);
      // wb_motor_set_velocity(RShoulderPitch, 3);
      wb_robot_step(1500);
      
      // wb_motor_set_position(RElbowRoll, 1);
      // wb_motor_set_velocity(RElbowRoll, 3);
      // wb_motor_set_position(LShoulderPitch, 2);
      // wb_motor_set_velocity(LShoulderPitch, 3);
      // wb_motor_set_position(RShoulderPitch, -2);
      // wb_motor_set_velocity(RShoulderPitch, 3);
      wb_robot_step(1500);
  }

  wb_robot_cleanup();
  return 0;
}