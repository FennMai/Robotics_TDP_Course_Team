#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/utils/motion.h>
#include "rb_ctrl.h"

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

// 脚腕左右旋转(-0.40~0.77)
static WbDeviceTag LAnkleRoll;  	
static WbDeviceTag RAnkleRoll;

// gps
static WbDeviceTag gps;

static int time_step = -1;

static WbMotionRef shoot, hand_wave, forwards, backwards, side_step_left, side_step_right, turn_left_60, turn_right_60, standupfromfront;
static WbMotionRef currently_playing = NULL;

void find_and_enable_devices() 
{
   LElbowRoll = wb_robot_get_device("LElbowRoll");
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
   
   LShoulderRoll = wb_robot_get_device("LShoulderRoll");
   RShoulderRoll = wb_robot_get_device("RShoulderRoll");
   
   HeadYaw = wb_robot_get_device("HeadYaw");
   HeadPitch = wb_robot_get_device("HeadPitch");
   
   // gps
   gps = wb_robot_get_device("gps");
   wb_gps_enable(gps, time_step);
}

void initial_arms()
{
	wb_motor_set_position(LElbowRoll, 0);
    wb_motor_set_velocity(LElbowRoll, 1);
	wb_motor_set_position(RElbowRoll, 0);
    wb_motor_set_velocity(RElbowRoll, 1);
	
    wb_motor_set_position(LShoulderPitch, 1.2);
    wb_motor_set_velocity(LShoulderPitch, 1);
    wb_motor_set_position(RShoulderPitch, 1.2);
    wb_motor_set_velocity(RShoulderPitch, 1);
	
	wb_motor_set_position(LShoulderRoll, 0.2);
    wb_motor_set_velocity(LShoulderRoll, 1);
    wb_motor_set_position(RShoulderRoll, -0.2);
    wb_motor_set_velocity(RShoulderRoll, 1);
	
	wb_motor_set_position(LElbowYaw, 0);
    wb_motor_set_velocity(LElbowYaw, 1);
    wb_motor_set_position(RElbowYaw, 0);
    wb_motor_set_velocity(RElbowYaw, 1);
}

void initial_head()
{
    wb_motor_set_position(HeadYaw, 0);
    wb_motor_set_velocity(HeadYaw, 3);
    wb_motor_set_position(HeadPitch, 0);
    wb_motor_set_velocity(HeadPitch, 3);
}

void initial_legs()
{
	wb_motor_set_position(LHipRoll, 0);
    wb_motor_set_velocity(LHipRoll, 1);
    wb_motor_set_position(RHipRoll, 0);
    wb_motor_set_velocity(RHipRoll, 1);
    
    wb_motor_set_position(LAnkleRoll, 0);
    wb_motor_set_velocity(LAnkleRoll, 1);
    wb_motor_set_position(RAnkleRoll, 0);
    wb_motor_set_velocity(RAnkleRoll, 1);
	
	wb_motor_set_position(LHipPitch, 0);
    wb_motor_set_velocity(LHipPitch, 1);
    wb_motor_set_position(RHipPitch, 0);
    wb_motor_set_velocity(RHipPitch, 1);
}

void squat()
{
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
}

void open_arms()
{
	wb_motor_set_position(LShoulderRoll, 1.3);
    wb_motor_set_velocity(LShoulderRoll, 3);
    wb_motor_set_position(RShoulderRoll, -1.3);
    wb_motor_set_velocity(RShoulderRoll, 3);
	
    wb_motor_set_position(LShoulderPitch, 0);
    wb_motor_set_velocity(LShoulderPitch, 3);
    wb_motor_set_position(RShoulderPitch, 0);
    wb_motor_set_velocity(RShoulderPitch, 3);
}

void raise_arms()
{
    wb_motor_set_position(LShoulderRoll, 0.2);
    wb_motor_set_velocity(LShoulderRoll, 3);
    wb_motor_set_position(RShoulderRoll, -0.2);
    wb_motor_set_velocity(RShoulderRoll, 3);
	
    wb_motor_set_position(LShoulderPitch, -1.3);
    wb_motor_set_velocity(LShoulderPitch, 3);
    wb_motor_set_position(RShoulderPitch, -1.3);
    wb_motor_set_velocity(RShoulderPitch, 3);
}

void bend_arms()
{
	wb_motor_set_position(LShoulderRoll, 0.2);
    wb_motor_set_velocity(LShoulderRoll, 3);
    wb_motor_set_position(RShoulderRoll, -0.2);
    wb_motor_set_velocity(RShoulderRoll, 3);
	
    wb_motor_set_position(LShoulderPitch, 1);
    wb_motor_set_velocity(LShoulderPitch, 3);
    wb_motor_set_position(RShoulderPitch, 1);
    wb_motor_set_velocity(RShoulderPitch, 3);
	
	wb_motor_set_position(LElbowYaw, -1.3);
    wb_motor_set_velocity(LElbowYaw, 3);
    wb_motor_set_position(RElbowYaw, 1.3);
    wb_motor_set_velocity(RElbowYaw, 3);
	
	wb_motor_set_position(LElbowRoll, 1.5);
    wb_motor_set_velocity(LElbowRoll, 3);
    wb_motor_set_position(RElbowRoll, 1.5);
    wb_motor_set_velocity(RElbowRoll, 3);
}

void turn_head_left()
{
	wb_motor_set_position(HeadYaw, 2);
    wb_motor_set_velocity(HeadYaw, 3);
}

void turn_head_right()
{
	wb_motor_set_position(HeadYaw, -2);
    wb_motor_set_velocity(HeadYaw, 3);
}

void turn_head_up()
{
	wb_motor_set_position(HeadPitch, -0.1);
    wb_motor_set_velocity(HeadPitch, 3);
}

void turn_head_down()
{
	wb_motor_set_position(HeadPitch, 0.2);
    wb_motor_set_velocity(HeadPitch, 3);
}

void load_motion_files()
{
  hand_wave = wbu_motion_new("../../motions/HandWave.motion");
  forwards = wbu_motion_new("../../motions/Forwards50.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  turn_left_60 = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right_60 = wbu_motion_new("../../motions/TurnRight60.motion");
  shoot = wbu_motion_new("../../motions/shoot.motion");
  standupfromfront = wbu_motion_new("../../motions/StandUpFromFront.motion");
}

void start_motion(WbMotionRef motion)
{
  // interrupt current motion
  if (currently_playing)
    wbu_motion_stop(currently_playing);

  // start new motion
  wbu_motion_play(motion);
  currently_playing = motion;
}