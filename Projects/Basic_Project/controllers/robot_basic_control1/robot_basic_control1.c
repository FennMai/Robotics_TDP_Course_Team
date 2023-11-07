/*
 * File:          robot_basic_control1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/accelerometer.h>
#include <webots/camera.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/touch_sensor.h>
#include <webots/utils/motion.h>


#define TIME_STEP 64
#define MAX_SPEED 6.28
static int time_step = -1;

static WbDeviceTag gps;

/* enable sensor */
static void enable_sensors_devices() {
  // 如何获取不同robot的gps位置呢？
  gps = wb_robot_get_device("gps"); // 调用多只bots，应该是在这里改具体的gps名？
  wb_gps_enable(gps, time_step);

}

static void print_gps() {  
  const double *p = wb_gps_get_values(gps);
  printf("----------gps----------\n");
  printf("position: [ x y z ] = [%f %f %f]\n", p[0], p[1], p[2]);
}


/* robot movement control */
// set motion 
static WbMotionRef shoot, forwards, backwards, side_step_left, side_step_right, turn_left_40, turn_left_60,
 turn_right_60, turn_right_40;
static WbMotionRef currently_playing = NULL;

// load motion files 
static void load_motion_files() {
  // step 
  forwards = wbu_motion_new("../../motions/Forwards.motion");
  backwards = wbu_motion_new("../../motions/Backwards.motion");
  side_step_left = wbu_motion_new("../../motions/SideStepLeft.motion");
  side_step_right = wbu_motion_new("../../motions/SideStepRight.motion");
  // turn
  turn_left_40 = wbu_motion_new("../../motions/TurnLeft40.motion");
  turn_left_60 = wbu_motion_new("../../motions/TurnLeft60.motion");
  turn_right_40 = wbu_motion_new("../../motions/TurnRight40.motion");
  turn_right_60 = wbu_motion_new("../../motions/TurnRight60.motion");
  // action
  shoot = wbu_motion_new("../../motions/shoot.motion");
}


/* initialize webots stuff */
static void terminate() {
  // add you cleanup code here: write results, close files, free memory, etc.
  // ...
  wb_robot_cleanup();
}

static void simulation_step() {
  if (wb_robot_step(time_step) == -1)
    terminate();
}


int main(int argc, char **argv) {

  wb_robot_init();
  time_step = wb_robot_get_basic_time_step();
  enable_sensors_devices();
  load_motion_files();

  // 例子：机器人往前走几步，用下面两行调用一次设定好的动作
  wbu_motion_play(forwards);
  currently_playing = forwards;
  
  
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
    print_gps();
    
  };

  wb_robot_cleanup();

  return 0;
}
