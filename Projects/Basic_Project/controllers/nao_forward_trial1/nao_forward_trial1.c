/*
 * File:          nao_forward_trial1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define MAX_SPEED 6.28
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */

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

// 跟上面的动作有点相似
// 大腿节点 内外侧旋转(-1.15~0.74)
static WbDeviceTag RHipYawPitch;
static WbDeviceTag LHipYawPitch;

// 腿膝关节控制小腿前后旋转(-0.09~2.11)
static WbDeviceTag RKneePitch;
static WbDeviceTag LKneePitch;


// 脚腕前后旋转(-1.19~0.92)
static WbDeviceTag RAnklePitch;
static WbDeviceTag LAnklePitch;


int main() {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("LHipPitch");
  WbDeviceTag right_motor = wb_robot_get_device("RHipPitch");

  wb_motor_set_position(left_motor, 0);
  wb_motor_set_position(right_motor, 0);
  wb_motor_set_velocity(left_motor, 3);
  wb_motor_set_velocity(right_motor, 3);

  while (wb_robot_step(TIME_STEP) != -1) {
    // Main control loop
  }

  wb_robot_cleanup();

  return 0;
}