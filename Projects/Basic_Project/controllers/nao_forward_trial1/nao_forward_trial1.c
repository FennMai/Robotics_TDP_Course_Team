/*
 * File:   nao_forward_trial1.c
 * Date:   20240211
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

int main()
{
  wb_robot_init();

  WbDeviceTag turn_head = wb_robot_get_device("HeadYaw");
  WbDeviceTag r_shoulder = wb_robot_get_device("RShoulderPitch");
  WbDeviceTag l_shoulder = wb_robot_get_device("LShoulderPitch");
  WbDeviceTag r_shoulder_roll = wb_robot_get_device("RShoulderRoll");
  WbDeviceTag l_shoulder_roll = wb_robot_get_device("LShoulderRoll");
  WbDeviceTag r_ElbowYaw = wb_robot_get_device("RElbowYaw");
  WbDeviceTag l_ElbowYaw = wb_robot_get_device("LElbowYaw");
  WbDeviceTag r_ElbowRoll = wb_robot_get_device("RElbowRoll");
  WbDeviceTag l_ElbowRoll = wb_robot_get_device("LElbowRoll");
  // ------------------------------------------------------- //
  //  动作 左右转头
  // -2 向左转到尽头
  // wb_motor_set_position(turn_head, -2);
  // wb_motor_set_velocity(turn_head, 2);

  // wb_robot_step(2000);

  // wb_motor_set_position(turn_head, 2);
  // wb_motor_set_velocity(turn_head, 2);

  // wb_robot_step(3000);

  // wb_motor_set_position(turn_head, 0);
  // wb_motor_set_velocity(turn_head, 2);

  // ------------------------------------------------------- //
  // 微调
  // wb_motor_set_position(r_shoulder, 1.9);
  // wb_motor_set_velocity(r_shoulder, 1);
  // wb_motor_set_position(l_shoulder, 1.9);
  // wb_motor_set_velocity(l_shoulder, 1);
  // ------------------------------------------------------- //
  // fun： 假设新添加一个机器人，如何初始化动作，如何设为行走姿态
  // r_shoulder_roll--- 横摆，-0.2，右臂向外摆

  wb_motor_set_position(r_shoulder_roll, -0.1);
  wb_motor_set_velocity(r_shoulder_roll, 1);
  // l_shoulder_roll--- 横摆， 0.2，左臂向外摆
  wb_motor_set_position(l_shoulder_roll, 0.1);
  wb_motor_set_velocity(l_shoulder_roll, 1);

  // wb_robot_step(1000);
  // -2 手臂会向上抬
  // 1.6 手放下, 平行腿
  wb_motor_set_position(r_shoulder, 1.8);
  wb_motor_set_velocity(r_shoulder, 1);
  wb_motor_set_position(l_shoulder, 1.8);
  wb_motor_set_velocity(l_shoulder, 1);

  wb_robot_step(1500);
  // 小手臂 外旋
  wb_motor_set_position(l_ElbowYaw, -1.75);
  wb_motor_set_velocity(l_ElbowYaw, 1);
  wb_motor_set_position(r_ElbowYaw, 1.75);
  wb_motor_set_velocity(r_ElbowYaw, 1);

  wb_robot_step(1500);
  // 小手臂 屈伸
  wb_motor_set_position(l_ElbowRoll, -1.2);
  wb_motor_set_velocity(l_ElbowRoll, 1);
  wb_motor_set_position(r_ElbowRoll, 1.2);
  wb_motor_set_velocity(r_ElbowRoll, 1);
  // -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- - //

  while (wb_robot_step(TIME_STEP) != -1)
  {
    // Main control loop
  }

  wb_robot_cleanup();

  return 0;
}