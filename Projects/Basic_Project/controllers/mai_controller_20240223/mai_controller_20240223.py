"""
mai_controller_20240223 controller.
基于mai_controller_20240208 controller.

"""

# load python reposity
import math
import _thread
import random
import time
import numpy as np
import mai_filter_0224

# load webot
from controller import Robot
from controller import Accelerometer
from controller import Camera
from controller import Motor
from controller import Motion
from controller import Receiver
from controller import Emitter
from controller import Supervisor
from controller import Node


class nao6_define(Robot):
    PHALANX_MAX = 8
    # action copy的，需要修改
    def loadMotionFiles(self):
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards.motion')
        self.fad50 = Motion('../../motions/Forwards50.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        # turn left
        self.turnLeft20 = Motion('../../motions/TurnLeft20.motion')
        self.turnLeft40 = Motion('../../motions/TurnLeft40.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnLeft90 = Motion('../../motions/TurnLeft90BasedOn180.motion')
        self.turnLeft180 = Motion('../../motions/TurnLeft180.motion')
        # turn right
        self.turnRight20 = Motion('../../motions/TurnRight20.motion')
        self.turnRight40 = Motion('../../motions/TurnRight40.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')

        # shoot
        self.shoot = Motion('../../motions/Shoot.motion')
    
    # 如果有新命令，可以直接中断前一个命令
    # 机器人是否会不稳定？会的，所以可以优化动作
    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()

        # start new motion
        motion.play()
        self.currentlyPlaying = motion

    # 初始化/启动 sensors服务
    def findAndEnableDevices(self):
        # get the time step of the current world.
        # self.timeStep = 64  # set the control time step
        self.timeStep = int(self.getBasicTimeStep())

        # camera
        self.cameraTop = self.getDevice("CameraTop")
        self.cameraBottom = self.getDevice("CameraBottom")
        self.cameraTop.enable(4 * self.timeStep)
        self.cameraBottom.enable(4 * self.timeStep)

        # accelerometer
        self.accelerometer = self.getDevice('accelerometer')
        self.accelerometer.enable(4 * self.timeStep)

        # gyro
        self.gyro = self.getDevice('gyro')
        self.gyro.enable(4 * self.timeStep)

        # gps
        self.gps = self.getDevice('gps')
        self.gps.enable(4 * self.timeStep)

        # inertial unit
        # 惯量
        self.inertialUnit = self.getDevice('inertial unit')
        self.inertialUnit.enable(self.timeStep)

        # ultrasound sensors
        self.us = []
        usNames = ['Sonar/Left', 'Sonar/Right']
        for i in range(0, len(usNames)):
            self.us.append(self.getDevice(usNames[i]))
            self.us[i].enable(self.timeStep)

        # foot sensors
        self.fsr = []
        fsrNames = ['LFsr', 'RFsr']
        for i in range(0, len(fsrNames)):
            self.fsr.append(self.getDevice(fsrNames[i]))
            self.fsr[i].enable(self.timeStep)

        # foot bumpers
        self.lfootlbumper = self.getDevice('LFoot/Bumper/Left')
        self.lfootrbumper = self.getDevice('LFoot/Bumper/Right')
        self.rfootlbumper = self.getDevice('RFoot/Bumper/Left')
        self.rfootrbumper = self.getDevice('RFoot/Bumper/Right')
        self.lfootlbumper.enable(self.timeStep)
        self.lfootrbumper.enable(self.timeStep)
        self.rfootlbumper.enable(self.timeStep)
        self.rfootrbumper.enable(self.timeStep)

        # there are 7 controlable LED groups in Webots
        self.leds = []
        self.leds.append(self.getDevice('ChestBoard/Led'))
        self.leds.append(self.getDevice('RFoot/Led'))
        self.leds.append(self.getDevice('LFoot/Led'))
        self.leds.append(self.getDevice('Face/Led/Right'))
        self.leds.append(self.getDevice('Face/Led/Left'))
        self.leds.append(self.getDevice('Ears/Led/Right'))
        self.leds.append(self.getDevice('Ears/Led/Left'))

        # get phalanx motor tags
        # the real Nao has only 2 motors for RHand/LHand
        # but in Webots we must implement RHand/LHand with 2x8 motors
        self.lphalanx = []
        self.rphalanx = []
        self.maxPhalanxMotorPosition = []
        self.minPhalanxMotorPosition = []
        for i in range(0, self.PHALANX_MAX):
            self.lphalanx.append(self.getDevice("LPhalanx%d" % (i + 1)))
            self.rphalanx.append(self.getDevice("RPhalanx%d" % (i + 1)))

            # assume right and left hands have the same motor position bounds
            self.maxPhalanxMotorPosition.append(self.rphalanx[i].getMaxPosition())
            self.minPhalanxMotorPosition.append(self.rphalanx[i].getMinPosition())

        # shoulder pitch motors
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")

        # # keyboard
        # self.keyboard = self.getKeyboard()
        # self.keyboard.enable(10 * self.timeStep)


    # sensors data print 
    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))
    # 
    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %.3f m,  right %.3f m' % (dist[0], dist[1]))
    
    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False
        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()




#-------------------------------------------------------#
# init, get the time step of the current world.
robot = nao6_define()
timestep = int(robot.getBasicTimeStep())
# test，测试，自定义输入目标位置
goal_Pos =[0,0,0]
# 传感器做平均滤波，每 de_av_num 个值，进行取平均
de_av_num = 5
# 调取 class Sensor_data_filter
av_filter = mai_filter_0224.Sensor_data_filter(de_av_num)
#-------------------------------------------------------#
def co_wait(duration):
    """
    function:  co_wait --- let the coding waiting when the robot's motion was finished
    
    args:
    - duration: input, waiting time
    """
    pre_time = robot.getTime()
    while robot.getTime() - pre_time < duration:
        robot.step(64)
#-------------------------------------------------------#
def calculate_r2p(current_position, target_position, robot_yaw):
    """
    Calculate robot to position information.

    Args:
    - current_position: The current position of the robot [x, y]
    - target_position: The target position [x, y]
    - robot_yaw: The yaw angle of the robot (in degrees)

    Returns:
    - r2p_info: Robot to position information [distance, angle_degrees, r2p_degrees]
    """
    # Preprocess the data
    current_position = [round(cur, 2) for cur in current_position]
    target_position  = [round(tar, 2) for tar in target_position]

    # Calculate the Euclidean distance between current and target positions
    vect_x = target_position[0] - current_position[0]
    vect_y = target_position[1] - current_position[1]
    distance = round(math.sqrt(vect_x ** 2 + vect_y ** 2), 2)

    # Calculate the angle between robot and target position (when considering robot as a point)
    agl_radians  = round(math.atan2(vect_y, vect_x), 2)
    agl_degrees  = round(math.degrees(agl_radians), 2)

    # Calculate the angle towards the target position
    r2p_degrees = agl_degrees - robot_yaw

    # Store the robot to position information
    r2p_info = [distance, agl_degrees, r2p_degrees]

    return r2p_info
#-------------------------------------------------------#
# function:  get_ro_yaw --- 
# Args:  ro_yaw --- robot's yaw
# 
def get_ro_yaw():
    rpy = robot.inertialUnit.getRollPitchYaw()
    yaw = math.degrees(rpy[2])
    yaw1 = mai_filter_0224.IMU_filter(yaw)
    print("yaw",yaw,'fil_yaw',yaw1)
    return yaw1
# 可以按照下面这个改进
def get_robot_yaw():
    """
    get the robot's yaw
    获取机器人的偏航角。

    Return:
    - yaw: 机器人的偏航角（单位：度）
    """
    # 获取机器人的姿态信息 radians
    rpy = robot.inertialUnit.getRollPitchYaw()
    # 提取偏航角并转换为度数
    yaw = rpy[2]
    return yaw
#-------------------------------------------------------#
# function:  filter_av5 --- mean 5 times inputs value,
# 减少波动


#-------------------------------------------------------#
# 规划动作与运动
## 测量角度与转向
### 度数分类：[-5,5]，不用转向
def cur_motion(goal_Pos):
    # getting the gps values, 可以分出去
    real_pos = robot.gps.getValues()
    real_pos = [round(re1, 2) for re1 in real_pos]

    # 
    pre_time = robot.getTime()
    print('----------------------------------' )
    print('----------[now: %.2fs]------------' %(pre_time))
    print('----------------------------------' )
    print('----------gps----------')
    print('position: [ x y z ] = [%.2f %.2f %.2f]' % (real_pos[0], real_pos[1], real_pos[2]))
    # get the info
    ro_yaw = get_ro_yaw()
    r2p_info = calculate_r2p(real_pos, goal_Pos, ro_yaw)
    print('----------r2p----------')
    print('r2p_info: [ distance, agl_degrees, r2p_degrees] = [%.2f %.2f %.2f]' % (r2p_info[0],r2p_info[1],r2p_info[2]))
    dis, theta = r2p_info[0], r2p_info[2]
    # 声明当前动作
    cur_action = None
    if theta < -180:
        theta += 360
    if theta > 180:
        theta -=360

    
    # 根据距离和角度选择动作
    if abs(theta) < 10 or cur_action == "turnLeft20" or cur_action == "turnRight20":
        if dis >= 0.5:
            robot.startMotion(robot.fad50)
            # robot.fad50.play()
            cur_action = "forwards50"
            # print('now motion:',cur_action)
            co_wait(3)
        elif 0.2 <= dis < 0.5:
            robot.startMotion(robot.forwards)
            # robot.forwards.play()

            cur_action = "forwards"
            # print('now motion:',cur_action)
            co_wait(1)
        else:
            cur_action = "standing"
            if robot.currentlyPlaying:
                robot.currentlyPlaying.stop()
            

    elif abs(theta) >= 10 and dis >= 0.3:
        # 正角度，左转向
        if theta >= 0:
            robot.startMotion(robot.turnLeft40)
            # robot.turnLeft20.play()
            cur_action = "turnLeft20"
            # print('now motion:',cur_action)
            # co_wait(3)
        # 负角度，右转向    
        if theta <= 0:
            robot.startMotion(robot.turnRight40)
            # robot.turnRight20.play()
            cur_action = "turnRight20"
            # print('now motion:',cur_action)
            # co_wait(3)   
    else:
        if robot.currentlyPlaying:
            robot.currentlyPlaying.stop()  # 停止当前动作
        cur_action = "no_turn_motion" 

    print('now motion:',cur_action)
    # print('currentlyPlaying', robot.currentlyPlaying)
    return cur_action

def ball_wld(distance):
    real_r_pos = robot.gps.getValues()
    real_r_pos = [round(re1, 2) for re1 in real_r_pos]
    # 需要机器人面向


def shoot_ges_Pos_pre(shoot_goal_Pos, ball_Pos, dis):
    '''
    ball_Pos --- [x,y,z], 足球世界坐标
    shoot_goal_Pos --- [x,y,z], 踢球目标位置
    output：
    ges_Pos_pre --[x,y]
    '''
    a = np.array(shoot_goal_Pos[0:2])
    b = np.array(ball_Pos[0:2])
    i = (b - a)/np.linalg.norm(b - a)*dis
    ges_Pos_pre = b + i 
    if ges_Pos_pre[1]>0:
        ges_Pos_pre[1] += 0.2
    elif ges_Pos_pre[1]<0:
        ges_Pos_pre[1] -= 0.2
    else:
        ges_Pos_pre[1] = ges_Pos_pre[1]
    return ges_Pos_pre.tolist()



def shoot_ges_motion(if_de_ball, ball_Pos, shoot_goal_Pos,status):
    '''
    if_de_ball --- boolean , 是否在机器人视野范围内，识别到足球，且获取足球坐标
    ball_Pos --- [x,y,z], 足球世界坐标
    shoot_goal_Pos --- [x,y,z], 踢球目标位置
    '''
    temp = "none"
    if if_de_ball ==1 and status == True:

        ges_Pos_pre = shoot_ges_Pos_pre(shoot_goal_Pos,ball_Pos,0.5) # 预先设定走到距离球0.5的位置
        print('ges_Pos_pre:', ges_Pos_pre)

        temp = cur_motion(ges_Pos_pre)
            
    else:
        print("stop")
        # robot.currentlyPlaying.stop()

    return temp 



        
def static_turn2goal(basic_pos, shoot_goal_Pos, ball_pos, ges_Pos_pre):
    temp = []

    # robot 2 goal theta
    r2g_grad = math.atan2((shoot_goal_Pos[1] - basic_pos[1]), (shoot_goal_Pos[0] - basic_pos[0]))
    r2b_grad = math.atan2((ball_pos[1] - basic_pos[1]), (ball_pos[0] - basic_pos[0]))
    PrePos2b_grad = math.atan2((shoot_goal_Pos[1] - ges_Pos_pre[1]), (shoot_goal_Pos[0] - ges_Pos_pre[0]))
    # trans to degree
    r2g_degree = math.degrees(r2g_grad)
    r2b_degree = math.degrees(r2b_grad)
    PrePos2b_degree = math.degrees(PrePos2b_grad)

    ro_yaw_degree = get_ro_yaw()

    rf2g_degree = r2g_degree - ro_yaw_degree
    rf2b_degree = r2b_degree - ro_yaw_degree
    PrePos_rf2b_degree = PrePos2b_degree - ro_yaw_degree

    print('r2g_degree',r2g_degree)
    print('ro_yaw_degree',ro_yaw_degree)
    print('rf2g_degree', rf2g_degree)
    print('r2b_degree', r2b_degree)
    print('rf2b_degree', rf2b_degree)
    print('PrePos_rf2b_degree',PrePos_rf2b_degree)

    degrees = PrePos_rf2b_degree

    # 微调motion
    if degrees < -180:
        degrees += 360
    if degrees > 180:
        degrees -=360

    if abs(degrees) <10:
        if robot.currentlyPlaying:
            robot.currentlyPlaying.stop()

    if degrees > 5 :
        
        robot.turnLeft20.play()
    
    if degrees < -5:
        
        robot.turnRight20.play()

    return 0
       


#-------------------------main--------------------------#
# motion test
# 确保动作稳定性，不会那么容易跌倒
# co_wait(2)
# robot.startMotion(robot.shoot)

# isOver = robot.turnRight20.isOver()
# getDuration = robot.turnRight20.getDuration()
# print(isOver, getDuration)

# co_wait(5)
# isOver = robot.turnRight20.isOver()
# getDuration = robot.turnRight20.getDuration()
# print(isOver, getDuration)

# robot.startMotion(robot.turnRight40)
# cur_motion(goal_Pos)
        
# robot.printUltrasoundSensors()


## shoot ges test
co_wait(3)
ball_Pos = [1., 1., 0]
shoot_goal_Pos = [0., 2., 0]
status = True
temp = None
#-------------------------loop--------------------------#

while robot.step(timestep) != -1:      
    # 测试 
    real_pos = robot.gps.getValues()
    # real_pos = [round(re1, 2) for re1 in real_pos]
    # print('----------gps----------')
    print('position: [ x y z ] = [%.2f %.2f %.2f]' % (real_pos[0], real_pos[1], real_pos[2]))
    # [dis, theta] = calculate_r2p(real_pos, goal_Pos)
    # print('r2p_info: [ distance , theta ] = [%.2f %.2f]' % (dis, theta))

    # 
    # rpy = robot.inertialUnit.getRollPitchYaw()
    # print('----------inertial unit----------')
    # print('roll/pitch/yaw: [%.4f %.4f %.4f]' % (rpy[0], rpy[1], rpy[2]))
    # print('roll/pitch/yaw: [%.4f %.4f %.4f]' % (math.degrees(rpy[0]), math.degrees(rpy[1]), math.degrees(rpy[2])))


    # fil_data = mai_filter_0224.IMU_filter(math.degrees(rpy[2]))
    # print('filter data1:', fil_data)
    # 
    # robot.turnLeft20.play()
    # cur_motion(goal_Pos)

    # ul sensor 
    # dist = []
    # for i in range(0, len(robot.us)):
    #      dist.append(robot.us[i].getValue())

    # print('-----ultrasound sensors-----')
    # print('left: %.3f m,  right %.3f m' % (dist[0], dist[1]))
    # print ul sensor 
    # print('-----ultrasound sensors av5-----')
    # # left sonar
    # av_filter.update_sensor_data(dist[0])
    # left_av = av_filter.calculate_average()
    # # right sonar
    # av_filter.update_sensor_data(dist[1])
    # right_av = av_filter.calculate_average()
    # print('left: %.3f m,  right %.3f m' % (left_av, right_av))
    print('statis ---', status)
    temp = shoot_ges_motion(1, ball_Pos, shoot_goal_Pos,status)
    print('temp---', temp)
    if temp == "no_turn_motion" or temp == "standing" or temp == "none":
        ges_Pos_pre = shoot_ges_Pos_pre(shoot_goal_Pos,ball_Pos,0.5) # 预先设定走到距离球0.5的位置
        static_turn2goal(real_pos, shoot_goal_Pos, ball_Pos, ges_Pos_pre)
        status = False
    else:
        print("sss")
        print("test")
    # static_turn2goal(real_pos, shoot_goal_Pos, ball_Pos)

    # cur_motion(ball_Pos)

    # pass




    

    

