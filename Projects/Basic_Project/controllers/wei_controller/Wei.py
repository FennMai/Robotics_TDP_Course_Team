"""Wei controller."""

# You may need to import some classes of the controller module. Ex:
# 

import numpy
import math
import time

from controller import Robot, Motion, Accelerometer, Camera, Motor, Node, TouchSensor


class Nao (Robot):
    PHALANX_MAX = 8
    ZERO_ANGLE  = 1.5828
    IS_GO = True
    IS_FALL = False
    Flag = True
    ARRIVED = False
    ISDETECTING = False
    Destinations = [
        [[-2.6, -2.00], [3.4, 2.62]],
        [[-2.6, -2.00], [0.4, -0.4]],
        [[-2.6, -2.00], [-2.4, -3.2]],
        [[-1.8, -1.4], [-3.2, -3.47]],
        [[1.4, 1.8], [-3.2, -3.47]],
        [[2.1, 2.4], [-2.4, -3.2]],
        [[2.1, 2.4], [0.4, -0.4]],
        [[2.1, 2.4], [3.4, 2.62]]
    ]

    # load motion files
    def loadMotionFiles(self):
        self.handWave = Motion('../../motions/HandWave.motion')
        self.forwards = Motion('../../motions/Forwards.motion')
        self.forwards = Motion('../../motions/Forwards50.motion')
        self.backwards = Motion('../../motions/Backwards.motion')
        self.shoot = Motion('../../motions/Shoot.motion')
        self.sideStepLeft = Motion('../../motions/SideStepLeft.motion')
        self.sideStepRight = Motion('../../motions/SideStepRight.motion')
        self.turnLeft60 = Motion('../../motions/TurnLeft60.motion')
        self.turnRight60 = Motion('../../motions/TurnRight60.motion')
        self.taiChi = Motion('../../motions/TaiChi.motion')
        self.wipeForhead = Motion('../../motions/WipeForehead.motion')
        self.standup = Motion('../../motions/StandUpFromFront.motion')

    # 如果当前存在动作则停止，同时对于不同的动作设定不同的系统休眠时间，保证动作不被立刻打断
    def startMotion(self, motion):
        # interrupt current motion
        if self.currentlyPlaying:
            self.currentlyPlaying.stop()
	
        # start new motion
        motion.play()
        self.currentlyPlaying = motion
    
        # 前进动作休眠1.5秒
        if motion == self.forwards:
            time.sleep(1.5)
        # 横向移动休眠2秒
        if motion == self.sideStepLeft or \
            motion == self.sideStepRight:
            time.sleep(2)
            self.IS_GO = True

    # 获取目标区域的中心点坐标
    def getCenter(self, destID):
        # 计算x坐标
        x = (self.Destinations[destID - 1][0][0] + self.Destinations[destID - 1][0][1]) / 2
        # 计算y坐标（z坐标）
        y = (self.Destinations[destID - 1][1][0] + self.Destinations[destID - 1][1][1]) / 2
        center = [x, y]
        return center   

    # 使用加速度计进行摔倒检测，判定为摔倒则摔倒标志IS_FALL = True
    def fallDetectA(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))
        
        if self.currentlyPlaying is None and abs(acc[0]) > abs(acc[1]) \
        and abs(acc[0]) > abs(acc[2]) and abs(acc[0]) > 9 and abs(acc[2]) < 3:
            self.IS_FALL = True
            
    # 使用足部触觉传感器进行摔倒检测，判定为摔倒则摔倒标志IS_FALL = True
    def fallDetectF(self):
        fsv = []  # force sensor values

        fsv.append(self.fsr[0].getValues())
        fsv.append(self.fsr[1].getValues())

        left = []
        right = []

        # The coefficients were calibrated against the real
        # robot so as to obtain realistic sensor values.
        left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Front Left
        left.append(fsv[0][2] / 3.4 + 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Front Right
        left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] - 1.15 * fsv[0][1])  # Left Foot Rear Right
        left.append(fsv[0][2] / 3.4 - 1.5 * fsv[0][0] + 1.15 * fsv[0][1])  # Left Foot Rear Left

        right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Front Left
        right.append(fsv[1][2] / 3.4 + 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Front Right
        right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] - 1.15 * fsv[1][1])  # Right Foot Rear Right
        right.append(fsv[1][2] / 3.4 - 1.5 * fsv[1][0] + 1.15 * fsv[1][1])  # Right Foot Rear Left

        for i in range(0, len(left)):
            left[i] = max(min(left[i], 25), 0)
            right[i] = max(min(right[i], 25), 0)
        
        if self.currentlyPlaying is None and \
        (((round(left[0], 1)<2.0)and(round(left[1], 1)<2.0)and(round(right[0], 1)<2.0)and(round(right[1], 1)<2.0)) \
        or ((round(left[3], 1)<2.0)and(round(left[2], 1)<2.0)and(round(right[3], 1)<2.0)and(round(right[2], 1)<2.0))) :
            self.IS_FALL = True
        
        print('----------foot sensors----------')
        print('+ left ---- right +')
        print('+-------+ +-------+')
        print('|' + str(round(left[0], 1)) +
              '  ' + str(round(left[1], 1)) +
              '| |' + str(round(right[0], 1)) +
              '  ' + str(round(right[1], 1)) +
              '|  front')
        print('| ----- | | ----- |')
        print('|' + str(round(left[3], 1)) +
              '  ' + str(round(left[2], 1)) +
              '| |' + str(round(right[3], 1)) +
              '  ' + str(round(right[2], 1)) +
              '|  back')
        print('+-------+ +-------+')
    
    # 初始化胳膊
    def initialArms(self):
        Motor.setVelocity(self.LElbowRoll,3)
        Motor.setPosition(self.LElbowRoll,0)
        Motor.setVelocity(self.RElbowRoll,3)
        Motor.setPosition(self.RElbowRoll,0)
        
        Motor.setVelocity(self.LShoulderPitch,3)
        Motor.setPosition(self.LShoulderPitch,1.2)
        Motor.setVelocity(self.RShoulderPitch,3)
        Motor.setPosition(self.RShoulderPitch,1.2)
        
        Motor.setVelocity(self.LShoulderRoll,3)
        Motor.setPosition(self.LShoulderRoll,0.2)
        Motor.setVelocity(self.RShoulderRoll,3)
        Motor.setPosition(self.RShoulderRoll,-0.2)
        
        Motor.setVelocity(self.LElbowYaw,3)
        Motor.setPosition(self.LElbowYaw,0)
        Motor.setVelocity(self.RElbowYaw,3)
        Motor.setPosition(self.RElbowYaw,0)
    
    # 初始化头部
    def initialHead(self):
        Motor.setVelocity(self.HeadYaw,3)
        Motor.setPosition(self.HeadYaw,0)
        Motor.setVelocity(self.HeadPitch,3)
        Motor.setPosition(self.HeadPitch,0)
        
    # 初始化腿部
    def initialLegs(self):
        Motor.setVelocity(self.LHipRoll,3)
        Motor.setPosition(self.LHipRoll,0)
        Motor.setVelocity(self.RHipRoll,3)
        Motor.setPosition(self.RHipRoll,0)
        
        Motor.setVelocity(self.LAnkleRoll,3)
        Motor.setPosition(self.LAnkleRoll,0)
        Motor.setVelocity(self.RAnkleRoll,3)
        Motor.setPosition(self.RAnkleRoll,0)
        
        Motor.setVelocity(self.LHipPitch,3)
        Motor.setPosition(self.LHipPitch,0)
        Motor.setVelocity(self.RHipPitch,3)
        Motor.setPosition(self.RHipPitch,0)
        
        Motor.setVelocity(self.LKneePitch,1)
        Motor.setPosition(self.LKneePitch,0)
        Motor.setVelocity(self.RKneePitch,1)
        Motor.setPosition(self.RKneePitch,0)
    
    # 下蹲，重心下降
    def squat(self):
        Motor.setVelocity(self.LHipPitch,4)
        Motor.setPosition(self.LHipPitch,-0.8)
        Motor.setVelocity(self.RHipPitch,4)
        Motor.setPosition(self.RHipPitch,-0.8)
        
        Motor.setVelocity(self.LAnklePitch,1)
        Motor.setPosition(self.LAnklePitch,-0.5)
        Motor.setVelocity(self.RAnklePitch,1)
        Motor.setPosition(self.RAnklePitch,-0.5)
        
        Motor.setVelocity(self.LKneePitch,3)
        Motor.setPosition(self.LKneePitch,1)
        Motor.setVelocity(self.RKneePitch,3)
        Motor.setPosition(self.RKneePitch,1)
    
    # 张开手臂，防守
    def openArms(self):
        Motor.setVelocity(self.LShoulderRoll,3)
        Motor.setPosition(self.LShoulderRoll,1.3)
        Motor.setVelocity(self.RShoulderRoll,3)
        Motor.setPosition(self.RShoulderRoll,-1.3)
        
        Motor.setVelocity(self.LShoulderPitch,3)
        Motor.setPosition(self.LShoulderPitch,0)
        Motor.setVelocity(self.RShoulderPitch,3)
        Motor.setPosition(self.RShoulderPitch,0)
    
    # 举起手臂，向队友要球
    def raiseArms(self):
        Motor.setVelocity(self.LShoulderRoll,3)
        Motor.setPosition(self.LShoulderRoll,0.2)
        Motor.setVelocity(self.RShoulderRoll,3)
        Motor.setPosition(self.RShoulderRoll,-0.2)
        
        Motor.setVelocity(self.LShoulderPitch,3)
        Motor.setPosition(self.LShoulderPitch,-1.3)
        Motor.setVelocity(self.RShoulderPitch,3)
        Motor.setPosition(self.RShoulderPitch,-1.3)
    
    # 弯曲收回手臂
    def bendArms(self):
        Motor.setVelocity(self.LShoulderRoll,3)
        Motor.setPosition(self.LShoulderRoll,0.2)
        Motor.setVelocity(self.RShoulderRoll,3)
        Motor.setPosition(self.RShoulderRoll,-0.2)
        
        Motor.setVelocity(self.LShoulderPitch,3)
        Motor.setPosition(self.LShoulderPitch,1)
        Motor.setVelocity(self.RShoulderPitch,3)
        Motor.setPosition(self.RShoulderPitch,1)
        
        Motor.setVelocity(self.LElbowYaw,3)
        Motor.setPosition(self.LElbowYaw,-1.3)
        Motor.setVelocity(self.RElbowYaw,3)
        Motor.setPosition(self.RElbowYaw,1.3)
        
        Motor.setVelocity(self.LElbowRoll,3)
        Motor.setPosition(self.LElbowRoll,1.5)
        Motor.setVelocity(self.RElbowRoll,3)
        Motor.setPosition(self.RElbowRoll,1.5)
    
    # 头部的上下左右，方便观测
    def headLeft(self):
        Motor.setVelocity(self.HeadYaw,3)
        Motor.setPosition(self.HeadYaw,2)
        
    def headRight(self):
        Motor.setVelocity(self.HeadYaw,3)
        Motor.setPosition(self.HeadYaw,-2)   
        
    def headUp(self):
        Motor.setVelocity(self.HeadPitch,3)
        Motor.setPosition(self.HeadPitch,-0.1)  
    
    def headDown(self):
        Motor.setVelocity(self.HeadPitch,3)
        Motor.setPosition(self.HeadPitch,0.2)  
    
    # the accelerometer axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printAcceleration(self):
        acc = self.accelerometer.getValues()
        print('----------accelerometer----------')
        print('acceleration: [ x y z ] = [%f %f %f]' % (acc[0], acc[1], acc[2]))

    # the gyro axes are oriented as on the real robot
    # however the sign of the returned values may be opposite
    def printGyro(self):
        vel = self.gyro.getValues()
        print('----------gyro----------')
        # z value is meaningless due to the orientation of the Gyro
        print('angular velocity: [ x y ] = [%f %f]' % (vel[0], vel[1]))

    def printGps(self):
        p = self.gps.getValues()
        print('----------gps----------')
        print('position: [ x y z ] = [%f %f %f]' % (p[0], p[1], p[2]))

    # the InertialUnit roll/pitch angles are equal to naoqi's AngleX/AngleY
    def printInertialUnit(self):
        rpy = self.inertialUnit.getRollPitchYaw()
        print('----------inertial unit----------')
        print('roll/pitch/yaw: [%f %f %f]' % (rpy[0], rpy[1], rpy[2]))

    def printUltrasoundSensors(self):
        dist = []
        for i in range(0, len(self.us)):
            dist.append(self.us[i].getValue())

        print('-----ultrasound sensors-----')
        print('left: %f m, right %f m' % (dist[0], dist[1]))

    def printCameraImage(self, camera):
        scaled = 2  # defines by which factor the image is subsampled
        width = camera.getWidth()
        height = camera.getHeight()

        # read rgb pixel values from the camera
        image = camera.getImage()

        print('----------camera image (gray levels)---------')
        print('original resolution: %d x %d, scaled to %d x %f'
              % (width, height, width / scaled, height / scaled))

        for y in range(0, height // scaled):
            line = ''
            for x in range(0, width // scaled):
                gray = camera.imageGetGray(image, width, x * scaled, y * scaled) * 9 / 255  # rescale between 0 and 9
                line = line + str(int(gray))
            print(line)

    def findAndEnableDevices(self):
        # get the time step of the current world.
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
    
        # motors        
        self.LElbowRoll = self.getDevice("LElbowRoll")
        self.RElbowRoll = self.getDevice("RElbowRoll")
        self.LElbowYaw = self.getDevice("LElbowYaw")
        self.RElbowYaw = self.getDevice("RElbowYaw")
        self.LHipYawPitch = self.getDevice("LHipYawPitch")
        self.RHipYawPitch = self.getDevice("RHipYawPitch")
        self.LHipRoll = self.getDevice("LHipRoll")
        self.RHipRoll = self.getDevice("RHipRoll")
        self.LHipPitch = self.getDevice("LHipPitch")
        self.RHipPitch = self.getDevice("RHipPitch")
        self.LKneePitch = self.getDevice("LKneePitch")
        self.RKneePitch = self.getDevice("RKneePitch")
        self.LAnklePitch = self.getDevice("LAnklePitch")
        self.RAnklePitch = self.getDevice("RAnklePitch")
        self.LAnkleRoll = self.getDevice("LAnkleRoll")
        self.RAnkleRoll = self.getDevice("RAnkleRoll")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")
        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.HeadYaw = self.getDevice("HeadYaw");
        self.HeadPitch = self.getDevice("HeadPitch");

    def __init__(self):
        Robot.__init__(self)
        self.currentlyPlaying = False

        # initialize stuff
        self.findAndEnableDevices()
        self.loadMotionFiles()

    def run(self):
        self.handWave.setLoop(True)
        self.handWave.play()
        self.currentlyPlaying = self.handWave

# create the Robot instance and run main loop
robot = Nao()
robot.run()
