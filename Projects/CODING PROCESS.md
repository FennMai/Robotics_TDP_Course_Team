# CODING PROCESS

新建一个环境，重新熟悉相关坐标系

**nao_demo_python.py**--- 官方的demo

webots中，重新刷新程序，需要在菜单栏中'**bulid**'---'**clean**'---'**build**'

## Project

### 1. worlds

| name                   | include |                                         |
| ---------------------- | ------- | --------------------------------------- |
| robot_wld_20240221.wbt | 2 NAO   |                                         |
| robot_wld_20240310.wbt | 3 NAO   | 匹配mai_controller_20240223.py 测试使用 |



### 2. controllers

| name                       | include                                                      |
| -------------------------- | ------------------------------------------------------------ |
| mai_controller_20240208.py | 完成机器人到目标点行走，转向非常稳定                         |
| nao_forward_trial1.c       | 新做的手臂运动姿态，避免面对新增nao机器人时，僵尸手型运动不稳定 |
| mai_controller_20240223.py | shoot mode相关代码，并完成测试，记得补充笔记到def function   |



nao_forward_trial1.c

- 完成运动状态

![image-20240211210354716](D:\aGit\Robotics_TDP_Course_Team\Projects\CODING PROCESS.assets\image-20240211210354716.png)

**Virsion**:

![image-20240208162529567](D:\aGit\Robotics_TDP_Course_Team\Projects\CODING PROCESS.assets\image-20240208162529567.png)



## reference frame

pitch---中点是原点(0,0,0)

robot---在球场上，默认Z=0.34

football--- z = 0.08

## robot motion 

```python
# setloop()---循环
# 对动作 handWave 设置setloop
# 
self.handWave.setLoop(True)

# play()---播放motion
motion.play()
```

### motion files 仿真测试调用表

每个文件动作，均测量三次取平均值

| files name        | 行径距离(m)                            | 单动作消耗仿真时间(s) | 注释             |
| ----------------- | -------------------------------------- | --------------------- | ---------------- |
| Forwards50.motion | 0.44                                   | 7.1-7.6               | 速度感觉会快一点 |
| Forwards.motion   | 0.1                                    | 3.0-3.6               | 速度感觉慢一点   |
| TurnLeft20        | 0                                      | 3-3.5                 | 生成的           |
| TurnLeft40        |                                        |                       | 自带的           |
| shoot             | 机器人到球最远距离0.2m，此时球行径0.2m | 4.8                   |                  |

🔺**attetion**:

motion files 的动作，不太稳定，需要预先停500-1000，再进行动作

motion, 放到while 是可以一直走

|                           |      |      |
| ------------------------- | ---- | ---- |
| TurnLeft20_Updated.motion | 不行 |      |
| TurnLeft90.motion         | 不行 |      |
|                           |      |      |

如果60的motion直接删除一半，变成30度可行吗？

## robot sensors 调用

### 1 -- gps

```python
# 以gps为例子
# 先使能
self.gps = self.getDevice(self.getName() + "gps")
self.gps.enable(self.timeStep)

# 后续可以调用
real_pos = robot.gps.getValues()
```

### 2 -- InertialUnit 

The [InertialUnit](https://cyberbotics.com/doc/reference/inertialunit?tab-language=python#inertialunit) computes and returns its attitude (*roll*, *pitch* and *yaw* angles) with respect to a global coordinate system defined in the [WorldInfo](https://cyberbotics.com/doc/reference/worldinfo) node.

```python
output：roll/pitch/yaw: [1.67 0.30 0.33]

```

roll/pitch/yaw: [1.67 0.30 0.33]

滚转角（Roll）、俯仰角（Pitch）和偏航角（Yaw）

- `主要通过roll, x轴? （世界坐标系下）查看当前机器人的面朝向，例如，正面朝向（0，0），z = 1.57；x 与 y主要用来判断机器人是否保持平衡。`错了，应该用偏航角yaw

**真傻逼，数据不准确**

**turnLeft20**

| 朝向         | 观察真实角度 | 传感器传输角度 | 差值 | 耗时 |
| ------------ | ------------ | -------------- | ---- | ---- |
| 红，正向球门 | 0            | 0              |      |      |
|              | 45           | 25             | 20   |      |
|              | 90           | 75             | 15   | 16   |
|              | 135          | 128            | 7    |      |
|              | 180          | 175            | 5    |      |
|              | -135         | -120           | 15   |      |
|              | -90          | -70            | 20   |      |
|              | -45          | -23            | 22   |      |
|              | 0            | 0              | 0    |      |

 [0;45;90;135;180;-135;-90;-45;0]

 [0;25;75;128;175;-116;-70;-23;0]

[  0  20  15   7   5 -19 -20 -22   0]



![image-20240309191533335](D:\aGit\Robotics_TDP_Course_Team\Projects\CODING PROCESS.assets\image-20240309191533335.png)

![image-20240309191851312](D:\aGit\Robotics_TDP_Course_Team\Projects\CODING PROCESS.assets\image-20240309191851312.png)



![image-20240309195656813](D:\aGit\Robotics_TDP_Course_Team\Projects\CODING PROCESS.assets\image-20240309195656813.png)

### 3 -- 'Sonar/Left', 'Sonar/Right'

![image-20240223235036382](D:\aGit\Robotics_TDP_Course_Team\Projects\CODING PROCESS.assets\image-20240223235036382.png)

| 测试工况           | 显示情况                                                     | 备注                                                  |
| ------------------ | ------------------------------------------------------------ | ----------------------------------------------------- |
| 静态，距离2.5m     | left: 2.5500 m,  right 2.5500 m                              | 应该是无法识别                                        |
| 静态，距离2m       | left: 2.550 m,  right 2.550 m                                | 无法识别                                              |
| 静态，距离1.5m     | left: 1.425 m,  right 1.425 m                                | 可以识别，因为前方机器人有厚度，所以不一定完全是(0,0) |
| 静态，距离1m       | left: 2.550 m,  right 2.550 m                                | 无法识别？？？                                        |
| 静态，距离0.5m     | left: 2.550 m,  right 2.550 m                                | 无法识别？？？                                        |
| 动态，距离1.5m到1m | left: 2.550 m,  right 0.951 m     <br /> left: 0.987 m,  right 2.550 m | 左右分别识别                                          |

倘若3s内，从第一次处罚识别值开始，左右sonar分别获取大于2.55的dist值n次（可能这里要做一个计数算法，因为传感器读的很快，一次可以返回很多次值，或者改变传感器读取0.1s一次，），则开始对dist值做判断：1. 距离小于0.5m，转向避开（默认左转避开）；2.障碍物偏左边的（左边识别到的次数多于右边的），则右转；3. 左转同理

## def function

| function name | input    | output | discribe             |
| ------------- | -------- | ------ | -------------------- |
| calculate_r2p |          |        |                      |
| cur_motion    | goal_Pos |        | 计算并执行机器人动作 |
| co_wait       |          |        |                      |
|               |          |        |                      |

```python
# 可以获取实时仿真时间
pre_time = robot.getTime()

def co_wait(duration):
    """
    function:  co_wait --- let the coding waiting when the robot's motion was finished
    
    args:
    - duration: input, waiting time
    """
    pre_time = robot.getTime()
    while robot.getTime() - pre_time < duration:
        robot.step(64)

```



```python
# 初版 运动调试
# 20240221 存档
def cur_motion(goal_Pos):
    # getting the gps values
    real_pos = robot.gps.getValues()
    real_pos = [round(re1, 4) for re1 in real_pos]
    print('----------gps----------')
    print('position: [ x y z ] = [%.2f %.2f %.2f]' % (real_pos[0], real_pos[1], real_pos[2]))
    # get the info
    ro_yaw = get_ro_yaw()
    r2p_info = calculate_r2p(real_pos, goal_Pos, ro_yaw)
    print('r2p_info: [ distance, agl_degrees, r2p_degrees] = [%.2f %.2f %.2f]' % (r2p_info[0],r2p_info[1],r2p_info[2]))
    dis, theta = r2p_info[0], r2p_info[2]
    # 声明当前动作
    cur_action = None
    
    # 根据距离和角度选择动作
    if abs(theta) < 5:
        if dis >= 0.5:
            # robot.startMotion(robot.fad50)
            robot.fad50.play()
            cur_action = "forwards50"
            # robot.step(1000)  # 等待一段时间，确保机器人有足够时间执行动作
        elif 0.1 <= dis < 0.5:
            robot.startMotion(robot.forwards)
            robot.forwards.play()
            cur_action = "forwards"
            # robot.step(1000)  # 等待一段时间，确保机器人有足够时间执行动作
        elif dis < 0.1:
            cur_action = "standing"
            robot.currentlyPlaying.stop()  # 停止当前动作
        else:
            cur_action = "no_motion" 
    elif 5 <= abs(theta) <= 25:
        # 正角度，左转向
        if theta > 0:
            robot.turnLeft20.play()
            cur_action = "turnLeft20"
    elif 25 < abs(theta) <= 45:
        # 正角度，左转向
        if theta > 0:
            robot.turnLeft40.play()
            cur_action = "turnLeft40"
    else:
        cur_action = "no_turn_motion" 

    print('now motion:',cur_action)
```

### shoot_ges_Pos_pre:

1. 三角形得理清楚，可以画图加逻辑推理一下

2. 与球近距离（可以设一个圆圈区域），shoot的模式，强制执行模式，自由模式（根据yaw - [机器人与足球，机器人与目标，目标与足球角度]）得出来的值，进行模糊目标范围的射球动作
3. 用向量解决了




## Question And Diary

#### 20240209：🔺

- 建议有空的时候，可以优化一下此处。当前是握拳超前放下，下一步试着旋转一下胳膊。


- 同时，建议建立运动时的摆臂动作，or 学习一下例程里面的

#### 20240210：

- 又是一天

#### 20240211：

- 完成手臂动作

#### 20240212:

- 在执行动作后，最好添加一些等待时间，以便机器人有足够的时间执行动作，然后再次读取GPS数据并进行下一步的判断。否则，可能会出现连续读取GPS数据并尝试执行动作的情况，这可能导致动作不正确或不连贯

webots 的时间函数有问题

在while循环内使用，robot.step(3000), 会中断motion的执行，很奇怪。

之前编写的，好像在while外的不会 

#### 20240220:

- robot 往下放，theta测得角度是正
- 刚好，先从左转开始测量
- 问题，光用gps，无法清除机器人正面朝向与角度？
- 对sensors数据，全部采取round(data, 2) 处理：

  1. 方便计算与阅读
  2. 节省内存与计算时间？

#### 20240221：🔺

- co_wait可能还是有问题，因为pre_time = robot.getTime()获取当前时间，但代码基本上是一秒先跑完，再做仿真的，所以输入的参数duration time 不是两个动作中间等待的时间，而是从一开始计时，然后duration time 后，再执行下一段代码

  我的感觉是：c代码和python对读取的文件读取处理有些本质上的不同

- 动作无法停下来，robot.currentlyPlaying.stop()会直接报错，这个currentlyPlaying是什么值？

- 靠近目的地的时候，会一直抖动，动作一致在切换，如何改进？

  1. 平均滤波，判断条件不要用定值，做一个函数，上下5都可以的范围值
  2. 优化robot动作，做成闭环控制，**在webots 官网手册，node，motion里面，有关于判断如果动作完成的函数**
  3. 可以做一个判断变量，确定上一个动作一定要和这个动作不一样

#### 20240223：🔺

- nao还有用到哪些有用的传感器？如何检测足球和身前物体？红外？雷达？
- cur_motion：可以再加一个shoot的motion，加一个input值，命令是否是踢球，如果是，则到达目的地踢球，如果不是，则不踢

#### 20240306：

- 重新熟悉之前代码
- 改善代码

#### 20240307

- 查看 mai_filter_0224内，Sensor_data_filter已经写好。
- 还可以写一个存储数据，并到处csv
- [Nao机器人手册 (tj-work.github.io)](https://tj-work.github.io/NRA/)
- [nao机器人正逆运动学.pdf](file:///D:/1_Glasgow_study/1_Robotics_TDP/Semester2/nao机器人正逆运动学.pdf)
- p2line--- 有缺陷
- 实现

#### 20240309:

11:00am - 11:00pm

- shoot 相关
- 策略研讨

#### 20240310：

5:00pm-11:00pm

motion动作测试

| turn    | angle | time-s                           |
| ------- | ----- | -------------------------------- |
| left20  | 90    | 16，15                           |
| left40  | 90    | 7，7.5                           |
| left40  | 360   | 28                               |
| left90  | 360   | 20                               |
| left180 | 360   | 16cur_motion --- 转向优先，有bug |

交付完成