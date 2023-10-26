# Webots 快速上手手册

Webots是一款专业的移动机器人仿真软件包。

本文旨在帮助快速上手，完成后续所需要的开发，并不会对Webots进行细节的讲解。如果想要详细了解该软件用法和功能细节，可到官网**[用户指南](https://cyberbotics.com/doc/guide/index)**

首先，我们将Webots内的任意一个内容称之为node

#### 1. 创建New World

新建一个World(.wbt)环境：

右上角”File---New---New World File"

文件存放位置尽可能全英文。

![](.\Webots 快速上手手册.assets\image-20231026152847276.png)

如下图所示：新建环境，包含4个nodes：![image-20231026153357358](.\Webots 快速上手手册.assets\image-20231026153357358.png)

| WorldInfo               | 世界信息 | 包含该构建的世界的基础信息内容          |
| ----------------------- | :------- | --------------------------------------- |
| Viewpoint               | 视角     | 可以调整观看仿真/建模时，你所观察的视角 |
| TexturedBackground      | 感知背景 | 不能删                                  |
| TexturedBackgroundLight | 光线     | 不能删                                  |

#### 2.导入模型与内容

导入已有的模型文件，如下图所示

<img src=".\Webots 快速上手手册.assets\image-20231026153700170.png" alt="image-20231026153700170" style="zoom: 67%;" />

直接在右上角**Find**中输入：soccer，即可找到过往的简易模型

我们选择其中 robocup--RobocupSoccerField，既可将足球场（已检查，符合课程要求的）导入

此处还可以将足球（RobocupSoccerBall）一并导入

<img src=".\Webots 快速上手手册.assets\image-20231026153806544.png" alt="image-20231026153806544" style="zoom: 67%;" />

继续在Find输入：nao，即可找到标准的NAO机器人（还需要确认该机器人参数与课程要求是否一直）

<img src=".\Webots 快速上手手册.assets\image-20231026154604360.png" alt="image-20231026154604360" style="zoom:80%;" />

完成导入后，拖动视角，即可得到下图

![image-20231026155339465](.\Webots 快速上手手册.assets\image-20231026155339465.png)



Q: 物体如何进行快速移动摆放？

A: 鼠标左键选中该物体，然后按住Shift键，移动鼠标即可

#### 3.导入基础模板的世界

在Webots软件中，我们还发现其中已经带有Softbank建立好的非常非常简陋的足球比赛仿真工程。

操作：Webots 右上角的File---Open world

然后找到Webots的app下，\projects\robots\softbank\nao\worlds，即可找到 **nao_robocup.wbt**

如下图所示

![image-20231026160004590](.\Webots 快速上手手册.assets\image-20231026160004590.png)

<img src=".\Webots 快速上手手册.assets\image-20231026160332124.png" alt="image-20231026160332124"  />

完成以上步骤，即可开始对每个node进行仔细研究学习。