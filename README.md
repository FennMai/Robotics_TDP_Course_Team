# Robotics_TDP_Course_Team
 All files and working documents 



**Development_Sections：**

(27,Oct Meeting)

| sections              | responsible people |
| --------------------- | ------------------ |
| robots_motion_control | 麦智鑫，魏子睿     |
| robots_version        | 赵仡飞，武鑫圆     |
| robots_Strategy       | 张恒明，季朝煦     |

**Projects:**

* Basic_Project

  已搭建好足球场地与机器人的基础环境



**快速上手Webots**：

对Webots进行基础操作的讲解



**Requirement.xlsx**

1. 根据比赛要求，以list的形式，列出需要实现的基本需求，拓展需求，探索和优化的可能性管理表
2. 实时展示进展
3. 解释在工程代码内，所属的部分和调用方式，提升复用率

**Python指南**
1. 最快、最方便也是最清晰的python管理工具：anaconda，值得每个python使用者学习！这里推荐使用轻量版的工具miniconda，可以在以下网址找到完整指南：https://docs.anaconda.com/free/miniconda/index.html
2. 按照指南安装完成后，在anaconda Powershell prompt中启动miniconda（按下Windows键，然后键入“anaconda powershell”，应该会出现作为选项）。
3. 通过运行以下命令（当询问时，输入“Y”确认）为Webots创建一个新环境：
```
conda create --name webots python=3.8
conda activate webots
```
4. 在anaconda powershell prompt中，跳转至webot项目的根目录下（使用cd命令）
5. 使用以下命令对相关依赖进行安装：
```
pip install -r requirements.txt
```
6. 最后一步，在webot中设置使用我们创建的虚拟环境作为webot的默认python环境，进行如下操作：
7. 在anaconda Powershell prompt中输入如下命令以获取当前虚拟环境的python解释器路径：
```
Get-Command python | %{$_.Source}
```
8. 复制以上命令返回的内容，以我的环境为例，它是E:\softwares\Miniconda\envs\vision\python.exe。
打开Webots程序，在顶部菜单中选择“工具（tools）” -> “偏好（preferences）”，然后将上述路径粘贴到中间的“Python command”框中。

完成以上操作后，即可统一所有开发成员的python环境，保证所有人都可以运行仓库代码，且不会对现有python环境造成干扰。