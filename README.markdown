# 基于ZMP预览控制的人形机器人步态模式生成器 / Gait Pattern Generator of Humanoid Robot Based on ZMP Preview Contorl

## 项目概述 / Project Overview

 
本项目是作者毕业设计的MATLAB代码部分。整体设计基于智元机器人开源项目灵犀X1的机械结构，采用ZMP（Zero Moment Point）预览控制算法生成平地和阶梯步行的步态模式。项目旨在为人形机器人步行控制提供有效的步态生成方法。

 
This project is the MATLAB code portion of author's graduation design. The overall design is based on the mechanical structure of the AgiBot open-source project Lingxi X1. It utilizes the ZMP (Zero Moment Point) preview control algorithm to generate gait patterns for walking on flat ground and stairs. The project aims to provide an effective gait generation method for humanoid robot walking control.

## 核心特性 / Core Features

 
**ZMP预览控制理论**：项目基于ZMP预览控制理论，确保机器人行走过程中的稳定性。


**ZMP Preview Control Theory**: The project is based on ZMP preview control theory to ensure stability during robot walking.

## 依赖 / Dependencies


**MATLAB环境**：项目代码在MATLAB环境中运行。建议使用较新版本以支持所有功能。


**MATLAB Environment**: The project code runs in a MATLAB environment. It is recommended to use a recent version to support all functionalities.

## 安装和设置 / Installation and Setup


1. **克隆仓库**：从GitHub或其他代码托管平台下载项目代码。  
   ```bash  
   git clone <https://github.com/cionHuang/ZMP-Preview-Control-of-AgiBot-Lingxi-X1.git>  
   ```  
2. **设置MATLAB路径**：将项目文件夹添加到MATLAB的搜索路径中。
3. **Adams联合仿真(可选)**:
- 简化灵犀模型与Adams配置:请联系作者
- 平地步行运行`X1_1.bin`
- 阶梯步行运行`X1_1_stairs.bin`

---
1. **Clone the Repository**: Download the project code from GitHub or another code hosting platform.  
   ```bash  
   git clone <https://github.com/cionHuang/ZMP-Preview-Control-of-AgiBot-Lingxi-X1.git>  
   ```  
2. **Set MATLAB Path**: Add the project folder to MATLAB's search path.
3. **Adams Co-simulation(Optional)**:
- Simplified Lingxi X1 and Adams configuration: Please contact the author.
- Walking on flat ground, run the `X1_1.bin` file.
- Walking on stairs, run the `X1_1_stairs.bin` file.

## 使用方法 / Usage
 
1. **步态模式生成**：  
   - 运行`main.m`文件，该文件是步态模式生成程序的入口。  
   - 根据提示或通过配置文件配置行走速度和步长等参数。
 
2. **运动学仿真可视化**：  
   - 运行`realtime_simulation.m`文件，进行运动学仿真和可视化。  
   - 观察机器人行走的动态效果，验证步态模式的正确性。
   - 捕获的关节驱动数据可以导入Adams中完成动力学仿真。

--- 
1. **Gait Pattern Generation**:  
   - Run the `main.m` file, which serves as the entry point for the gait pattern generation program.  
   - Configure parameters such as walking speed and step length as prompted or via a configuration file.  
2. **Kinematic Simulation Visualization**:  
   - Run the `realtime_simulation.m` file to perform kinematic simulation and visualization.  
   - Observe the dynamic effects of the robot's walking to verify the correctness of the gait patterns.
   - The joint motion data captured by this programme can be imported into Adams to run the dynamics simulation.

## 参考文献 / References

- Kajita S. *Humanoid Robots: 2nd Edition* [M]. Beijing: China Machine Press, 2024: 1-177.  
- Kajita S, Kanehiro F, Kaneko K, et al. *The 3D Linear Inverted Pendulum Mode: A Simple Modeling for a Biped Walking Pattern Generation* [C]//Proceedings 2001 IEEE/RSJ International Conference on Intelligent Robots and Systems. Expanding the Societal Role of Robotics in the Next Millennium (Cat. No. 01CH37180). IEEE, 2001, 1: 239-246.  
- Kajita S, Kanehiro F, Kaneko K, et al. *Biped Walking Pattern Generation by Using Preview Control of Zero-Moment Point* [C]//2003 IEEE International Conference on Robotics and Automation (Cat. No. 03CH37422). IEEE, 2003, 2: 1620-1626.

## 参考代码仓库 / Reference Code Repositories

- [IntroductionToHumanoidRobotics](https://github.com/s-kajita/IntroductionToHumanoidRobotics)  
- [ZMP_preview_control](https://github.com/chauby/ZMP_preview_control)
- [Lingxi X1](https://github.com/AgibotTech/agibot_x1_hardware)


**This project is licensed under the [MIT License](LICENSE).**