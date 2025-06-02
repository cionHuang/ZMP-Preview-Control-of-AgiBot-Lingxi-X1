%{
步态规划主函数入口：
1.设定环境参数
2.设定控制参数
3.设定步态参数
4.获取预览参数
5.执行模式生成
6.保存执行数据
7.可视对比分析
%}

clc;
clear;

%% 设定约束平面
zc = 0.55 ; % m 平面截距，选取质心高度，可以调整。越大，响应越慢，但更稳定；越小，响应越快，但易震荡 初始状态COM_Z = 0.591m

%% 设定控制参数
dt = 0.01 ; % s 采样间隔，可以调整。不同的采样频率产生不同跟踪响应
t_step = 0.70 ; % s 走一步所需的时间，用以判断当前时间是双脚支撑期还是单脚支撑期
t_preview = 1.00 ; % s 预览总时间，决定了预览步数，可以调整
g = 9.81; % m/s^2 重力加速度

% 设定步态参数（支撑点、ZMP位置）
% % 平面步行的步态参数
% k_x = 0.00 ; % m 平面x斜率，与地形相关，可以调整
% k_y = 0.00 ; % m 平面y斜率，与地形相关，可以调整
% lift_z = 0.10; % m 抬脚高度
% height_stairs = 0;
% % 这里默认第一个支撑脚为右脚
% % 首先迈出左腿
% % 从静止态到下一个静止态，需要奇数步，以实现双脚并拢
% footstep = [ 0.0                0.0          0.0 0.0 0.0 0.0 0.0;
%              -5.451135E-04    -0.144664858  0.0 0.0 0.0 0.0 2.0; 
%              -5.576409E-04+0.25   0.1262158855 0.0 0.0 0.0 0.0 1.0; 
%              -5.451135E-04+0.50  -0.144664858  0.0 0.0 0.0 0.0 2.0; 
%              -5.576409E-04+0.75   0.1262158855 0.0 0.0 0.0 0.0 1.0; 
%              -5.451135E-04+0.75  -0.144664858  0.0 0.0 -15 0.0 2.0; 
%              0.9665 1.2158855*10e-3 0.0 0.0 -30 0.0 1.0; 
%              0.9267766953 -0.3214415533 0.0 0.0 -45 0.0 2.0;
%              1.0915 -0.2152841145 0.0 0.0 -60 0.0 1.0;
%              0.9914766953 -0.5629230099 0.0 0.0 -75 0.0 2.0;
%              1.2 -0.4652841145 0.0 0.0 -90 0.0 1.0;
%              1.0 -0.715 0.0 0.0 -90 0.0 2.0;
%              1.2 -0.965 0.0 0.0 -90 0.0 1.0;
%              1.0 -1.215 0.0 0.0 -90 0.0 2.0;
%              1.2 -1.465 0.0 0.0 -90 0.0 1.0;
%              1.0 -1.465 0.0 0.0 -90 0.0 2.0;
%              1.1 -1.465 0.0 0.0 -90 0.0 0.0;
%              1.1 -1.465 0.0 0.0 -90 0.0 0.0;
%             ]; % 预定义的ZMP位置X Y ；质心、脚步上升高度Z； 盆骨、脚步姿态roll yaw pitch ；左右脚标志位(0.0中立位 1.0左脚 2.0右脚)，这里隐含初始支撑脚 m deg

% 上下楼梯的步态参数
height_stairs = 0.10; % 台阶高度 m
step_distance = 0.24807; % 单步跨度 m
lift_z = 0.05; % 新的抬腿高度，考虑到了安全系数
k_x = height_stairs/0.15; % 更新约束平面斜率，此处的0.15是单步前进距离
k_y = 0.0;
footstep = [ 0.0                           0.0          0.0 0.0 0.0 0.0 0.0;
            -5.451135E-04                 -0.144664858  0.0 0.0 0.0 0.0 2.0; 
            -5.576409E-04+step_distance*1  0.1262158855 height_stairs*1 0.0 0.0 0.0 1.0; 
            -5.451135E-04+step_distance*2 -0.144664858  height_stairs*2 0.0 0.0 0.0 2.0; 
            -5.576409E-04+step_distance*3  0.1262158855 height_stairs*3 0.0 0.0 0.0 1.0; 
            -5.451135E-04+step_distance*4 -0.144664858  height_stairs*4 0.0 0.0 0.0 2.0; 
            -5.576409E-04+step_distance*5  0.1262158855 height_stairs*5 0.0 0.0 0.0 1.0; 
            -5.451135E-04+step_distance*6 -0.144664858  height_stairs*6 0.0 0.0 0.0 2.0; 
            -5.576409E-04+step_distance*7  0.1262158855 height_stairs*7 0.0 0.0 0.0 1.0; 
            -5.451135E-04+step_distance*8 -0.144664858  height_stairs*8 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*9  0.1262158855 height_stairs*9 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*9 -0.144664858  height_stairs*9 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*10  0.1262158855 height_stairs*9 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*11 -0.144664858  height_stairs*9 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*12  0.1262158855 height_stairs*9 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*12 -0.144664858  height_stairs*9 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*13  0.1262158855 height_stairs*8.5 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*14 -0.144664858  height_stairs*8 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*15  0.1262158855 height_stairs*7.5 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*16 -0.144664858  height_stairs*7 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*17  0.1262158855 height_stairs*6.5 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*18 -0.144664858  height_stairs*6 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*19  0.1262158855 height_stairs*5.5 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*20 -0.144664858  height_stairs*5 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*21  0.1262158855 height_stairs*5 0.0 0.0 0.0 1.0;
            -5.451135E-04+step_distance*22 -0.144664858  height_stairs*5 0.0 0.0 0.0 2.0;
            -5.576409E-04+step_distance*22  0.1262158855 height_stairs*5 0.0 0.0 0.0 1.0;
             0.0+step_distance*22 0.0 height_stairs*5 0.0 0.0 0.0 0.0;
             0.0+step_distance*22 0.0 height_stairs*5 0.0 0.0 0.0 0.0];

initial_leftfoot_pos = [-5.576409E-04 0.1262158855 0.0 0.0 0.0 0.0]; % 起步脚在世界坐标系中的绝对位姿,由仿真环境确定
initial_rightfoot_pos = [-5.451135E-04 -0.144664858 0.0 0.0 0.0 0.0];
initial_COM = [0.00 0.00   0.591]; % 初始状态下质心位置，由仿真环境确定

%% 计算仿真参数（区分仿真步数和行走步数的区别）
N_step = length(footstep); % 读取行走步数
t_simulation = N_step*t_step - t_preview - dt; % 仿真总时间，预留了预览时间，并对齐了时间步
N_preview = round(t_preview / dt); % 每次采样能观测的预览采样次数
N_simulation = round(t_simulation / dt); % 总仿真采样次数

% 读取确定时间下目标ZMP的位置，约束平面的斜率对运动的水平投影分量没有影响，所以无需考虑Z和姿态
ZMP_x_ref = [];
ZMP_y_ref = [];
k = 1;
for i=0:dt:N_step*t_step
    
    % 判断当前采样时间是否已涵盖一个完整的行走步，如果是的话，就可以开始存储下一步的信息
    if i ~=0 && mod(i,t_step) ==0
        k = k+1;
    end
    
    if height_stairs ~=0
        % 鉴于脚踝的奇异位姿，在这里将规划ZMP向内移动3cm，减少脚踝的roll变换量
        if k <= N_step
            if (footstep(k,7)-0.0)<10E-06
                theta = 0;
            elseif (footstep(k,7)-1.0)<10E-06
                theta = 1;
            elseif (footstep(k,7)-2.0)<10E-06
                theta = -1;
            end
    
            ZMP_x_ref = [ZMP_x_ref footstep(k,1)]; % 目标ZMP是瞬时跳跃变化的，所以在一个时间步内，其值不会变化
            ZMP_y_ref = [ZMP_y_ref theta*(abs(footstep(k,2))-0.04)];
        else
            break
        end
    else
        if k <= N_step
           ZMP_x_ref = [ZMP_x_ref footstep(k,1)]; % 目标ZMP是瞬时跳跃变化的，所以在一个时间步内，其值不会变化
           ZMP_y_ref = [ZMP_y_ref footstep(k,2)];
        else
            break
        end
    end
end

%% 定义状态方程各参数矩阵
A = [1 dt dt^2/2;
     0 1 dt;
     0 0 1]; % 状态转移矩阵

B = [dt^3/6;
     dt^2/2;
     dt]; % 输入矩阵

C = [1 0 -zc/g]; % 输出矩阵

% 评价函数的参数，可以调整
Q = 1.00;
R = 1e-6;

%% 计算预览控制参数
% 改进预览控制法（考虑历史误差）
[Ks,Kx,G] = improved_preview_control_params(A,B,C,Q,R,N_preview);

%% 仿真计算
[ZMP_x_recorder,ZMP_y_recorder,COM_x_position,COM_y_position] = improved_preview_control(N_simulation,N_preview,ZMP_x_ref,ZMP_y_ref,A,B,C,Ks,Kx,G,dt);

visualize_preview_control(ZMP_x_ref,ZMP_y_ref,ZMP_x_recorder,ZMP_y_recorder,COM_x_position(:,2),COM_y_position(:,2));

% 生成足部轨迹。
[left_traj, right_traj] = generate_gait_trajectory(footstep,N_step,initial_rightfoot_pos,initial_leftfoot_pos,lift_z,t_step,dt,height_stairs);
%% 保存COM、左右脚轨迹
left_time_position = [left_traj.time left_traj.position];
right_time_position = [right_traj.time right_traj.position];

% 由于质心的约束平面斜率对运动并无影响，所以将质心的Z值求解与足部的抬升联系起来，在此结算
COM_z_position = COM_Z_calculation(left_traj.position(:,1:3),right_traj.position(:,1:3),height_stairs,t_step,dt,zc);

mergeAndExportData(COM_x_position,COM_y_position,COM_z_position,left_time_position,right_time_position,'stairs.csv');












