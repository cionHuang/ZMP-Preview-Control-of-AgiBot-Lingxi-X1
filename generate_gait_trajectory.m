%{
双足轨迹生成器
%}

function [left_traj, right_traj] = generate_gait_trajectory(footstep,N_step,init_rightfoot_pos, init_leftfoot_pos,lift_z, t_step,dt,height_stairs)

%% 初始化参数
total_time = N_step*t_step;
time_vector = 0:dt:total_time;
step_dt = t_step/dt;
t_segment = (0:1:step_dt)/step_dt;

% 初始化轨迹存储
left_pos = nan(length(time_vector),6);
right_pos = nan(length(time_vector),6);
left_vel = zeros(length(time_vector),6);
right_vel = zeros(length(time_vector),6);
left_acc = zeros(length(time_vector),6);
right_acc = zeros(length(time_vector),6);

% 设置初始位置
right_pos(1,:) = init_rightfoot_pos;
left_pos(1,:) = init_leftfoot_pos;

%% 步态相位分配
% 遍历预定义足迹点
for i=1:size(footstep,1)
    if footstep(i,7) == 1.0
        left_pos(round((i-1)*t_step/dt)+1,:) = footstep(i,1:6);
    elseif footstep(i,7) == 2.0
        right_pos(round((i-1)*t_step/dt)+1,:) = footstep(i,1:6);
    end
end

%% 生成左右脚轨迹（考虑到分别做初始脚的情况）
% 左脚
for i=step_dt+1:step_dt:size(left_pos,1)
    % 如果初始支撑脚不是左脚，则第一次t_step/dt的值应该为Nan,则复制初始左脚位姿即可
    if all(isnan(left_pos(i,:))) || isequal(left_pos(i,:),left_pos(i-step_dt,:))
        left_pos(i+1-step_dt:i,:) = repmat(left_pos(i-step_dt,:),step_dt, 1);
    else
    % 进行拟合插值
    [pos, vel, acc] = quintic_gait_trajectory(left_pos(i-step_dt,:), left_pos(i,:), lift_z, t_step, t_segment,height_stairs);
    % 赋值
    left_pos(i+1-step_dt:i-1,:) = pos(2:end-1,:);
    left_vel(i+1-step_dt:i-1,:) = vel(2:end-1,:);
    left_acc(i+1-step_dt:i-1,:) = acc(2:end-1,:);
    end
end
% 右脚
for i=step_dt+1:step_dt:size(right_pos,1)
    % 如果初始支撑脚是右脚，则第一次t_step/dt的值应该与初始位姿相同，直接复制初始右脚位姿即可
    if all(isnan(right_pos(i,:))) || isequal(right_pos(i,:),right_pos(i-step_dt,:))
        right_pos(i+1-step_dt:i,:) = repmat(right_pos(i-step_dt,:),step_dt, 1);
    else
    % 进行拟合插值
    [pos, vel, acc] = quintic_gait_trajectory(right_pos(i-step_dt,:), right_pos(i,:), lift_z, t_step, t_segment,height_stairs);
    % 赋值
    right_pos(i+1-step_dt:i-1,:) = pos(2:end-1,:);
    right_vel(i+1-step_dt:i-1,:) = vel(2:end-1,:);
    right_acc(i+1-step_dt:i-1,:) = acc(2:end-1,:);
    end
end

%% 构建输出结构体
left_traj = struct('time',time_vector', 'position',left_pos(:,1:6), 'velocity',left_vel(:,1:3), 'acceleration',left_acc(:,1:3));
right_traj = struct('time',time_vector', 'position',right_pos(:,1:6), 'velocity',right_vel(:,1:3), 'acceleration',right_acc(:,1:3));

%% 三维可视化
visualize_gait_3d(left_traj, right_traj, footstep);
end

%% 足部轨迹多项式拟合
function [pos, vel, acc] = quintic_gait_trajectory(start_pos, end_pos, h, T, tau,height_stairs)
% 输入参数：
%   start_pos - 起始位置[x,y,z]
%   end_pos - 终止位置[x,y,z]
%   h - 最大抬腿高度
%   T - 步态周期时间
%   t - 时间向量
%   height_stairs - 楼梯高度

% X方向五次多项式
dx = end_pos(1) - start_pos(1);
x = start_pos(1) + dx*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
vx = dx*(30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
ax = dx*(60*tau - 180*tau.^2 + 120*tau.^3)/T^2;

% Y方向五次多项式
dy = end_pos(2) - start_pos(2);
y = start_pos(2) + dy*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
vy = dy*(30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
ay = dy*(60*tau - 180*tau.^2 + 120*tau.^3)/T^2;

% 如果z高度不变，说明当前步为平地运动
if start_pos(3) == end_pos(3)
    % Z方向抬腿轨迹
    z = start_pos(3) + h*(16*tau.^2.*(1-tau).^2);
    vz = h*(32*tau.*(1-tau).^2 - 32*tau.^2.*(1-tau))/T;
    az = h*(32*(1-tau).^2 - 128*tau.*(1-tau) + 32*tau.^2)/T^2;
else
    delta_h = end_pos(3)-start_pos(3); % 每切换一次支撑脚，单腿要迈过台阶高度，并考虑安全系数
    if delta_h >0
        h = 1.15*delta_h;
            % 分为三个阶段：抬脚、跨越、落脚
        z = zeros(size(tau));
        % 抬脚阶段 (0 < tau < 0.3)
        phase1 = tau < 0.4;
        t1 = tau(phase1)/0.4; % 归一化到[0,1]
        z(phase1) = start_pos(3) + h * (3*t1.^2 - 2*t1.^3);   
        % 跨越阶段 (0.3 ≤ tau ≤ 0.6)
        phase2 = (tau >= 0.4) & (tau <= 0.6);
        z(phase2) = start_pos(3) + h;
        % 落脚阶段 (0.6 < tau ≤ 1)
        phase3 = tau > 0.6;
        t3 = (tau(phase3)-0.6)/0.4; % 归一化到[0,1]
        z(phase3) = end_pos(3)  + (start_pos(3) + h - end_pos(3)) * (1 - 10*t3.^3 + 15*t3.^4 - 6*t3.^5);
    else
        h = 0.1*abs(delta_h);
        z = zeros(size(tau));
        % 抬脚阶段 (0 < tau < 0.3)
        phase1 = tau < 0.4;
        t1 = tau(phase1) / 0.4; % 归一化到 [0,1]
        z(phase1) = start_pos(3) + h * (3*t1.^2 - 2*t1.^3);
        % 落脚阶段 (0.7 < tau ≤ 1)
        phase3 = tau >= 0.4;
        t3 = (tau(phase3) - 0.4) / 0.6; % 归一化到 [0,1]
        z(phase3) = end_pos(3) + (start_pos(3) + h - end_pos(3)) * (1 - 10*t3.^3 + 15*t3.^4 - 6*t3.^5);
    end
    % 计算速度和加速度
    vz = gradient(z, tau/T);
    az = gradient(vz, tau/T);
end

% roll方向五次多项式
d_roll = end_pos(4) - start_pos(4);
roll = start_pos(4) + d_roll*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
roll_vel = d_roll*(30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
roll_acc = d_roll*(60*tau - 180*tau.^2 + 120*tau.^3)/T^2;

% yaw方向五次多项式
d_yaw = end_pos(5) - start_pos(5);
yaw = start_pos(5) + d_yaw*(10*tau.^3 - 15*tau.^4 + 6*tau.^5);
yaw_vel = d_yaw*(30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
yaw_acc = d_yaw*(60*tau - 180*tau.^2 + 120*tau.^3)/T^2;

% 放置脚踝奇异位姿，踝关节pitch轨迹需要优化
pitch = zeros(1,length(tau));
pitch_vel = zeros(1,length(tau));
pitch_acc = zeros(1,length(tau));

% 单阶高度上楼
if abs(end_pos(3) - start_pos(3) - height_stairs) <1e-5
%     [pitch,pitch_vel,pitch_acc] = generate_pitch_trajectory(start_pos,end_pos,tau,T,20.0,0.05,0.55); % 楼梯
    [pitch,pitch_vel,pitch_acc] = generate_pitch_trajectory(start_pos,end_pos,tau,T,20.0,0.05,0.65); % 平地   
% 双阶高度上楼
elseif abs(end_pos(3) - start_pos(3) - 2*height_stairs) <1e-5
    [pitch,pitch_vel,pitch_acc] = generate_pitch_trajectory(start_pos,end_pos,tau,T,30.0,0.00,0.65);
% 单阶高度下楼
elseif abs(start_pos(3)-end_pos(3) - 0.5*height_stairs) <1e-5
    [pitch,pitch_vel,pitch_acc] = generate_pitch_trajectory(start_pos,end_pos,tau,T,20.0,0.05,0.55);
% 双阶高度下楼
elseif abs(start_pos(3)-end_pos(3) - height_stairs) <1e-5
    [pitch,pitch_vel,pitch_acc] = generate_pitch_trajectory(start_pos,end_pos,tau,T,22.0,0.00,0.65);
end

pos = [x' y' z' roll' yaw' pitch'];
vel = [vx' vy' vz' roll_vel' yaw_vel' pitch_vel'];
acc = [ax' ay' az' roll_acc' yaw_acc' pitch_acc'];
end

%% 三维可视化函数
function visualize_gait_3d(left_traj, right_traj, footstep)
figure('Position',[100 100 1200 800])

% 三维轨迹
subplot(2,2,[1,3]);
hold on;
% 绘制左脚轨迹
plot3(left_traj.position(:,1), left_traj.position(:,2), left_traj.position(:,3), 'b', 'LineWidth',1.5);
% 绘制右脚轨迹
plot3(right_traj.position(:,1), right_traj.position(:,2), right_traj.position(:,3), 'r', 'LineWidth',1.5);
% 绘制目标落脚点
scatter3(footstep(:,1), footstep(:,2), footstep(:,3), 100, 'k', 'filled');

% 装饰图形
xlabel('X（米）','FontSize',15,'FontName','宋体'); ylabel('Y（米）','FontSize',15,'FontName','宋体'); zlabel('Z（米）','FontSize',15,'FontName','宋体');
% title('3维双足轨迹','FontSize',15,'FontName','宋体');
legend('左脚', '右脚', 'ZMP目标点','FontSize',15,'FontName','宋体');
grid on;
% axis image;
pbaspect([1 1 0.15]);
view(-30,30)

% Z轴轨迹
subplot(2,2,2);
grid on;
hold on;
plot(left_traj.time, left_traj.position(:,3), 'b','LineWidth',1.5);
plot(right_traj.time, right_traj.position(:,3), 'r','LineWidth',1.5);
% title('法向轨迹','FontSize',15,'FontName','宋体');
xlabel(' 时间（秒）','FontSize',15,'FontName','宋体'); ylabel('高度（米）','FontSize',15,'FontName','宋体');
legend('左', '右','FontSize',15,'FontName','宋体');

% 速度曲线
subplot(2,2,4);
grid on;
hold on;
plot(left_traj.time, vecnorm(left_traj.velocity,2,2), 'b','LineWidth',1.5);
plot(right_traj.time, vecnorm(right_traj.velocity,2,2), 'r','LineWidth',1.5);
% title('合速度','FontSize',15,'FontName','宋体');
xlabel(' 时间（秒）','FontSize',15,'FontName','宋体'); ylabel(' 速度（米/秒）','FontSize',15,'FontName','宋体');
legend('左', '右','FontSize',15,'FontName','宋体');
end

function [pitch, pitch_vel, pitch_acc] = generate_pitch_trajectory(start_pos, end_pos, tau, T, max_offset_deg, offset_start, offset_end)
% 参数说明:
%   max_offset_deg: 最大偏移角度[°]
%   offset_start: 偏移开始时间点 (0~1)
%   offset_end:   偏移结束时间点 (0~1且>offset_start)
    
    % ========== 参数校验 ==========
    assert(offset_end > offset_start, '偏移结束时间必须大于开始时间');
    assert(all([offset_start, offset_end] >= 0 & [offset_start, offset_end] <= 1),...
           '时间参数需在[0,1]范围内');
    
    % ========== 单位转换 ==========
    max_offset = deg2rad(max_offset_deg);  % 转弧度
    
    % ========== 主逻辑 ==========
    d_pitch = end_pos(6) - start_pos(6);
%     z_dis = end_pos(3) -start_pos(3); % 如果是平地则不触发该优化
    
    if abs(d_pitch) < 1e-5
        % 预分配数组
        offset = zeros(size(tau));
        offset_vel = zeros(size(tau));
        offset_acc = zeros(size(tau));
        
        % 有效时间区段
        valid_mask = (tau >= offset_start) & (tau <= offset_end);
        valid_tau = tau(valid_mask);
        
        if ~isempty(valid_tau)
            % ===== 核心修正部分 =====
            % 时间归一化到[0,1]
            s = (valid_tau - offset_start)/(offset_end - offset_start);
            
            % 五次多项式系数（修正后的正确系数）
            a5 = 32 * max_offset;
            a4 = -64 * max_offset;
            a3 = 32 * max_offset;
            
            % ---------- 轨迹计算 ----------
            % 位置
            offset_segment = a5*s.^5 + a4*s.^4 + a3*s.^3;
            
            % 速度计算（包含时间缩放）
            time_scale = 1/((offset_end - offset_start)*T);  % 时间缩放因子
            offset_vel_segment = (5*a5*s.^4 + 4*a4*s.^3 + 3*a3*s.^2) .* time_scale;
            
            % 加速度计算
            offset_acc_segment = (20*a5*s.^3 + 12*a4*s.^2 + 6*a3*s) .* time_scale^2;
            
            % 赋值到有效区段
            offset(valid_mask) = offset_segment;
            offset_vel(valid_mask) = offset_vel_segment;
            offset_acc(valid_mask) = offset_acc_segment;
        end
        
        % 合成轨迹
        pitch = rad2deg(start_pos(6) + offset);
        pitch_vel = offset_vel;
        pitch_acc = offset_acc;
    else
        % 原标准轨迹生成代码保持不变
        pitch = rad2deg(start_pos(6) + d_pitch*(10*tau.^3 - 15*tau.^4 + 6*tau.^5));
        pitch_vel = d_pitch*(30*tau.^2 - 60*tau.^3 + 30*tau.^4)/T;
        pitch_acc = d_pitch*(60*tau - 180*tau.^2 + 120*tau.^3)/T^2;
    end
end















