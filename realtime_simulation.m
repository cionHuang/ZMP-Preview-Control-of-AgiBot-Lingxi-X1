%{
运动学仿真可视化与关节驱动数据保存
%}

global uLINK

% 加载机器人参数
X1_PARAMS;

% 加载数据
data = readtable('stairs.csv');

% 定义常量
pelvis_delta_Z = 0.6473743781-0.591; % 盆骨与质心相对Z值

% 初始化图形窗口（可调整大小）
figure('Position', [100 100 10000 1000], 'Resize', 'on');
grid on;
hold on;

% 创建VideoWriter对象
video = VideoWriter('stairs_paper.mp4', 'MPEG-4');
video.FrameRate = 30;  % 设置帧率
open(video);

% 初始化坐标轴设置
% 阶梯步行
xlim([-1.0, 6.0]);
ylim([-0.5, 0.5]);
zlim([-0.2, 3.0]);
daspect([1 1 1]);
view([0, 0]);

% % 平地步行
% xlim([-1.0, 2.0]);
% ylim([-2.0, 1.0]);
% zlim([-0.2, 2.0]);
% daspect([1 1 1]);
% view([45, 45]);

% 保存关节数据文件 
joint_data = table('Size',...
    [height(data) 13],...
    'VariableTypes',...
    {'double','double','double','double','double','double','double','double','double',...
     'double','double','double','double',},...
    'VariableNames',...
    {'Timestamp','LL1','LL2','LL3','LL4','LL5','LL6',...
     'RL1','RL2','RL3','RL4','RL5','RL6'});

% 给膝关节一个任意正角度，以使得解析法求解逆运动学能得到符合人类直观感受的膝关节位置
uLINK(LL4).q = deg2rad(40);
uLINK(RL4).q = deg2rad(40);
ForwardKinematics(1);

com_trajectory = [];

% 动画循环
for i = 1:height(data)
    % 设置盆骨位置
    uLINK(Pelvic).p = [data.X(i), data.Y(i), data.Z(i)+pelvis_delta_Z]';
    uLINK(Pelvic).R = rpy2rot(ToRad*0.0,ToRad*0.0,ToRad*(data.Left_YAW(i)+data.Right_YAW(i))/2);
    
    % 设置右脚目标位置
    R_foot_CAL.p = [data.Right_X(i), data.Right_Y(i), data.Right_Z(i)+3.3307119903E-02]';
    R_foot_CAL.R = rpy2rot(ToRad*data.Right_ROLL(i),ToRad*data.Right_PITCH(i),ToRad*data.Right_YAW(i));
    InverseKinematics_LM(RL6, R_foot_CAL);
    % 设置左脚目标位置
    L_foot_CAL.p = [data.Left_X(i), data.Left_Y(i), data.Left_Z(i)+3.3245602429E-02]';
    L_foot_CAL.R = rpy2rot(ToRad*data.Left_ROLL(i),ToRad*data.Left_PITCH(i),ToRad*data.Left_YAW(i));
    InverseKinematics_LM(LL6, L_foot_CAL);

    
    % 正运动学更新
    ForwardKinematics(1);

    % 保存每步关节驱动数据
    joint_data(i,:) = {data.Timestamp(i),rad2deg(uLINK(LL1).q),rad2deg(uLINK(LL2).q),rad2deg(uLINK(LL3).q),...
        rad2deg(uLINK(LL4).q),rad2deg(uLINK(LL5).q),rad2deg(uLINK(LL6).q),...
        rad2deg(uLINK(RL1).q),rad2deg(uLINK(RL2).q),rad2deg(uLINK(RL3).q),...
        rad2deg(uLINK(RL4).q),rad2deg(uLINK(RL5).q),rad2deg(uLINK(RL6).q)};

    % 保存质心轨迹
    com_trajectory = [com_trajectory,uLINK(Pelvic).p];
  
    % 清除并重绘
    cla;     
    % 绘制机器人
    DrawAllJoints(1);
    if i > 1
        plot3(com_trajectory(1,1:i), com_trajectory(2,1:i), com_trajectory(3,1:i), 'r-', 'LineWidth', 1.5);
    end
    % 捕获帧
    frame = getframe(gcf);
    writeVideo(video, frame);
    
    % 控制显示速度
    pause(0.001); 
end

% 保存数据
writetable(joint_data,'test.csv');

% 关闭视频文件
close(video);
hold off;