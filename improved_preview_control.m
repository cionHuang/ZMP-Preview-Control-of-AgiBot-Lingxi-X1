%{
ZMP预览控制步态模式生成器
%}

function [ZMP_x_recorder,ZMP_y_recorder,COM_x_position,COM_y_position] = improved_preview_control(N_simulation,N_preview,ZMP_x_ref,ZMP_y_ref,A,B,C,Ks,Kx,G,dt)

% 初始化参数
ux = zeros(N_simulation,1);
uy = zeros(N_simulation,1);

% 默认质心从原点处出发
COM_x = zeros(3,N_simulation + 1);
COM_y = zeros(3,N_simulation + 1);


% 误差存放与累计
error_x = zeros(N_simulation,1);
error_y = zeros(N_simulation,1);

sum_error_x = 0;
sum_error_y = 0;

% 存放位置参数
COM_x_position = zeros(N_simulation,2);
COM_y_position = zeros(N_simulation,2);

for k = 1:N_simulation

    % 提取预览所需的所有ZMP
    ZMP_x_preview = ZMP_x_ref(k:k+N_preview-1)';
    ZMP_y_preview = ZMP_y_ref(k:k+N_preview-1)';

    % 计算当前状态下的实际ZMP
    ZMP_x = C*COM_x(:,k);
    ZMP_y = C*COM_y(:,k);
   
    % 记录实际ZMP位置
    ZMP_x_recorder(k) = ZMP_x(1);
    ZMP_y_recorder(k) = ZMP_y(1);

    % 计算误差
    error_x(k) = ZMP_x - ZMP_x_ref(k);
    error_y(k) = ZMP_y - ZMP_y_ref(k);
    sum_error_x = sum_error_x + error_x(k);
    sum_error_y = sum_error_y + error_y(k);

    % 更新输入u
    ux(k) = -Ks*sum_error_x - Kx*COM_x(:,k) - G*ZMP_x_preview;
    uy(k) = -Ks*sum_error_y - Kx*COM_y(:,k) - G*ZMP_y_preview;
    

    % 更新COM位置
    COM_x(:,k+1) = A*COM_x(:,k) + B*ux(k);
    COM_y(:,k+1) = A*COM_y(:,k) + B*uy(k);

    % 记录COM位置    
    COM_x_position(k,:) = [(k-1)*dt COM_x(1,k)];
    COM_y_position(k,:) = [(k-1)*dt COM_y(1,k)];
end
end