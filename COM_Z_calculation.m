%{
生成步行过程中质心轨迹
%}

function COM_z_position = COM_Z_calculation(left_position,right_position,height_stairs,t_step,dt,zc)

assert(height(left_position)==height(right_position),'左右脚规划轨迹时间不等');

% 初始化与参数计算
COM_z_position = zeros(height(left_position),1);
COM_z_position(1) = zc;

step_segment = t_step/dt;
step_vector = 1:1:step_segment;

height_vector = ((height_stairs/step_segment)*step_vector)';

% 下楼的高度变换曲线
height_vector_down = nonlinear_height_trajectory(step_segment,height_stairs,0.4,0.3);

flag_down = 0; % 标志位
flag_up = 0;

% 开始结算
for i=1+step_segment:step_segment:height(left_position)
    % 先判定是否处于上下楼状态
    if abs(left_position(i,3) - right_position(i,3)) <10e-8
        % 是否为上楼第一步
        if left_position(i,3) - left_position(i-step_segment,3) > 0 && flag_up ==0
            COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) + height_vector;
            flag_up =1;
            continue
        % 是否为下楼第一步
        elseif left_position(i,3) - left_position(i-step_segment,3) < 0 && flag_down ==0
            COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) - 0.5*height_vector_down; % 引入优化参数，避免下楼的奇异位姿
            flag_down = 1;
            continue
        elseif right_position(i,3) - right_position(i-step_segment,3) > 0 && flag_up ==0
            COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) + height_vector;
            flag_up =1;
            continue
        elseif right_position(i,3) - right_position(i-step_segment,3) < 0 && flag_down ==0
            COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) - 0.5*height_vector_down; % 引入优化参数，避免下楼的奇异位姿
            flag_down = 1;
            continue
        % 上楼最后一步
        elseif left_position(i,3) - left_position(i-step_segment,3) > 0 && flag_up ==1 
            COM_z_position(i+1-step_segment:i) = right_position(i,3) + zc;
            continue
        elseif right_position(i,3) - right_position(i-step_segment,3) > 0 && flag_up ==1
            COM_z_position(i+1-step_segment:i) = left_position(i,3) + zc;
            continue
        % 下楼最后一步
        elseif left_position(i,3) - left_position(i-step_segment,3) < 0 && flag_down ==1
            COM_z_position(i+1-step_segment:i) = right_position(i,3) + zc;
            continue
        elseif right_position(i,3) - right_position(i-step_segment,3) < 0 && flag_down ==1
            COM_z_position(i+1-step_segment:i) = left_position(i,3) + zc;
            continue
        % 平地预热
        else 
            COM_z_position(i+1-step_segment:i) = min(left_position(i,3),right_position(i,3)) + zc;
            continue
        end
    % 如果是上下楼过程中，再判定哪只脚为支撑脚
    elseif right_position(i,3) == right_position(i-step_segment,3) % 右脚高度没变，说明为支撑脚 
        % 上楼
        if left_position(i,3) - left_position(i-step_segment,3) > 0
            % 判断是否刚踏上楼梯
            if left_position(i,3) - left_position(i-step_segment,3) - height_stairs <= 10e-10
                COM_z_position(i+1-step_segment:i) = right_position(i,3) + zc; % 刚上楼时质心高度变化有滞后性
                continue
            else 
                COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) + height_vector;
                continue
            end
        % 下楼
        else
            % 判断是否刚踏下楼梯
            if left_position(i-step_segment,3) - left_position(i,3) - 0.5*height_stairs <= 10e-10
                COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) - 0.5*height_vector_down; % 下楼时质心高度立即变化
                flag_down =1;
                continue
            else
                COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) - 0.5*height_vector_down; % 下楼时质心高度立即变化
                continue
            end
        end

    elseif left_position(i,3) == left_position(i-step_segment,3) % 左脚高度没变，说明为支撑脚
        % 上楼
        if right_position(i,3) - right_position(i-step_segment,3) > 0
            % 判断是否刚踏上楼梯
            if right_position(i,3) - right_position(i-step_segment,3) - height_stairs <= 10e-10
                COM_z_position(i+1-step_segment:i) = left_position(i,3) + zc; % 刚上楼时质心高度变化有滞后性
                continue
            else 
                COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) + height_vector;
                continue
            end
        % 下楼
        else
            COM_z_position(i+1-step_segment:i) = COM_z_position(i-step_segment) - 0.5*height_vector_down; % 下楼时质心高度立即变化
            continue
        end
    end
end
end


function height_vector = nonlinear_height_trajectory(step_segment,height_stairs, turn_point_ratio, factor)
    % nonlinear_height_trajectory - Generate a nonlinear height trajectory for centroid z-coordinate
    % 
    % Syntax: height_vector = nonlinear_height_trajectory(t_step, dt, height_stairs, turn_point_ratio, factor)
    %
    % Inputs:
    %   t_step - Total time for the step
    %   dt - Time step
    %   height_stairs - Total height change
    %   turn_point_ratio - Ratio of step_vector where the transition occurs (0 to 1)
    %   factor - Factor determining the height at the transition point (0 to 1)
    %
    % Output:
    %   height_vector - Vector of height values over the step_vector


    % Calculate the transition point index
    k = round(turn_point_ratio * step_segment);

    height_at_k = height_stairs * factor;

    % Linear part before the transition point
    height_vector_linear = (height_at_k / k) * (1:k);

    % Exponential growth part after the transition point
    b = height_at_k;
    c = log(height_stairs / b) / (step_segment - k);

    height_vector_exp = b * exp(c * ((k+1:step_segment) - k));

    % Combine both parts
    height_vector = [height_vector_linear, height_vector_exp]';

    % Ensure the last point is exactly height_stairs
    height_vector(end) = height_stairs;
end
