%{ 
合并数据并处理NaN，截断行数，保存为CSV
输入参数:
COM_x, COM_y, COM_z: ~×2 double，列1时间戳，列2坐标
left, right: ~×7 double，列1时间戳，列2-7三维位姿（可能含NaN）
outputFileName: 输出文件名（如 'merged_data.csv'）
%}

function mergeAndExportData(COM_x, COM_y,COM_z,left, right, outputFileName)
    % 确保输入数据列数正确
    assert(size(COM_x,2) == 2 && size(COM_y,2) == 2 , 'COM数据应为2列');
    assert(size(left,2) == 7 && size(right,2) == 7, 'left/right数据应为4列');

    % 截断所有数据到最小行数
    min_rows = min([size(COM_x,1), size(COM_y,1), size(COM_z,1),size(left,1), size(right,1)]);
    COM_x = COM_x(1:min_rows, :);
    COM_y = COM_y(1:min_rows, :);
    COM_z = COM_z(1:min_rows, :);
    left = left(1:min_rows, :);
    right = right(1:min_rows, :);

    % 提取时间戳（以COM_x的时间为准）
    timestamp = COM_x(:, 1);
    x = COM_x(:, 2);
    y = COM_y(:, 2);
    z = COM_z(:);

    % 处理left和right的NaN（前向填充）
    left_coords = fillmissing(left(:, 2:7), 'previous', 1); % 按行填充
    right_coords = fillmissing(right(:, 2:7), 'previous', 1);

    % 合并最终数据矩阵
    merged_data = [timestamp, x, y, z, left_coords, right_coords];

    % 生成列标题
    headers = {'Timestamp', 'X', 'Y', 'Z' ...
               'Left_X', 'Left_Y', 'Left_Z', 'Left_ROLL', 'Left_YAW', 'Left_PITCH'...
               'Right_X', 'Right_Y', 'Right_Z', 'Right_ROLL', 'Right_YAW', 'Right_PITCH'};

    % 保存为CSV文件（带标题）
    writetable(array2table(merged_data, 'VariableNames', headers), outputFileName);
    disp(['数据已保存至: ' outputFileName]);
end