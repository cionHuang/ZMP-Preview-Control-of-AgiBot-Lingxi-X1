%{
可视化预览控制的生成数据
%}

function visualize_preview_control(ZMP_x_ref,ZMP_y_ref,ZMP_x_record, ZMP_y_record,COM_x_record, COM_y_record)
figure;
% 绘制X方向数据对比
% subplot(3,1,1);
grid on;
hold on;
plot(ZMP_x_ref, 'k--','LineWidth',1.5, 'DisplayName', 'ZMP_x目标位置');
plot(ZMP_x_record, 'r', 'LineWidth',1.5,'DisplayName', 'ZMP_x实际位置');
plot(COM_x_record, 'b','LineWidth',1.5,'DisplayName', 'COM_x实际位置');
hold off;
xlabel('采样点','FontSize',15,'FontName','宋体');  % 设置x轴名称
ylabel('位置（米）','FontSize',15,'FontName','宋体');  % 设置y轴名称
legend('Location', 'best','FontSize',15,'FontName','宋体');
% title('X方向轨迹对比','FontSize',15,'FontName','宋体');

figure;
% 绘制Y方向数据对比
% subplot(3,1,2);
grid on;
hold on;
plot(ZMP_y_ref, 'k--','LineWidth',1.5, 'DisplayName', 'ZMP_y目标位置');
plot(ZMP_y_record, 'r', 'LineWidth',1.5,'DisplayName', 'ZMP_y实际位置');
plot(COM_y_record, 'b', 'LineWidth',1.5, 'MarkerSize',3,'DisplayName', 'COM_y实际位置');
hold off;
xlabel('采样点','FontSize',15,'FontName','宋体');  % 设置x轴名称
ylabel('位置（米）','FontSize',15,'FontName','宋体');  % 设置y轴名称
legend('Location', 'best','FontSize',15,'FontName','宋体');
% title('Y方向轨迹对比','FontSize',15,'FontName','宋体');

figure;
% 绘制2D平面轨迹对比
% subplot(3,1,3);
grid on;
hold on;
plot(ZMP_x_ref, ZMP_y_ref, 'k--', 'LineWidth',1.5,'DisplayName', 'ZMP目标位置');
plot(ZMP_x_record, ZMP_y_record, 'r', 'LineWidth',1.5,'DisplayName', 'ZMP实际位置');
plot(COM_x_record, COM_y_record, 'b', 'LineWidth',1.5, 'MarkerSize',3,'DisplayName', 'COM实际位置');
axis equal;
hold off;
xlabel('采样点','FontSize',15,'FontName','宋体');  % 设置x轴名称
ylabel('位置（米）','FontSize',15,'FontName','宋体');  % 设置y轴名称
legend('Location', 'best','FontSize',15,'FontName','宋体');
% title('二维轨迹对比','FontSize',15,'FontName','宋体');
end

