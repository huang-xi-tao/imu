data = readtable('/home/hxt/Desktop/algorithm/motor_test/new_old_motor/new_motor_data.csv');
data1 = readtable('/home/hxt/Desktop/algorithm/motor_test/new_old_motor/old_motor_data.csv');

figure;
subplot(2,1,1);
hold on;
plot(data.Time, data.out_torque,'DisplayName','out','Color','r', 'LineWidth', 1.5);
plot(data.Time, data.in_torque, 'DisplayName','in','Color','b', 'LineWidth', 1.5);
% 添加标题和标签
title('new torque');
xlabel('Time');
ylabel('torque');
% 添加图例
legend;
grid on;
subplot(2,1,2);
hold on;
plot(data1.Time, data1.out_torque,'DisplayName','out','Color','r', 'LineWidth', 1.5);
plot(data1.Time, data1.in_torque, 'DisplayName','in','Color','b', 'LineWidth', 1.5);
% 添加标题和标签
title('old torque');
xlabel('Time');
ylabel('torque');
% 添加图例
legend;
grid on;