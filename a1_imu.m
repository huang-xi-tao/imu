clc;%清理命令行窗口
clear;%清理工作区

imu_data = struct();

imu_data.init = false;
imu_data.gyro_bias = [0, 0, 0];
imu_data.update_cnt = 0;
imu_data.average_cnt = 0;
imu_data.rotation = [0, 0, 0];
imu_data.init_accl = [0, 0, 0];
imu_data.accl = [0, 0, 0];
imu_data.linear_accl = [0, 0, 0];


imu_data.w_accl = 0.6;
imu_data.w_gyro = 0.2;
imu_data.gyro_bias_max = 0.05;
imu_data.init_dt_accl = 0.001;
imu_data.i = 0;

imu_data.w = 1;
imu_data.x = 0;
imu_data.y = 0;
imu_data.z = 0;% 假设这是一个四元数类实例
imu_data.q_record = [0,0,0,0];  % 假设这是一个四元数类实例

data = readtable('/home/hxt/Desktop/algorithm/imu_calc/imu_data.csv');

figure;
subplot(2,1,1);
hold on;
plot(data.Time, data.accl0,'DisplayName','accl0','Color','r', 'LineWidth', 1.5);
plot(data.Time, data.accl1, 'DisplayName','accl1','Color','g', 'LineWidth', 1.5);
plot(data.Time, data.accl2, 'DisplayName','accl2','Color','b', 'LineWidth', 1.5);
% 添加标题和标签
title('accl');
xlabel('Time');
ylabel('accl');
% 添加图例
legend;
grid on;
subplot(2,1,2);
hold on;
plot(data.Time, data.rot0,'DisplayName','rot0','Color','r', 'LineWidth', 1.5);
plot(data.Time, data.rot1, 'DisplayName','rot1','Color','g', 'LineWidth', 1.5);
plot(data.Time, data.rot2, 'DisplayName','rot2','Color','b', 'LineWidth', 1.5);
% 添加标题和标签
title('accl');
xlabel('Time');
ylabel('accl');
% 添加图例
legend;
grid on;

for i = 1:size(data.Index)
gyro_meas = [data.rot0(i,:) data.rot1(i,:) data.rot2(i,:)];
accl_meas = [data.accl0(i,:) data.accl1(i,:) data.accl2(i,:)];
dt = 0.001;

imu_data = update_gyro(imu_data,gyro_meas,dt);
imu_data = update_accl(imu_data,gyro_meas,accl_meas,dt);
imu_data.q_record(i,:) = [imu_data.x imu_data.y imu_data.z imu_data.w];
imu_data.i = i;
end


figure;
subplot(2,1,1);
hold on;
plot(data.Time, data.x,'DisplayName','x','Color','r', 'LineWidth', 1.5);
plot(data.Time, data.y, 'DisplayName','y','Color','g', 'LineWidth', 1.5);
plot(data.Time, data.z, 'DisplayName','z','Color',[0.0,1.0,1.0], 'LineWidth', 1.5);
plot(data.Time, data.w, 'DisplayName','w','Color','b', 'LineWidth', 1.5);
title('quaternion data');
xlabel('Time');
ylabel('quaternion');
legend;
grid on;

subplot(2,1,2);
hold on;
title('quaternion data');
xlabel('Time');
ylabel('quaternion');
plot(data.Time, imu_data.q_record(:,1),'DisplayName','x','Color','r', 'LineWidth', 1.5);
plot(data.Time, imu_data.q_record(:,2), 'DisplayName','y','Color','g', 'LineWidth', 1.5);
plot(data.Time, imu_data.q_record(:,3), 'DisplayName','z','Color',[0.0,1.0,1.0], 'LineWidth', 1.5);
plot(data.Time, imu_data.q_record(:,4), 'DisplayName','w','Color','b', 'LineWidth', 1.5);

title('calculate quaternion data');
xlabel('Time');
ylabel('quaternion');
legend;
grid on;

w = data.w;
x = data.x;
y = data.y;
z = data.z;

     sinr_cosp = +2.0 * (w .* x + y .* z);
     cosr_cosp = +1.0 - 2.0 * (x .* x + y .* y);
    roll = atan2(sinr_cosp, cosr_cosp);

   
     sinp = +2.0 * (w .* y - z .* x);
    if abs(sinp) >= 1
        pitch = sign(sinp) * (pi / 2.0);
    else
        pitch = asin(sinp);
    end
    
     siny_cosp = +2.0 * (w .* z + x .* y);
     cosy_cosp = +1.0 - 2.0 * (y .* y + z .* z);
    yaw = atan2(siny_cosp, cosy_cosp);


figure;
subplot(1,1,1);
hold on;

plot(data.Time, yaw, 'DisplayName','calc\_yaw','Color','b', 'LineWidth', 1.5);
plot(data.Time, pitch,'DisplayName','calc\_pitch','Color','r', 'LineWidth', 1.5);
plot(data.Time, roll, 'DisplayName','calc\_roll','Color','g', 'LineWidth', 1.5);

title('calculate data');
xlabel('Time');
ylabel('calculate angle');

legend;
grid on;


%deal with imu data
function imu = update_gyro(imu, gyro_meas, dt)

    if ~imu.init
        if abs(gyro_meas(3)) > 0.06
            imu.gyro_bias = [0, 0, 0];
            imu.update_cnt = 0;
            imu.average_cnt = 0;
            return;
        end
        if imu.average_cnt < 1500
            imu.gyro_bias = imu.gyro_bias + gyro_meas / 1500;
            imu.average_cnt = imu.average_cnt + 1;
        end
        return;
    end
    
    rotation = gyro_meas - imu.gyro_bias;
    rotation(3) = deadzone(rotation(3), 0.01);
    
    dq = Quaternion(rotation, dt);
    
    imu.x =  imu.x * dq.w + imu.y * dq.z - imu.z * dq.y + imu.w * dq.x;
    imu.y = -imu.x * dq.z + imu.y * dq.w + imu.z * dq.x + imu.w * dq.y;
    imu.z =  imu.x * dq.y - imu.y * dq.x + imu.z * dq.w + imu.w * dq.z;
    imu.w = -imu.x * dq.x - imu.y * dq.y - imu.z * dq.z + imu.w * dq.w;
end

        
function imu = update_accl(imu,gyro_meas,accl_meas, dt)

    if ~imu.init
        norm_accl = norm(accl_meas);
        if norm_accl > 1.05 || norm_accl < 0.95
            return;
        end

        imu.init_accl = imu.init_accl + accl_meas;
        imu.update_cnt = imu.update_cnt + 1;

        if imu.update_cnt < 20
            return;
        end

        Reb = zeros(3, 3);
        Reb(3, :) = imu.init_accl / imu.update_cnt;%????????????
        Reb(3, :) = Reb(3, :) / norm(Reb(3, :));

        Reb(1, 1) = Reb(3, 3);
        Reb(1, 3) = -Reb(3, 1);
        Reb(1, :) = Reb(1, :) / norm(Reb(1, :));
        Reb(2, :) = cross(Reb(3, :), Reb(1, :));

        imu = rotation_matrix_to_quaternion(imu,Reb);

        if imu.update_cnt > 2000
            imu.init = true;
            imu.update_cnt = 2000;
        end
        return;

    else
        n_accl = norm(accl_meas);

        v2 = [2 * (imu.x * imu.z - imu.w* imu.y), ...
              2 * (imu.y * imu.z + imu.w * imu.x), ...
              1 - 2 * (imu.x * imu.x + imu.y * imu.y)];

        if n_accl < 1.05 && n_accl > 0.95
            norm_accel = accl_meas / n_accl;
            accel_corr = cross(norm_accel, v2);
            corr = accel_corr * imu.w_accl;
            if all(abs(gyro_meas) < 0.1)
                for i = 1:3
                    imu.gyro_bias(i) = imu.gyro_bias(i) - corr(i) * (imu.w_gyro * dt);
                    imu.gyro_bias(i) = max(min(imu.gyro_bias(i), imu.gyro_bias_max), - imu.gyro_bias_max);
                end
            end
        else
            return;
        end

      dq = Quaternion(corr, dt);

      imu.x =  imu.x * dq.w + imu.y * dq.z - imu.z * dq.y + imu.w * dq.x;
      imu.y = -imu.x * dq.z + imu.y * dq.w + imu.z * dq.x + imu.w * dq.y;
      imu.z =  imu.x * dq.y - imu.y * dq.x + imu.z * dq.w + imu.w * dq.z;
      imu.w = -imu.x * dq.x - imu.y * dq.y - imu.z * dq.z + imu.w * dq.w;

      imu.update_cnt = imu.update_cnt + 1;

        if imu.update_cnt > 5000
            imu.update_cnt = 5000;
        end
        return;
    end
end
%calculate dq
function q = Quaternion(w, dt)
    % Ensure w is a 3-element vector
    if length(w) ~= 3
        error('Input w must be a 3-element vector');
    end
    norm_w = sqrt(sum(w.^2));
    theta = norm_w * dt;
    
    if theta > 0.03
        sin_theta_2 = sin(theta / 2);
        q.x = (w(1) / norm_w) * sin_theta_2;
        q.y = (w(2) / norm_w) * sin_theta_2;
        q.z = (w(3) / norm_w) * sin_theta_2;
    else
        q.x = w(1) * (dt / 2);
        q.y = w(2) * (dt / 2);
        q.z = w(3) * (dt / 2);
    end
    
    q.w = 1 - q.x^2 - q.y^2 - q.z^2;

end

function input = deadzone(input, deadzone)
    if input > deadzone
        input = input - deadzone;
    elseif input < -deadzone
        input = input + deadzone;
    else
        input = 0;
    end
end

function imu_data = rotation_matrix_to_quaternion(imu_data,rotation_matrix)
    tr = rotation_matrix(1, 1) + rotation_matrix(2, 2) + rotation_matrix(3, 3);
    q = zeros(1, 4);

    if tr > 0.0
        s = sqrt(tr + 1.0);
        q(1) = s * 0.5;
        s = 0.5 / s;
        q(2) = (rotation_matrix(3, 2) - rotation_matrix(2, 3)) * s;
        q(3) = (rotation_matrix(1, 3) - rotation_matrix(3, 1)) * s;
        q(4) = (rotation_matrix(2, 1) - rotation_matrix(1, 2)) * s;
    else
        dcm_i = 1;
        for i = 2:3
            if rotation_matrix(i, i) > rotation_matrix(dcm_i, dcm_i)
                dcm_i = i;
            end
        end

        dcm_j = mod(dcm_i, 3) + 1;
        dcm_k = mod(dcm_i + 1, 3) + 1;
        s = sqrt((rotation_matrix(dcm_i, dcm_i) - rotation_matrix(dcm_j, dcm_j) - rotation_matrix(dcm_k, dcm_k)) + 1.0);
        q(dcm_i + 1) = s * 0.5;
        s = 0.5 / s;
        q(dcm_j + 1) = (rotation_matrix(dcm_i, dcm_j) + rotation_matrix(dcm_j, dcm_i)) * s;
        q(dcm_k + 1) = (rotation_matrix(dcm_k, dcm_i) + rotation_matrix(dcm_i, dcm_k)) * s;
        q(1) = (rotation_matrix(dcm_k, dcm_j) - rotation_matrix(dcm_j, dcm_k)) * s;
    end
    imu_data.w = q(1);
    imu_data.x = q(2);
    imu_data.y = q(3);
    imu_data.z = q(4);

    imu_data = normalize(imu_data);
end


function imu = normalize(imu)
    w = imu.w;
    x = imu.x;
    y = imu.y;
    z = imu.z;

    norm_q = sqrt(w^2 + x^2 + y^2 + z^2);
    if norm_q>0.000001
    imu.w = w / norm_q;
    imu.x = x / norm_q;
    imu.y = y / norm_q;
    imu.z = z / norm_q;
    end
end



