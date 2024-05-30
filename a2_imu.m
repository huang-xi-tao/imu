clc;%清理命令行窗口
clear;%清理工作区

% 定义 FusionConvention 枚举
FusionConvention = {'Nwu', 'Enu', 'Ned'};

% 定义 FusionMatrix 结构体
FusionMatrix = struct(...
    'array', zeros(3, 3), ...
    'element', struct('xx', 0.0, 'xy', 0.0, 'xz', 0.0, ...
                      'yx', 0.0, 'yy', 0.0, 'yz', 0.0, ...
                      'zx', 0.0, 'zy', 0.0, 'zz', 0.0) ...
);

% 定义 FusionVector 结构体
FusionVector = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));

% 定义 FusionOffset 结构体
FusionOffset = struct('filterCoefficient', 0.0, 'timeout', 0, 'timer', 0, 'gyroscopeOffset', FusionVector);

% 定义 FusionQuaternion 结构体
FusionQuaternion = struct('array', [0.0, 0.0, 0.0, 0.0], 'element', struct('w', 0.0, 'x', 0.0, 'y', 0.0, 'z', 0.0));

% 定义 FusionAhrsSettings 结构体
FusionAhrsSettings = struct('convention', 0, 'gain', 0.0, 'accelerationRejection', 0.0, 'magneticRejection', 0.0, 'rejectionTimeout', 0);

% 定义 FusionAhrs 结构体
FusionAhrs = struct(...
    'settings', FusionAhrsSettings, ...
    'quaternion', FusionQuaternion, ...
    'accelerometer', FusionVector, ...
    'initialising', false, ...
    'rampedGain', 0.0, ...
    'rampedGainStep', 0.0, ...
    'halfAccelerometerFeedback', FusionVector, ...
    'halfMagnetometerFeedback', FusionVector, ...
    'accelerometerIgnored', false, ...
    'accelerationRejectionTimer', 0, ...
    'accelerationRejectionTimeout', false, ...
    'magnetometerIgnored', false, ...
    'magneticRejectionTimer', 0, ...
    'magneticRejectionTimeout', false ...
);

% 定义 ahrs_fusion_t 结构体
ahrs_fusion_t = struct(...
    'gyroscopeMisalignment', FusionMatrix, ...
    'gyroscopeSensitivity', FusionVector, ...
    'gyroscopeOffset', FusionVector, ...
    'accelerometerMisalignment', FusionMatrix, ...
    'accelerometerSensitivity', FusionVector, ...
    'accelerometerOffset', FusionVector, ...
    'softIronMatrix', FusionMatrix, ...
    'hardIronOffset', FusionVector, ...
    'offset', FusionOffset, ...
    'ahrs', FusionAhrs, ...
    'settings', FusionAhrsSettings, ...
    'rate', uint32(0), ...
    'quaternion', FusionQuaternion ...
);
robot = struct(...
    'ahrs', ahrs_fusion_t, ...
    'accel', [0.0, 0.0, 0.0], ... % 加速度数据
    'gyro', [0.0, 0.0, 0.0], ...  % 陀螺仪数据
    'mag', [0.0, 0.0, 0.0], ...   % 磁力计数据
    'quaternion', [0.0, 0.0, 0.0, 0.0], ... % 四元数数据
    'temperature', 0.0, ... % 温度数据
    'q_record', zeros(1, 4) ... % 记录四元数数据
);

data = readtable('/home/hxt/Desktop/algorithm/imu_calc/imu_data.csv');
% 
% 
robot.ahrs = ahrs_reset(robot.ahrs,1000);

for i = 1:size(data.Index)
robot.accel = [data.accl0(i,:), data.accl1(i,:),data.accl2(i,:)];
robot.gyro = [data.rot0(i,:), data.rot1(i,:), data.rot2(i,:)];
dt = 0.001;
robot.ahrs = ahrs_update(robot.ahrs, robot.accel, robot.gyro, robot.mag, dt);
robot.q_record(i,:) = [robot.ahrs.quaternion.element.w robot.ahrs.quaternion.element.x robot.ahrs.quaternion.element.y robot.ahrs.quaternion.element.z];
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

plot(data.Time, robot.q_record(:,2), 'DisplayName','x','Color','r', 'LineWidth', 1.5);
plot(data.Time, robot.q_record(:,3), 'DisplayName','y','Color','g', 'LineWidth', 1.5);
plot(data.Time, robot.q_record(:,4), 'DisplayName','z','Color',[0.0,1.0,1.0], 'LineWidth', 1.5);
plot(data.Time, robot.q_record(:,1),'DisplayName','w','Color','b', 'LineWidth', 1.5);
title('calculate quaternion data');
xlabel('Time');
ylabel('quaternion');
legend;
grid on;




function ahrs = ahrs_update(ahrs, accel, gyro, mag, dt)
    % 创建 FusionVector 结构体并赋值
%     gyroscope = struct('axis', [FusionRadiansToDegrees(gray(1)), FusionRadiansToDegrees(gray(2)), FusionRadiansToDegrees(gray(3))]); % degrees/s
%     accelerometer = struct('axis', [accel(1), accel(2), accel(3)]); % g
%     magnetometer = struct('axis', [0, 0, 0]); % FUSION_VECTOR_ZERO
    gyroscope = struct('axis', struct('x', FusionRadiansToDegrees(gyro(1)), 'y', FusionRadiansToDegrees(gyro(2)), 'z', FusionRadiansToDegrees(gyro(3)))); % degrees/s
    accelerometer = struct('axis', struct('x', accel(1), 'y', accel(2), 'z', accel(3))); % g
    magnetometer = struct('axis', struct('x', mag(1), 'y', mag(2), 'z', mag(3))); % magnetometer data

    % 应用校准
    gyroscope = FusionCalibrationInertial(gyroscope, ahrs.gyroscopeMisalignment, ahrs.gyroscopeSensitivity, ahrs.gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, ahrs.accelerometerMisalignment, ahrs.accelerometerSensitivity, ahrs.accelerometerOffset);
    magnetometer = FusionCalibrationMagnetic(magnetometer, ahrs.softIronMatrix, ahrs.hardIronOffset);

    % 更新陀螺仪偏移校正算法
    [gyroscope, ahrs.offset] = FusionOffsetUpdate(ahrs.offset, gyroscope);

    % 更新陀螺仪姿态估计算法
    ahrs.ahrs = FusionAhrsUpdate(ahrs.ahrs, gyroscope, accelerometer, magnetometer, dt);

    % 算法输出
    ahrs.quaternion = FusionAhrsGetQuaternion(ahrs.ahrs);
end

function quaternion = FusionAhrsGetQuaternion(ahrs)
    % 返回四元数
    quaternion = ahrs.quaternion;
end


function ahrs = FusionAhrsUpdate(ahrs, gyroscope, accelerometer, magnetometer, deltaTime)
    % 定义四元数元素
%     Qw = ahrs.quaternion.element.w;
%     Qx = ahrs.quaternion.element.x;
%     Qy = ahrs.quaternion.element.y;
%     Qz = ahrs.quaternion.element.z;
    
    % 存储加速度计数据
    ahrs.accelerometer = accelerometer;
    
    % 初始化时逐渐减小增益
    if ahrs.initialising
        ahrs.rampedGain = ahrs.rampedGain - ahrs.rampedGainStep * deltaTime;
        if ahrs.rampedGain < ahrs.settings.gain
            ahrs.rampedGain = ahrs.settings.gain;
            ahrs.initialising = false;
            ahrs.accelerationRejectionTimeout = false;
        end
    end
    
    % 计算算法指示的重力方向
    halfGravity = HalfGravity(ahrs);
    % 定义 FUSION_VECTOR_ZERO
    FUSION_VECTOR_ZERO = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));

    % 计算加速度计反馈
    halfAccelerometerFeedback = FUSION_VECTOR_ZERO;
    ahrs.accelerometerIgnored = true;
    if ~FusionVectorIsZero(accelerometer)
        if ahrs.accelerationRejectionTimer > ahrs.settings.rejectionTimeout
            quaternion = ahrs.quaternion;
            FusionAhrsReset(ahrs);
            ahrs.quaternion = quaternion;
            ahrs.accelerationRejectionTimer = 0;
            ahrs.accelerationRejectionTimeout = true;
        end
        
        ahrs.halfAccelerometerFeedback = FusionVectorCrossProduct(FusionVectorNormalise(accelerometer), halfGravity);
        
        if ahrs.initialising || (FusionVectorMagnitudeSquared(ahrs.halfAccelerometerFeedback) <= ahrs.settings.accelerationRejection)
            halfAccelerometerFeedback = ahrs.halfAccelerometerFeedback;
            ahrs.accelerometerIgnored = false;
            ahrs.accelerationRejectionTimer = ahrs.accelerationRejectionTimer - (ahrs.accelerationRejectionTimer >= 10) * 10;
        else
            ahrs.accelerationRejectionTimer = ahrs.accelerationRejectionTimer + 1;
        end
    end
    
    % 计算磁力计反馈
    halfMagnetometerFeedback = FUSION_VECTOR_ZERO;
    ahrs.magnetometerIgnored = true;
    if ~FusionVectorIsZero(magnetometer)
        ahrs.magneticRejectionTimeout = false;
        if ahrs.magneticRejectionTimer > ahrs.settings.rejectionTimeout
            FusionAhrsSetHeading(ahrs, FusionCompassCalculateHeading(ahrs.settings.convention, halfGravity, magnetometer));
            ahrs.magneticRejectionTimer = 0;
            ahrs.magneticRejectionTimeout = true;
        end
        
        halfMagnetic = HalfMagnetic(ahrs);
        
        ahrs.halfMagnetometerFeedback = FusionVectorCrossProduct(FusionVectorNormalise(FusionVectorCrossProduct(halfGravity, magnetometer)), halfMagnetic);
        
        if ahrs.initialising || (FusionVectorMagnitudeSquared(ahrs.halfMagnetometerFeedback) <= ahrs.settings.magneticRejection)
            halfMagnetometerFeedback = ahrs.halfMagnetometerFeedback;
            ahrs.magnetometerIgnored = false;
            ahrs.magneticRejectionTimer = ahrs.magneticRejectionTimer - (ahrs.magneticRejectionTimer >= 10) * 10;
        else
            ahrs.magneticRejectionTimer = ahrs.magneticRejectionTimer + 1;
        end
    end
    
    % 将陀螺仪数据转换为弧度每秒并乘以0.5
    halfGyroscope = FusionVectorMultiplyScalar(gyroscope, FusionDegreesToRadians(0.5));
    x = FusionVectorAdd(halfAccelerometerFeedback, halfMagnetometerFeedback);
    y = FusionVectorMultiplyScalar(x, ahrs.rampedGain);
    % 应用反馈到陀螺仪
    adjustedHalfGyroscope = FusionVectorAdd(halfGyroscope, y);
    
    % 计算四元数的变化率的积分
    ahrs.quaternion = FusionQuaternionAdd(ahrs.quaternion, FusionQuaternionMultiplyVector(ahrs.quaternion, FusionVectorMultiplyScalar(adjustedHalfGyroscope, deltaTime)));
    
    % 归一化四元数
    ahrs.quaternion = FusionQuaternionNormalise(ahrs.quaternion);
    return;
end


function result = FusionQuaternionNormalise(quaternion)
    % 提取四元数元素
    Qw = quaternion.element.w;
    Qx = quaternion.element.x;
    Qy = quaternion.element.y;
    Qz = quaternion.element.z;
    
    % 计算四元数模的倒数
    magnitudeReciprocal = 1.0 / sqrt(Qw^2 + Qx^2 + Qy^2 + Qz^2);
    
    % 计算归一化后的四元数
    result.element.w = Qw * magnitudeReciprocal;
    result.element.x = Qx * magnitudeReciprocal;
    result.element.y = Qy * magnitudeReciprocal;
    result.element.z = Qz * magnitudeReciprocal;
end



function result = FusionQuaternionAdd(quaternionA, quaternionB)
    % 计算四元数的加法
    result.element.w = quaternionA.element.w + quaternionB.element.w;
    result.element.x = quaternionA.element.x + quaternionB.element.x;
    result.element.y = quaternionA.element.y + quaternionB.element.y;
    result.element.z = quaternionA.element.z + quaternionB.element.z;
end


function halfMagnetic = HalfMagnetic(ahrs)
    % 提取四元数元素
    Qw = ahrs.quaternion.element.w;
    Qx = ahrs.quaternion.element.x;
    Qy = ahrs.quaternion.element.y;
    Qz = ahrs.quaternion.element.z;
    
    switch ahrs.settings.convention
        case 'Nwu'
            % 计算磁场的一半
            halfMagnetic.axis.x = Qx * Qy + Qw * Qz;
            halfMagnetic.axis.y = Qw * Qw - 0.5 + Qy * Qy;
            halfMagnetic.axis.z = Qy * Qz - Qw * Qx;
        case 'Enu'
            % 计算磁场的一半
            halfMagnetic.axis.x = 0.5 - Qw * Qw - Qx * Qx;
            halfMagnetic.axis.y = Qw * Qz - Qx * Qy;
            halfMagnetic.axis.z = -1.0 * (Qx * Qz + Qw * Qy);
        case 'Ned'
            % 计算磁场的一半
            halfMagnetic.axis.x = -1.0 * (Qx * Qy + Qw * Qz);
            halfMagnetic.axis.y = 0.5 - Qw * Qw - Qy * Qy;
            halfMagnetic.axis.z = Qw * Qx - Qy * Qz;
        otherwise
            error('Invalid convention');
    end
end


function FusionAhrsSetHeading(ahrs, heading)
    % 提取四元数元素
    Qw = ahrs.quaternion.element.w;
    Qx = ahrs.quaternion.element.x;
    Qy = ahrs.quaternion.element.y;
    Qz = ahrs.quaternion.element.z;
    
    % 计算 yaw 角
    yaw = atan2(Qw * Qz + Qx * Qy, 0.5 - Qy^2 - Qz^2);
    
    % 计算半航向角与航向角之差的一半
    halfYawMinusHeading = 0.5 * (yaw - FusionDegreesToRadians(heading));
    
    % 计算旋转四元数
    rotation.element.w = cos(halfYawMinusHeading);
    rotation.element.x = 0.0;
    rotation.element.y = 0.0;
    rotation.element.z = -1.0 * sin(halfYawMinusHeading);
    
    % 将旋转四元数乘以当前姿态四元数
    ahrs.quaternion = FusionQuaternionMultiply(rotation, ahrs.quaternion);
end

function radians = FusionDegreesToRadians(degrees)
    % 将度转换为弧度
    radians = degrees * (pi / 180);
end


function heading = FusionCompassCalculateHeading(convention, accelerometer, magnetometer)
    switch convention
        case 'Nwu'
            % 计算西向量
            west = FusionVectorNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));
            % 计算北向量
            north = FusionVectorNormalise(FusionVectorCrossProduct(west, accelerometer));
            % 计算方位角
            heading = FusionRadiansToDegrees(atan2(west.axis.x, north.axis.x));
        case 'Enu'
            % 计算西向量
            west = FusionVectorNormalise(FusionVectorCrossProduct(accelerometer, magnetometer));
            % 计算北向量
            north = FusionVectorNormalise(FusionVectorCrossProduct(west, accelerometer));
            % 计算东向量
            east = FusionVectorMultiplyScalar(west, -1.0);
            % 计算方位角
            heading = FusionRadiansToDegrees(atan2(north.axis.x, east.axis.x));
        case 'Ned'
            % 计算向上向量
            up = FusionVectorMultiplyScalar(accelerometer, -1.0);
            % 计算西向量
            west = FusionVectorNormalise(FusionVectorCrossProduct(up, magnetometer));
            % 计算北向量
            north = FusionVectorNormalise(FusionVectorCrossProduct(west, up));
            % 计算方位角
            heading = FusionRadiansToDegrees(atan2(west.axis.x, north.axis.x));
        otherwise
            error('Invalid convention');
    end
end

function result = FusionQuaternionMultiply(quaternionA, quaternionB)
    % 提取四元数元素
    Aw = quaternionA.element.w;
    Ax = quaternionA.element.x;
    Ay = quaternionA.element.y;
    Az = quaternionA.element.z;
    
    Bw = quaternionB.element.w;
    Bx = quaternionB.element.x;
    By = quaternionB.element.y;
    Bz = quaternionB.element.z;
    
    % 计算四元数乘积的元素
    result.element.w = Aw * Bw - Ax * Bx - Ay * By - Az * Bz;
    result.element.x = Aw * Bx + Ax * Bw + Ay * Bz - Az * By;
    result.element.y = Aw * By - Ax * Bz + Ay * Bw + Az * Bx;
    result.element.z = Aw * Bz + Ax * By - Ay * Bx + Az * Bw;
end





function [gyroscope, offset] = FusionOffsetUpdate(offset, gyroscope)
    % 定义阈值常量
    THRESHOLD = 3.0;

    % 向量相减
    gyroscope = FusionVectorSubtract(gyroscope, offset.gyroscopeOffset);

    % 如果陀螺仪没有静止，则重置计时器
    if (abs(gyroscope.axis.x) > THRESHOLD) || (abs(gyroscope.axis.y) > THRESHOLD) || (abs(gyroscope.axis.z) > THRESHOLD)
        offset.timer = 0;
        return;
    end

    % 当陀螺仪静止时，计时器递增
    if offset.timer < offset.timeout
        offset.timer = offset.timer + 1;
        return;
    end

    % 如果计时器已经超过了设定的时间，则调整偏移
    offset.gyroscopeOffset = FusionVectorAdd(offset.gyroscopeOffset, FusionVectorMultiplyScalar(gyroscope, offset.filterCoefficient));
    return;
end




function halfGravity = HalfGravity(ahrs)
    Q = ahrs.quaternion.element;
    switch ahrs.settings.convention
        case {'Nwu', 'Enu'}
            % 第三列的转置旋转矩阵乘以0.5
            halfGravity.axis.x = Q.x * Q.z - Q.w * Q.y;
            halfGravity.axis.y = Q.y * Q.z + Q.w * Q.x;
            halfGravity.axis.z = Q.w * Q.w - 0.5 + Q.z * Q.z;
        case 'Ned'
            % 第三列的转置旋转矩阵乘以-0.5
            halfGravity.axis.x = Q.w * Q.y - Q.x * Q.z;
            halfGravity.axis.y = -1.0 * (Q.y * Q.z + Q.w * Q.x);
            halfGravity.axis.z = 0.5 - Q.w * Q.w - Q.z * Q.z;
        otherwise
            % 默认返回零向量，避免编译器警告
            halfGravity.axis.x = 0;
            halfGravity.axis.y = 0;
            halfGravity.axis.z = 0;
    end
end

function isZero = FusionVectorIsZero(vector)
    % 检查向量的所有分量是否为零
    isZero = (vector.axis.x == 0.0) && (vector.axis.y == 0.0) && (vector.axis.z == 0.0);
end


function result = FusionQuaternionMultiplyVector(quaternion, vector)
    % 提取四元数元素
    Qw = quaternion.element.w;
    Qx = quaternion.element.x;
    Qy = quaternion.element.y;
    Qz = quaternion.element.z;
    
    % 提取向量元素
    Vx = vector.axis.x;
    Vy = vector.axis.y;
    Vz = vector.axis.z;
    
    % 计算四元数与向量的乘法
    result.element.w = -Qx * Vx - Qy * Vy - Qz * Vz;
    result.element.x = Qw * Vx + Qy * Vz - Qz * Vy;
    result.element.y = Qw * Vy - Qx * Vz + Qz * Vx;
    result.element.z = Qw * Vz + Qx * Vy - Qy * Vx;
end

function result = FusionVectorCrossProduct(vectorA, vectorB)
    % 初始化结果结构体
    result.axis.x = vectorA.axis.y * vectorB.axis.z - vectorA.axis.z * vectorB.axis.y;
    result.axis.y = vectorA.axis.z * vectorB.axis.x - vectorA.axis.x * vectorB.axis.z;
    result.axis.z = vectorA.axis.x * vectorB.axis.y - vectorA.axis.y * vectorB.axis.x;
end



function magnitudeSquared = FusionVectorMagnitudeSquared(vector)
    % 计算向量的模的平方
    magnitudeSquared = FusionVectorSum(FusionVectorHadamardProduct(vector, vector));
end


function vector = FusionVectorNormalise(vector)
    % 计算向量的模的平方
    magnitudeSquared = FusionVectorMagnitudeSquared(vector);
    
    magnitudeReciprocal = 1.0 / sqrt(magnitudeSquared);

    % 对向量进行归一化
    vector = FusionVectorMultiplyScalar(vector, magnitudeReciprocal);
end


function result = FusionCalibrationInertial(uncalibrated, misalignment, sensitivity, offset)
    % 向量相减
    subtracted = FusionVectorSubtract(uncalibrated, offset);
    
    % 向量的哈达玛积
    hadamardProduct = FusionVectorHadamardProduct(subtracted, sensitivity);
    
    % 矩阵和向量的乘法
    result = FusionMatrixMultiplyVector(misalignment, hadamardProduct);
end

function result = FusionCalibrationMagnetic(uncalibrated, softIronMatrix, hardIronOffset)
    % 矩阵和向量的乘法
    multipliedVector = FusionMatrixMultiplyVector(softIronMatrix, uncalibrated);
    
    % 向量相减
    result = FusionVectorSubtract(multipliedVector, hardIronOffset);
end

function degrees = FusionRadiansToDegrees(radians)
    % 将弧度转换为度
    degrees = radians * (180.0 / pi);
end


function result = FusionVectorMultiplyScalar(vector, scalar)
    % 初始化结果结构体
    result.axis.x = vector.axis.x * scalar;
    result.axis.y = vector.axis.y * scalar;
    result.axis.z = vector.axis.z * scalar;
end

function sum = FusionVectorSum(vector)
    % 计算向量分量之和
    sum = vector.axis.x + vector.axis.y + vector.axis.z;
end

function result = FusionVectorAdd(vectorA, vectorB)
    % 初始化结果结构体
    result.axis.x = vectorA.axis.x + vectorB.axis.x;
    result.axis.y = vectorA.axis.y + vectorB.axis.y;
    result.axis.z = vectorA.axis.z + vectorB.axis.z;
end


function result = FusionVectorSubtract(vectorA, vectorB)
    % 初始化结果结构体
    result.axis.x = vectorA.axis.x - vectorB.axis.x;
    result.axis.y = vectorA.axis.y - vectorB.axis.y;
    result.axis.z = vectorA.axis.z - vectorB.axis.z;
end

function result = FusionVectorHadamardProduct(vectorA, vectorB)
    % 初始化结果结构体
    result.axis.x = vectorA.axis.x * vectorB.axis.x;
    result.axis.y = vectorA.axis.y * vectorB.axis.y;
    result.axis.z = vectorA.axis.z * vectorB.axis.z;
end

function result = FusionMatrixMultiplyVector(matrix, vector)
    % 使用 matrix.element 字段的矩阵元素进行乘法运算
    R = matrix.element;
    
    % 初始化结果结构体
    result.axis.x = R.xx * vector.axis.x + R.xy * vector.axis.y + R.xz * vector.axis.z;
    result.axis.y = R.yx * vector.axis.x + R.yy * vector.axis.y + R.yz * vector.axis.z;
    result.axis.z = R.zx * vector.axis.x + R.zy * vector.axis.y + R.zz * vector.axis.z;
end

function fusion = ahrs_reset(fusion, rate)
    % 初始化各种常量
    FUSION_IDENTITY_MATRIX = eye(3);
    FUSION_VECTOR_ONES = struct('array', [1.0, 1.0, 1.0], 'axis', struct('x', 1.0, 'y', 1.0, 'z', 1.0));
    FUSION_VECTOR_ZERO = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));

    % 重置陀螺仪相关参数
    fusion.gyroscopeMisalignment.array = FUSION_IDENTITY_MATRIX;
    fusion.gyroscopeMisalignment.element = struct('xx', 1.0, 'xy', 0.0, 'xz', 0.0, ...
                                                  'yx', 0.0, 'yy', 1.0, 'yz', 0.0, ...
                                                  'zx', 0.0, 'zy', 0.0, 'zz', 1.0);
    fusion.gyroscopeSensitivity = FUSION_VECTOR_ONES;
    fusion.gyroscopeOffset = FUSION_VECTOR_ZERO;

    % 重置加速度计相关参数
    fusion.accelerometerMisalignment.array = FUSION_IDENTITY_MATRIX;
    fusion.accelerometerMisalignment.element = struct('xx', 1.0, 'xy', 0.0, 'xz', 0.0, ...
                                                      'yx', 0.0, 'yy', 1.0, 'yz', 0.0, ...
                                                      'zx', 0.0, 'zy', 0.0, 'zz', 1.0);
    fusion.accelerometerSensitivity = FUSION_VECTOR_ONES;
    fusion.accelerometerOffset = FUSION_VECTOR_ZERO;

    % 重置磁力计相关参数
    fusion.softIronMatrix.array = FUSION_IDENTITY_MATRIX;
    fusion.softIronMatrix.element = struct('xx', 1.0, 'xy', 0.0, 'xz', 0.0, ...
                                           'yx', 0.0, 'yy', 1.0, 'yz', 0.0, ...
                                           'zx', 0.0, 'zy', 0.0, 'zz', 1.0);
    fusion.hardIronOffset = FUSION_VECTOR_ZERO;

    % 设置采样率
    fusion.rate = rate;

    % 初始化偏移和姿态估计算法
    fusion.offset = FusionOffsetInitialise(fusion.offset, fusion.rate);
    fusion.ahrs = FusionAhrsInitialise(fusion.ahrs);

    % 设置AHRS算法参数
    fusion.settings.convention = 'Nwu';
    fusion.settings.gain = 0.5;
    fusion.settings.accelerationRejection = 10.0;
    fusion.settings.magneticRejection = 20.0;
    fusion.settings.rejectionTimeout = 5 * fusion.rate;

    fusion.ahrs = FusionAhrsSetSettings(fusion.ahrs, fusion.settings);

    % 返回更新后的结构体
end

function offset = FusionOffsetInitialise(offset, sampleRate)
    % 初始化 FusionOffset 结构体
    CUTOFF_FREQUENCY = 0.02;  % 示例值
    TIMEOUT = 5;  % 示例值
    offset.filterCoefficient = 2.0 * pi * CUTOFF_FREQUENCY * (1.0 / sampleRate);
    offset.timeout = TIMEOUT * sampleRate;
    offset.timer = 0;
    offset.gyroscopeOffset = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));
    end

    function ahrs = FusionAhrsInitialise(ahrs)
    % 定义 FusionAhrsSettings 结构体
    settings = struct(...
        'convention', 'Nwu', ...
        'gain', 0.5, ...
        'accelerationRejection', 90.0, ...
        'magneticRejection', 90.0, ...
        'rejectionTimeout', 0 ...
    );
    
    % 设置 AHRS 参数
    ahrs = FusionAhrsSetSettings(ahrs, settings);
    
    % 重置 AHRS
    ahrs = FusionAhrsReset(ahrs);
end

function ahrs = FusionAhrsReset(ahrs)
INITIAL_GAIN = 10.0;
    % 重置 FusionAhrs 结构体
    ahrs.quaternion = struct('array', [1.0, 0.0, 0.0, 0.0], 'element', struct('w', 1.0, 'x', 0.0, 'y', 0.0, 'z', 0.0));
    ahrs.accelerometer = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));
    ahrs.initialising = true;
    ahrs.rampedGain = INITIAL_GAIN;  % 示例值
    ahrs.halfAccelerometerFeedback = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));
    ahrs.halfMagnetometerFeedback = struct('array', [0.0, 0.0, 0.0], 'axis', struct('x', 0.0, 'y', 0.0, 'z', 0.0));
    ahrs.accelerometerIgnored = false;
    ahrs.accelerationRejectionTimer = 0;
    ahrs.accelerationRejectionTimeout = false;
    ahrs.magnetometerIgnored = false;
    ahrs.magneticRejectionTimer = 0;
    ahrs.magneticRejectionTimeout = false;
end

function ahrs = FusionAhrsSetSettings(ahrs, settings)
INITIAL_GAIN = 10;
INITIALISATION_PERIOD = 3.0;
    % 设置 FusionAhrs 结构体的参数
    ahrs.settings = settings;
    if settings.accelerationRejection == 0.0 || settings.rejectionTimeout == 0
        ahrs.settings.accelerationRejection = realmax('single');
    else
        ahrs.settings.accelerationRejection = (0.5 * sind(settings.accelerationRejection))^2;
    end
    if settings.magneticRejection == 0.0 || settings.rejectionTimeout == 0
        ahrs.settings.magneticRejection = realmax('single');
    else
        ahrs.settings.magneticRejection = (0.5 * sind(settings.magneticRejection))^2;
    end
    ahrs.settings.rejectionTimeout = settings.rejectionTimeout;
    if ~ahrs.initialising
        ahrs.rampedGain = ahrs.settings.gain;
    end
    ahrs.rampedGainStep = (INITIAL_GAIN - ahrs.settings.gain) / INITIALISATION_PERIOD;  % 示例值
end
