clc
clear 
clf
close all 
%% GPS Data
gpsdataTable = readtable('gps.csv');
gpsTime=(table2array(gpsdataTable(:,1)));
gpsTime=gpsTime-gpsTime(1);
lat = table2array(gpsdataTable(:,6));
lon = table2array(gpsdataTable(:,7));

imudataTable = readtable('imu.csv');
imuTime=(table2array(imudataTable(:,1)));
imuTime=imuTime-imuTime(1);
imuQuat = table2array(imudataTable(:,11:14));
imuGyr = table2array(imudataTable(:,16:18));
imuAcc = table2array(imudataTable(:,20:22));
imuMag = table2array(imudataTable(:,28:30));

L = size(imuTime);
x = imuMag(13000:18000,1);
y = imuMag(13000:18000,2);
z = imuMag(13000:18000,3);
% Compute the average for each axis
x_avg = mean(x);
y_avg = mean(y);
z_avg = mean(z);

% Compute the radius for each axis
x_radius = x - x_avg;
y_radius = y - y_avg;
z_radius = z - z_avg;
% Compute the maximum radius
max_radius = max([max(x_radius), max(y_radius), max(z_radius)]);
% Compute the scale factors for each axis
x_scale = max_radius / mean(abs(x_radius));
y_scale = max_radius / mean(abs(y_radius));
z_scale = max_radius / mean(abs(z_radius));
% Compute the rotation matrix
R = diag([x_scale, y_scale, z_scale]);
R(1,3) = -x_avg*x_scale;
R(2,3) = -y_avg*y_scale;
R(3,3) = -z_avg*z_scale;
imuMag=imuMag-[x_avg y_avg z_avg];
imuMagCal=(imuMag)*R';
intYaw = cumtrapz(imuTime,imuGyr(:,3));
yawCal = unwrap( atan2(imuMagCal(:,1),imuMagCal(:,2)));
yawRaw = unwrap( atan2(imuMag(:,1),imuMag(:,2)));



[b, a] = butter(3, 0.1/40, 'low');
lpf = filtfilt(b, a, yawCal);
[b, a] = butter(3, 0.00001/40, 'high');
hpf = filtfilt(b, a, intYaw);
figure('Position', [0 0 800 400]);
plot(imuTime,lpf, 'LineWidth', 2);
hold on;
plot(imuTime,hpf, 'LineWidth', 2);


alpha = 0.6;

yaw_filtered = zeros(L);
yaw_filtered(1) = 0;

for i = 1:L-1
    j = i + 1;
    yaw_filtered(j) = alpha * (yaw_filtered(i) + hpf(j) * 0.05) + (1 - alpha) * lpf(j);
end
plot(imuTime,yaw_filtered, 'LineWidth', 2);

xlabel('time(s)')
ylabel('angle(degree)')
legend({'LPF Calibrated Yaw', 'HPF Gyro Yaw','CF'}, 'FontSize', 14, 'Location', 'southeast');
grid on;
title('LPF for Magnetic Yaw and HPF for Gyro Yaw')
%% 
figure('Position', [0 0 800 400]);
RPY = quat2eul(imuQuat);
plot(imuTime,intYaw,'LineWidth',2)
hold on
plot(imuTime,yawCal,'LineWidth',2)
plot(imuTime,yaw_filtered,'LineWidth',2)
plot(imuTime,unwrap(RPY(:,3)),'LineWidth',2)
grid on
legend('Yaw Integrated from Gyro','Magnetometer','Complementary filter','Yaw angle computed by the IMU', 'FontSize', 14, 'Location', 'southeast');
xlabel('time(s)')
ylabel('angle(degree)')

