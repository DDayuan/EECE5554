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

%% Magnetometer Calibration
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
% Apply the correction to the magnetometer readings
m_corr =( [x y z] -[x_avg y_avg z_avg])*R;
figure('Position', [0 0 800 800]);
plot(x,y,'LineWidth',2)
hold on
grid on
xlabel('Mag_x')
ylabel('Mag_y')
plot(m_corr(:,1),m_corr(:,2),'LineWidth',1)
legend('Before calibration','After calibration')
%% Cal Mag
imuMag=imuMag-[x_avg y_avg z_avg];
imuMagCal=(imuMag)*R';
figure('Position', [100 100 800 800]);
for i = 1:3
    subplot(3, 1, i, 'position', [0.1 (i-1)*0.25+0.05 0.8 0.2])
end
subplot 311 
hold on
plot(imuTime,imuMag(:,1),'LineWidth',2)
plot(imuTime,imuMagCal(:,1),'LineWidth',2)
title('Mag_x')
legend('Before Calibration','After Calibration', 'Location', 'southeast')
grid on
subplot 312 
hold on
title('Mag_y')
plot(imuTime,imuMag(:,2),'LineWidth',2)
plot(imuTime,imuMagCal(:,2),'LineWidth',2)
legend('Before Calibration','After Calibration','Location', 'southeast')
grid on
subplot 313 
hold on
title('Mag_z')
plot(imuTime,imuMag(:,3),'LineWidth',2)
plot(imuTime,imuMagCal(:,3),'LineWidth',2)
legend('Before Calibration','After Calibration', 'Location', 'southeast')
grid on
set(gcf, 'color', 'w')