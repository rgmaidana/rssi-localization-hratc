%% Accelerometer validation
clear all; close all; clc;

% Experiment directories
% Change this to the relevant dataset root directory
root_dir = '../../../../xbee_experiments_estacionamento_pucrs_25_11';
traj1_dir = sprintf('%s/ideal_setup_trajectory/1', root_dir);
traj2_dir = sprintf('%s/ideal_setup_trajectory/2', root_dir);
traj3_dir = sprintf('%s/ideal_setup_trajectory/3', root_dir);
rtk1_dir = sprintf('%s/parsed/RTK', traj1_dir);
rtk2_dir = sprintf('%s/parsed/RTK', traj2_dir);
rtk3_dir = sprintf('%s/parsed/RTK', traj3_dir);
imu1_dir = sprintf('%s/parsed/imu', traj1_dir);
imu2_dir = sprintf('%s/parsed/imu', traj2_dir);
imu3_dir = sprintf('%s/parsed/imu', traj3_dir);

% Trajectory selection
trajectory = 1;

%% Data acquisition
% Get the RTK and IMU data from the datasets
size = [4 Inf];
formatSpec = '%f,%f,%f,%f'; % Lat, Lon, H, Time
traj1_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj3_dir), 'r'), formatSpec, size)';

% IMU
size = [11 Inf];
% orientation[x;y;z;w], angular_velocity[x;y;z], linear_acceleration[x;y;z], time
formatSpec = '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f';
traj1_imu_data = fscanf(fopen(sprintf('%s/parsed/imu/imu_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_imu_data = fscanf(fopen(sprintf('%s/parsed/imu/imu_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_imu_data = fscanf(fopen(sprintf('%s/parsed/imu/imu_log.txt',traj3_dir), 'r'), formatSpec, size)';

% Extract data according to the selected trajectory
switch trajectory
    case 1
        % RTK
        x = traj1_rtk_data(:,1);
        y = traj1_rtk_data(:,2);
        z = traj1_rtk_data(:,3);
        time = traj1_rtk_data(:,4);
        % IMU
        orient_quat = [traj1_imu_data(:,4) traj1_imu_data(:,3) traj1_imu_data(:,2) traj1_imu_data(:,1)];
        ang_vel_x = traj1_imu_data(:,5);
        ang_vel_y = traj1_imu_data(:,6);
        ang_vel_z = traj1_imu_data(:,7);
        accf = traj1_imu_data(:,8);         % Acceleration in the X axis
        accl = traj1_imu_data(:,9);         % Acceleration in the Y axis
    case 2
        % RTK
        st = 5; ed = 635;   % Early stop
        x = traj2_rtk_data(st:ed,1);
        y = traj2_rtk_data(st:ed,2);
        z = traj2_rtk_data(st:ed,3);
        time = traj2_rtk_data(st:ed,4);
        % IMU
        orient_quat = [traj2_imu_data(st:ed,4) traj2_imu_data(st:ed,3) ...
                       traj2_imu_data(st:ed,2) traj2_imu_data(st:ed,1)];
        ang_vel_x = traj2_imu_data(st:ed,5);
        ang_vel_y = traj2_imu_data(st:ed,6);
        ang_vel_z = traj2_imu_data(st:ed,7);
        accf = traj2_imu_data(st:ed,8);         % Acceleration in the X axis
        accl = traj2_imu_data(st:ed,9);         % Acceleration in the Y axis
    case 3
        % RTK
        x = traj3_rtk_data(:,1);
        y = traj3_rtk_data(:,2);
        z = traj3_rtk_data(:,3);
        time = traj3_rtk_data(:,4);
        % IMU
        orient_quat = [traj3_imu_data(:,4) traj3_imu_data(:,3) traj3_imu_data(:,2) traj3_imu_data(:,1)];
        ang_vel_x = traj3_imu_data(:,5);
        ang_vel_y = traj3_imu_data(:,6);
        ang_vel_z = traj3_imu_data(:,7);
        accf = traj3_imu_data(:,8);         % Acceleration in the X axis
        accl = traj3_imu_data(:,9);         % Acceleration in the Y axis
    otherwise
        disp('Invalid trajectory. Please select a trajectory between 1 and 3.');
        return
end

% Transform RTK coordinates from geodetic lat-lon to local ENU
[x, y, z] = geodetic2enu(x,y,z,x(1),y(1),z(1),wgs84Ellipsoid);

% Transform UTC time to local time
time = time - time(1);

% Find sampling time from frequency of measurements (number_of_measurements/elapsed_time)
freq = length(time)/(time(end)-time(1));
T = 1/freq;

% Ideal measurements
% Find velocities from position variation over time and
% accelerations from velocity variation over time
vx = zeros([1,length(time)]);
vy = zeros([1,length(time)]);
ax = zeros([1,length(time)]);
ay = zeros([1,length(time)]);
for k = 1:length(time)-1
    vx(k+1) = (x(k+1) - x(k))/T;
    vy(k+1) = (y(k+1) - y(k))/T;
    ax(k+1) = (vx(k+1) - vx(k))/T;
    ay(k+1) = (vy(k+1) - vy(k))/T;
end

% From validated gyro, find orientation by integrating from angular velocity
t = zeros([1,length(time)]);
t(1) = -pi/2;        % Initial orientation
for k = 1:length(time)-1
    t(k+1) = t(k) + ang_vel_z(k)*T;
end

% With orientation, convert x and y accelerations to frontal and lateral
% accelerations
accf_ideal = zeros([1,length(time)]);
accl_ideal = zeros([1,length(time)]);
for k = 1:length(time)-1
    R = [cos(t(k))  sin(t(k)); -sin(t(k))  cos(t(k))];  % Rotation matrix
    Acc = R * [ax(k+1) ay(k+1)]';   
    accf_ideal(k+1) = Acc(1);
    accl_ideal(k+1) = Acc(2);
end

%% Plot!
% Frontal acceleration (measured vs ideal)
figure
hold on
grid on
title('Frontal acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
plot(time,accf,'b-');
plot(time,accf_ideal,'r-','Linewidth',2);
legend('Measured','Ideal');
% Lateral acceleration (measured vs ideal)
figure
hold on
grid on
title('Lateral acceleration');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
plot(time,accl,'b-');
plot(time,accl_ideal,'r-','Linewidth',2);
legend('Measured','Ideal');