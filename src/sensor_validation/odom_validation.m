%% Odometry validation
clear all; close all; clc;

% Experiment directories
% Change this to the relevant dataset root directory
root_dir = '../../../xbee_experiments_estacionamento_pucrs_25_11';
traj1_dir = sprintf('%s/ideal_setup_trajectory/1', root_dir);
traj2_dir = sprintf('%s/ideal_setup_trajectory/2', root_dir);
traj3_dir = sprintf('%s/ideal_setup_trajectory/3', root_dir);
rtk1_dir = sprintf('%s/parsed/RTK', traj1_dir);
rtk2_dir = sprintf('%s/parsed/RTK', traj2_dir);
rtk3_dir = sprintf('%s/parsed/RTK', traj3_dir);
odom1_dir = sprintf('%s/parsed/odom', traj1_dir);
odom2_dir = sprintf('%s/parsed/odom', traj2_dir);
odom3_dir = sprintf('%s/parsed/odom', traj3_dir);

% Trajectory selection
trajectory = 1;

%% Data acquisition
% Get the RTK and odometry data from the datasets
size = [4 Inf];
formatSpec = '%f,%f,%f,%f'; % Lat, Lon, H, Time
traj1_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj3_dir), 'r'), formatSpec, size)';

% Odom
size = [8 Inf];
% position[x;y;z], orientation[x;y;z;w], time
formatSpec = '%f,%f,%f,%f,%f,%f,%f,%f';
traj1_odom_data = fscanf(fopen(sprintf('%s/parsed/odom/odom_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_odom_data = fscanf(fopen(sprintf('%s/parsed/odom/odom_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_odom_data = fscanf(fopen(sprintf('%s/parsed/odom/odom_log.txt',traj3_dir), 'r'), formatSpec, size)';

% Extract data according to the selected trajectory
switch trajectory
    case 1
        offset = 9;
        % RTK
        x = traj1_rtk_data(:,1);
        y = traj1_rtk_data(:,2);
        z = traj1_rtk_data(:,3);
        time = traj1_rtk_data(:,4);
        % Odometry
        x_odom = traj1_odom_data(:,2);      % RosAria considers forward movement as positive the x axis, so x = y
        y_odom = -traj1_odom_data(:,1);     % Likewise, y = x. As forward means south, y = -x
    case 2
        offset = 9;
        % RTK
        x = traj2_rtk_data(:,1);
        y = traj2_rtk_data(:,2);
        z = traj2_rtk_data(:,3);
        time = traj2_rtk_data(:,4);
        % Odometry
        x_odom = traj2_odom_data(:,2);      % RosAria considers forward movement as positive the x axis, so x = y
        y_odom = -traj2_odom_data(:,1);     % Likewise, y = x. As forward means south, y = -x
    case 3
        offset = 10;
        % RTK
        x = traj3_rtk_data(:,1);
        y = traj3_rtk_data(:,2);
        z = traj3_rtk_data(:,3);
        % Odometry
        time = traj3_rtk_data(:,4);
        x_odom = traj3_odom_data(:,2);      % RosAria considers forward movement as positive the x axis, so x = y
        y_odom = -traj3_odom_data(:,1);     % Likewise, y = x. As forward means south, y = -x
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

% Odometry velocities
vx_odom = zeros([1,length(time)]);
vy_odom = zeros([1,length(time)]);
ax_odom = zeros([1,length(time)]);
ay_odom = zeros([1,length(time)]);
for k = 1:length(time)-1
    vx_odom(k+1) = (x_odom(k+1) - x_odom(k))/T;
    vy_odom(k+1) = (y_odom(k+1) - y_odom(k))/T;
    ax_odom(k+1) = (vx_odom(k+1) - vx_odom(k))/T;
    ay_odom(k+1) = (vy_odom(k+1) - vy_odom(k))/T;
end

% Ideal measurements
% Find velocities from position variation over time
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

% The Ground-Truth seems to be delayed relative to the odometry.
% Add an offset to delay the odometry so it can be in phase with the
% Ground-Truth.
vx_odom = [zeros([1 offset]) vx_odom];
vy_odom = [zeros([1 offset]) vy_odom];
ax_odom = [zeros([1 offset]) ax_odom];
ay_odom = [zeros([1 offset]) ay_odom];
vx = [vx zeros([1 offset])];
vy = [vy zeros([1 offset])];
ax = [ax zeros([1 offset])];
ay = [ay zeros([1 offset])];
time = [time' time(end)*ones([1 offset])]';

%% Plot!
% X velocity (measured vs ideal)
figure
hold on
grid on
title(sprintf('Velocities in X (trajectory %d)', trajectory));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
plot(time,vx,'r-');
plot(time,vx_odom,'b-');
legend('Ideal','Measured');
% Y velocity (measured vs ideal)
figure
hold on
grid on
title(sprintf('Velocities in Y (trajectory %d)', trajectory));
xlabel('Time (s)');
ylabel('Velocity (m/s)');
plot(time,vy,'r-');
plot(time,vy_odom,'b-');
legend('Ideal','Measured');

% X acceleration (measured vs ideal)
figure
hold on
grid on
title(sprintf('Accelerations in X (trajectory %d)', trajectory));
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
plot(time,ax,'r-');
plot(time,ax_odom,'b-');
legend('Ideal','Measured');
% Y acceleration (measured vs ideal)
figure
hold on
grid on
title(sprintf('Accelerations in Y (trajectory %d)', trajectory));
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
plot(time,ay,'r-');
plot(time,ay_odom,'b-');
legend('Ideal','Measured');