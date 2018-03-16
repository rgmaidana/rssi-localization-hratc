%% Accelerometer validation
clear all; close all; clc;

% Dirs
root_dir = '../../../../xbee_experiments_estacionamento_pucrs_25_11';
traj1_dir = sprintf('%s/ideal_setup_trajectory/1', root_dir);
traj2_dir = sprintf('%s/ideal_setup_trajectory/2', root_dir);
traj3_dir = sprintf('%s/ideal_setup_trajectory/3', root_dir);
rtk1_dir = sprintf('%s/parsed/RTK', traj1_dir);
rtk2_dir = sprintf('%s/parsed/RTK', traj2_dir);
rtk3_dir = sprintf('%s/parsed/RTK', traj3_dir);
gps1_dir = sprintf('%s/parsed/gps', traj1_dir);
gps2_dir = sprintf('%s/parsed/gps', traj2_dir);
gps3_dir = sprintf('%s/parsed/gps', traj3_dir);

% Parameters
trajectory = 1;

% Get the RTK and XBee data from the trajectories
% RTK
size = [4 Inf];
formatSpec = '%f,%f,%f,%f'; % Lat, Lon, H, Time
traj1_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj3_dir), 'r'), formatSpec, size)';

% GPS
size = [4 Inf];
% position[x;y;z], time
formatSpec = '%f,%f,%f,%f';
traj1_gps_data = fscanf(fopen(sprintf('%s/parsed/gps/gps_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_gps_data = fscanf(fopen(sprintf('%s/parsed/gps/gps_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_gps_data = fscanf(fopen(sprintf('%s/parsed/gps/gps_log.txt',traj3_dir), 'r'), formatSpec, size)';

% Select data according to selected trajectory
switch trajectory
    case 1
        offset = 9;
        x = traj1_rtk_data(:,1);
        y = traj1_rtk_data(:,2);
        z = traj1_rtk_data(:,3);
        time = traj1_rtk_data(:,4);
        x_gps = traj1_gps_data(:,1);      
        y_gps = traj1_gps_data(:,2);      
        z_gps = traj1_gps_data(:,3);
    case 2
        offset = 9;
        x = traj2_rtk_data(:,1);
        y = traj2_rtk_data(:,2);
        z = traj2_rtk_data(:,3);
        time = traj2_rtk_data(:,4);
        x_gps = traj2_gps_data(:,1);      
        y_gps = traj2_gps_data(:,2);
        z_gps = traj2_gps_data(:,3);
    case 3
        offset = 10;
        x = traj3_rtk_data(:,1);
        y = traj3_rtk_data(:,2);
        z = traj3_rtk_data(:,3);
        time = traj3_rtk_data(:,4);
        x_gps = traj3_gps_data(:,1);      
        y_gps = traj3_gps_data(:,2);
        z_gps = traj3_gps_data(:,3);
    otherwise
        disp('Invalid trajectory. Please select a trajectory between 1 and 3.');
        return
end

% Transform RTK coordinates from geodetic lat-lon to local ENU
[x_gps, y_gps, z_gps] = geodetic2enu(x_gps,y_gps,z_gps,x(1),y(1),z(1),wgs84Ellipsoid);
[x, y, z] = geodetic2enu(x,y,z,x(1),y(1),z(1),wgs84Ellipsoid);

% Transform UTC time to local time
time = time - time(1);

% Find sampling time from frequency of measurements (number_of_measurements/elapsed_time)
freq = length(time)/(time(end)-time(1));
T = 1/freq;

% The Ground-Truth seems to be delayed relative to the odometry.
% Add an offset to delay the odometry so it can be in phase with the
% Ground-Truth.
% x_gps = [x_gps(1)*ones([1 offset]) x_gps']';
% y_gps = [y_gps(1)*ones([1 offset]) y_gps']';
% time = [time' time(end)*ones([1 offset])]';
% x = [x' x(end)*ones([1 offset])]';
% y = [y' y(end)*ones([1 offset])]';

%% Plot!
figure;
hold on
grid on
plot(x,y,'-','LineWidth',3);
plot(x_gps,y_gps,'-','LineWidth',3);
plot(x(1),y(1),'o','MarkerSize',10,'MarkerFaceColor','b');
xlabel('x (m)');
ylabel('y (m)');
legend('Ground-Truth','GPS');
axis([-1 18 -9 2]);