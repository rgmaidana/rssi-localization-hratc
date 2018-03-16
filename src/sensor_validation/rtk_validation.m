%% RTK validation
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

% Trajectory selection
trajectory = 3;

%% Data acquisition
% Get the RTK data from the datasets
size = [4 Inf];
formatSpec = '%f,%f,%f,%f'; % Lat, Lon, H, Time
traj1_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj3_dir), 'r'), formatSpec, size)';

% Extract data according to the selected trajectory
switch trajectory
    case 1
        x = traj1_rtk_data(:,1);
        y = traj1_rtk_data(:,2);
        z = traj1_rtk_data(:,3);
        time = traj1_rtk_data(:,4);
    case 2
        x = traj2_rtk_data(:,1);
        y = traj2_rtk_data(:,2);
        z = traj2_rtk_data(:,3);
        time = traj2_rtk_data(:,4);
    case 3
        x = traj3_rtk_data(:,1);
        y = traj3_rtk_data(:,2);
        z = traj3_rtk_data(:,3);
        time = traj3_rtk_data(:,4);
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

% Predefined trajectory total distance in meters
d_pred = 47.58;

%% Plot!
figure;
hold on
grid on
plot(x(1),y(1),'o','MarkerSize',10,'MarkerFaceColor','b');
plot(x,y,'-','LineWidth',3);
plot(x(1),y(1),'o','MarkerSize',10,'MarkerFaceColor','b');
xlabel('x (m)');
ylabel('y (m)');
legend('Initial Position');
axis([-5 25 -7.5 1.5]);

% Integrate the trajectory displacement to find out the total distance
% travelled
d_gt = 0;
eucl = @(x,y,Sx,Sy)( sqrt((x-Sx)^2 + (y-Sy)^2) );
for i = 2:length(x)
    d_gt = d_gt + eucl(x(i),y(i),x(i-1),y(i-1));
end

% Calculate accuracy in percent
accuracy = (1-(sqrt((d_gt-d_pred)^2))/d_pred)*100;

% Display results
str = sprintf('Total Distance: %f', d_gt);
disp(str);
str = sprintf('Accuracy: %f', accuracy);
disp(str);