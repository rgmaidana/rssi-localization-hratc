%% RSSI measurement validation
clc; close all; clear all;

% Experiment directories
% Change this to the relevant dataset root directory
root_dir = '../../../../xbee_experiments_estacionamento_pucrs_25_11';
traj1_dir = sprintf('%s/ideal_setup_trajectory/1', root_dir);
traj2_dir = sprintf('%s/ideal_setup_trajectory/2', root_dir);
traj3_dir = sprintf('%s/ideal_setup_trajectory/3', root_dir);
rtk1_dir = sprintf('%s/parsed/RTK', traj1_dir);
rtk2_dir = sprintf('%s/parsed/RTK', traj2_dir);
rtk3_dir = sprintf('%s/parsed/RTK', traj3_dir);
xbee1_dir = sprintf('%s/parsed/xbee', traj1_dir);
xbee2_dir = sprintf('%s/parsed/xbee', traj2_dir);
xbee3_dir = sprintf('%s/parsed/xbee', traj3_dir);

% Trajectory selection
trajectory = 1;

%% Data acquisition
% Get the RTK and RSSI data from the datasets
size = [4 Inf];
formatSpec = '%f,%f,%f,%f'; % Lat, Lon, H, Time
traj1_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_rtk_data = fscanf(fopen(sprintf('%s/parsed/RTK/RTK_log.txt',traj3_dir), 'r'), formatSpec, size)';

% RSSI
size = [4 Inf];
formatSpec = '%f,%f,%f,%f'; % TX1, TX2, TX3, Time
traj1_xbee_data = fscanf(fopen(sprintf('%s/parsed/xbee/xbee_log.txt',traj1_dir), 'r'), formatSpec, size)';
traj2_xbee_data = fscanf(fopen(sprintf('%s/parsed/xbee/xbee_log.txt',traj2_dir), 'r'), formatSpec, size)';
traj3_xbee_data = fscanf(fopen(sprintf('%s/parsed/xbee/xbee_log.txt',traj3_dir), 'r'), formatSpec, size)';

% Extract data according to the selected trajectory
switch trajectory
    case 1
        % RTK
        x = traj1_rtk_data(:,1);
        y = traj1_rtk_data(:,2);
        z = traj1_rtk_data(:,3);
        t = traj1_rtk_data(:,4);
        % RSSI
        tx1_rssi = traj1_xbee_data(:,1);
        tx2_rssi = traj1_xbee_data(:,2);
        tx3_rssi = traj1_xbee_data(:,3);
        
    case 2
        % RTK
        x = traj2_rtk_data(:,1);
        y = traj2_rtk_data(:,2);
        z = traj2_rtk_data(:,3);
        t = traj2_rtk_data(:,4);
        % RSSI
        tx1_rssi = traj2_xbee_data(:,1);
        tx2_rssi = traj2_xbee_data(:,2);
        tx3_rssi = traj2_xbee_data(:,3);
    case 3
        % RTK
        x = traj3_rtk_data(:,1);
        y = traj3_rtk_data(:,2);
        z = traj3_rtk_data(:,3);
        t = traj3_rtk_data(:,4);
        % RSSI
        tx1_rssi = traj3_xbee_data(:,1);
        tx2_rssi = traj3_xbee_data(:,2);
        tx3_rssi = traj3_xbee_data(:,3);
    otherwise
        disp('Invalid trajectory. Please select a trajectory between 1 and 3.');
        return
end

% Optimized beacon positions
% Lat,lon,H
S1_coord = [-30.0619427161 -51.175868012 0];
S2_coord = [-30.061802209 -51.175800 0];
S3_coord = [-30.0619346883 -51.1757012998 0];

% Convert to local ENU
[xS1, yS1, zS1] = geodetic2enu(S1_coord(1),S1_coord(2),S1_coord(3),x(1),y(1),z(1),wgs84Ellipsoid);
[xS2, yS2, zS2] = geodetic2enu(S2_coord(1),S2_coord(2),S2_coord(3),x(1),y(1),z(1),wgs84Ellipsoid);
[xS3, yS3, zS3] = geodetic2enu(S3_coord(1),S3_coord(2),S3_coord(3),x(1),y(1),z(1),wgs84Ellipsoid);
S1 = [xS1, yS1, zS1];
S2 = [xS2, yS2, zS2];
S3 = [xS3, yS3, zS3];

% Transform RTK coordinates from geodetic lat-lon to local ENU
[x, y, z] = geodetic2enu(x,y,z,x(1),y(1),z(1),wgs84Ellipsoid);

% Transform UTC time to local time
t = t - t(1);

% Ideal distance measurements
eucl = @(x,y,Sx,Sy) (sqrt((x-Sx).^2 + (y-Sy).^2));
dS1 = eucl(x,y,S1(1), S1(2));
dS2 = eucl(x,y,S2(1), S2(2));
dS3 = eucl(x,y,S3(1), S3(2));

% Ideal RSSI measurements
h = @(d)(20*log10(4*pi*d*2.4e9/299792458));
tx1_rssi_ideal = h(dS1);
tx2_rssi_ideal = h(dS2);
tx3_rssi_ideal = h(dS3);

% Estimated constant biases
b1 = 9; b2 = 9; b3 = 9;
tx1_rssi = tx1_rssi + b1;
tx2_rssi = tx2_rssi + b2;
tx3_rssi = tx3_rssi + b3;

%% Plot!
% B1 (measured vs ideal)
figure
hold on
grid on
title(sprintf('Bias = %.2f', b1));
xlabel('Time (s)');
ylabel('RSSI (dB)');
plot(t,tx1_rssi_ideal,'r-','Linewidth',2);
plot(t,tx1_rssi,'b-');
legend('Ideal','Measured');
% B2 (measured vs ideal)
figure
hold on
grid on
title(sprintf('Bias = %.2f', b2));
xlabel('Time (s)');
ylabel('RSSI (dB)');
plot(t,tx2_rssi_ideal,'r-','Linewidth',2);
plot(t,tx2_rssi,'b-');
legend('Ideal','Measured');
% B3 (measured vs ideal)
figure
hold on
grid on
title(sprintf('Bias = %.2f', b3));
xlabel('Time (s)');
ylabel('RSSI (dB)');
plot(t,tx3_rssi_ideal,'r-','Linewidth',2);
plot(t,tx3_rssi,'b-');
legend('Ideal','Measured');