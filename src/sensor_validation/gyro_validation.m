%% Gyroscope validation
clear all; close all; clc;

% Experiment directories
% Change this to the relevant dataset root directory (in this case, the
% stationary dataset)
root_dir = '../../../xbee_experiments_estacionamento_pucrs_15_11';
exp_dir = sprintf('%s/imu_calibration/', root_dir);

% Experimental run selection and experiment parameters
run = 3;
imu_freq = 20;
T = 1/imu_freq;         % Sampling time
start_angle_cc = 0;     % Angular position before counter-clockwise rotation
end_angle_cc = 2*pi;    % Final angular position after counter-clockwise rotation
start_angle_c = 0;      % Angular position before clockwise rotation
end_angle_c = -2*pi;    % Final angular position after clockwise rotation

% Get IMU data from the stationary datasets
size = [11 Inf];
% orientation[x;y;z;w], angular_velocity[x;y;z], linear_acceleration[x;y;z], time
formatSpec = '%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f';
ang1_data = fscanf(fopen(sprintf('%s/angular1.txt',exp_dir), 'r'), formatSpec, size)';
ang2_data = fscanf(fopen(sprintf('%s/angular2.txt',exp_dir), 'r'), formatSpec, size)';
ang3_data = fscanf(fopen(sprintf('%s/angular3.txt',exp_dir), 'r'), formatSpec, size)';

% Extract data according to the selected experimental run
switch run
    case 1
        start_time_cc = 2;      % Time counter-clockwise rotation started
        end_time_cc = 19;       % Time counter-clockwise rotation ended
        start_time_c = 19.3;    % Time clockwise rotation started
        end_time_c = 35;        % Time clockwise rotation ended
        orient_quat = [ang1_data(:,4) ang1_data(:,3) ang1_data(:,2) ang1_data(:,1)];
        ang_vel_x = ang1_data(:,5);
        ang_vel_y = ang1_data(:,6);
        ang_vel_z = ang1_data(:,7);
        time = ang1_data(:,11);
    case 2
        start_time_cc = 5.1;
        end_time_cc = 21.2;
        start_time_c = 21.6;
        end_time_c = 37.5;
        orient_quat = [ang1_data(:,4) ang1_data(:,3) ang1_data(:,2) ang1_data(:,1)];
        ang_vel_x = ang2_data(:,5);
        ang_vel_y = ang2_data(:,6);
        ang_vel_z = ang2_data(:,7);
        time = ang2_data(:,11);
    case 3
        start_time_cc = 3.2;
        end_time_cc = 19.5;
        start_time_c = 19.9;
        end_time_c = 35.9;
        orient_quat = [ang1_data(:,4) ang1_data(:,3) ang1_data(:,2) ang1_data(:,1)];
        ang_vel_x = ang3_data(:,5);
        ang_vel_y = ang3_data(:,6);
        ang_vel_z = ang3_data(:,7);
        time = ang3_data(:,11);
    otherwise
        disp('Invalid experimental run. Please select between 1 and 3.');
        return
end

% Transform to local time
time = time - time(1);

% Transform quaternion to euler angles
orient_eul = quat2eul(orient_quat);

% Ideal measurements
% Angular velocities
vx = zeros([1,length(time)]);
vy = zeros([1,length(time)]);
vtheta = zeros([1,length(time)]);

% Counter-clockwise rotation
dtime = end_time_cc-start_time_cc;      % Time elapsed
dtheta = end_angle_cc-start_angle_cc;   % Angular displacement
ang_vel_ideal = dtheta/dtime;   % Angular velocity (assuming v0 = 0 and instant acceleration)
vtheta(start_time_cc/T+1:end_time_cc/T) = ang_vel_ideal;

% Clockwise rotation
dtime = end_time_c-start_time_c;        % Time elapsed
dtheta = end_angle_c-start_angle_c;     % Angular displacement
ang_vel_ideal = dtheta/dtime;   % Angular velocity (assuming v0 = 0 and instant acceleration)
vtheta(start_time_c/T:end_time_c/T) = ang_vel_ideal;

%% Plot!
% Angular velocity in z axis
figure
hold on
grid on
plot(time,vtheta,'r-','Linewidth',2);
plot(time,ang_vel_z);
title(sprintf('Gyro_%d', trajectory));
xlabel('Time (s)');
ylabel('\omega_z (rad/s)');
legend('Ideal', 'Measured');