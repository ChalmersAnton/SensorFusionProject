%% INIT
clear; close all; clc

% Simple exercise
alpha = 0;
syms vx vy vz
q = [cos(1/2*alpha); sin(1/2*alpha)*[vx; vy; vz]];
Q = Qq(q);
assert(norm(double(Q)-eye(3)) < 0.00001, ...
    'Rotation matrix not as expected!');

% Run startup every time matlab is restarted!
% startup();

% Run filterTemplate before initiating streaming from smartphone
[xhat, meas] = filterTemplate();

%% TASK 2 - CALCULATE MEAN AND COV OF MEASUREMENTS
close all

% Measurement interval

%meas_int = 28000:35000;
meas_int = [];
% Save measurements from gyro angles [rad/s] and remove NaNs
gyr = meas.gyr(:, ~any(isnan(meas.gyr), 1));
if isempty(meas_int)
    meas_int = 1:length(gyr)-1;
end
gyr = gyr(:, meas_int);

% Calculate mean and covariance
mean_gyr = mean(gyr, 2);
R_gyr = cov(gyr');

% Histogram of gyr data
figure
subplot(1,3,1)
histogram(gyr(1,:))
xlabel('X')
subplot(1,3,2)
histogram(gyr(2,:))
xlabel('Y')
title('Gyroscope')
subplot(1,3,3)
histogram(gyr(3,:))
xlabel('Z')

% Plot measurement signals
figure
subplot(1,3,1)
plot(gyr(1,:))
% hold on
% plot(meas_int - meas_int(1), mean_gyr(1)*ones(size(meas_int)), 'r--', ...
%     'LineWidth', 2.5)
xlabel('X')
ylim([-4 4]*1e-3)
subplot(1,3,2)
plot(gyr(2,:))
xlabel('Y') % X-axis
ylim([-4 4]*1e-3)
title('Gyroscope - signals')
subplot(1,3,3)
plot(gyr(3,:))
xlabel('Z')
ylim([-4 4]*1e-3)

% PLOT GOOGLE COMPARED TO OWN ESTIMATE - QUATERNION
x_est = xhat.x; %(:, ~any(isnan(xhat.x), 1));
google_est = meas.orient; %(:, ~any(isnan(meas.orient), 1));
figure
subplot(221)
plot(x_est(1,:))
hold on
plot(google_est(1,:))
xlabel('q_1')
legend('OWN', 'GOOGLE', 'Location', 'best')
subplot(222)
plot(x_est(2,:))
hold on
plot(google_est(2,:))
xlabel('q_2')
legend('OWN', 'GOOGLE', 'Location', 'best')
subplot(223)
plot(x_est(3,:))
hold on
plot(google_est(3,:))
xlabel('q_3')
legend('OWN', 'GOOGLE', 'Location', 'best')
subplot(224)
plot(x_est(4,:))
hold on
plot(google_est(4,:))
xlabel('q_4')
legend('OWN', 'GOOGLE', 'Location', 'best')
sgtitle('Estimated quaternions using gyroscope')


% PLOT GOOGLE COMPARE TO OUR ESTIMATE - EULER
x_euler = q2euler(x_est);
google_euler = q2euler(google_est);
figure
subplot(131)
plot(x_euler(1,:))
hold on
plot(google_euler(1,:))
xlabel('Roll')
legend('OWN', 'GOOGLE', 'Location', 'best')
subplot(132)
plot(x_euler(2,:))
hold on
plot(google_euler(2,:))
xlabel('Pitch')
legend('OWN', 'GOOGLE', 'Location', 'best')
subplot(133)
plot(x_euler(3,:))
hold on
plot(google_euler(3,:))
xlabel('Yaw')
legend('OWN', 'GOOGLE', 'Location', 'best')
sgtitle('Estimated Euler angles using gyroscope')



% Save measurements from acc [m/(s^2)] and remove NaNs
acc = meas.acc(:, ~any(isnan(meas.acc), 1));
if isempty(meas_int)
    meas_int = 1:length(acc)-1;
end
acc = acc(:, meas_int);

% Calculate mean and covariance
mean_acc = mean(acc, 2);
% cov(acc_x')
R_acc = cov(acc');

% Histogram of acc data
figure
subplot(1,3,1)
xlabel('X')
histogram(acc(1,:))
subplot(1,3,2)
xlabel('Y')
histogram(acc(2,:))
title('Accelerometer')
subplot(1,3,3)
xlabel('Z')
histogram(acc(3,:))

% Plot measurement signals
figure
subplot(1,3,1)
plot(acc(1,:))
xlabel('X')
subplot(1,3,2)
plot(acc(2,:))
xlabel('Y')
title('Accelerometer - signals')
subplot(1,3,3)
plot(acc(3,:))
xlabel('Z')


% Save measurements from magnetometer [micro T] and remove NaNs
mag = meas.mag(:, ~any(isnan(meas.mag), 1));
if isempty(meas_int)
    meas_int = 1:length(mag)-1;
end
mag = mag(:, meas_int);

% Calculate mean and covariance
mean_mag = mean(mag, 2);
R_mag = cov(mag');

% Histogram of mag data
figure
subplot(1,3,1)
xlabel('X')
histogram(mag(1,:))
subplot(1,3,2)
xlabel('Y')
histogram(mag(2,:))
title('Magnetometer')
subplot(1,3,3)
xlabel('Z')
histogram(mag(3,:))


% Plot measurement signals
figure
subplot(1,3,1)
plot(mag(1,:))
xlabel('X')
subplot(1,3,2)
plot(mag(2,:))
xlabel('Y')
title('Magnetometer - signals')
subplot(1,3,3)
plot(mag(3,:))
xlabel('Z')

%% READ LOG FILES
% % Used for visualization.
% figure;
% subplot(1, 2, 1);
% ownView = OrientationView('Own filter', gca);  % Used for visualization.
% googleView = [];
% 
log = readtable(...
    'LogFiles/sensorLog_20200523T085738_face_up_then_moved.txt');
% log.Properties.VariableNames = {'Time', 'Sensor', 'X', 'Y', 'Z'};
% log(1:20, :);
% log(1,3:5);
% counter = 1;
% log_array =  table2array(log(:,3:5));
% while 1
%     try
%         l = log_array([1 2 3 4]+4*counter,:)';
%         data = l(:)';
%     catch e
%         fprintf(['End of log file!\n']);
%         break
%     end
%     counter = counter + 1;
%     orientation = data(1, 18:21)';  % Google's orientation estimate.
% 
%     % Visualize result
%     if rem(counter, 10) == 0
%     setOrientation(ownView, x(1:4));
%     title(ownView, 'OWN', 'FontSize', 16);
%     if ~any(isnan(orientation))
%         if isempty(googleView)
%         subplot(1, 2, 2);
%         % Used for visualization.
%         googleView = OrientationView('Google filter', gca);
%         end
%         setOrientation(googleView, orientation);
%         title(googleView, 'GOOGLE', 'FontSize', 16);
%     end
%     end
% end
