clear; clc; close all;

device = serialport("COM5", 115200)
total_time = 30; % sec
FREQ = 400; % Hz
dt = 1/400; % sec
Nsample = FREQ * total_time; % collect samples for 60 seconds

time_history = zeros(1, Nsample);
acc_history = zeros(3, Nsample);
gyro_history = zeros(3, Nsample);

global quat_history

quat_history = zeros(4, Nsample+1); % Kalman filtered
quat_history(:, 1) = [1 0 0 0]'; % initial condition


for i=1:Nsample
   
   time_history(i) = read(device, 1, "uint32"); % microsec, timestamp
   if i == 1
       first_time = time_history(i);
   end
   disp("time [sec]: " + num2str((time_history(i)-first_time)/1e6, '%.4f'));
   acc_history(:, i) = read(device, 3, "single")' * 9.8; % m/s2, acc.
   gyro_history(:, i) = read(device, 3, "single")' * pi/180; % rad/s, gyro.

   quat_history(:, i+1) = quaternionEKF(acc_history(:, i), gyro_history(:, i), dt, i);
%    set(fg_accx, 'XData', time_history(1:i), 'YData', acc_history(1, 1:i))
%    set(fg_accy, 'XData', time_history(1:i), 'YData', acc_history(2, 1:i))
%    set(fg_accz, 'XData', time_history(1:i), 'YData', acc_history(3, 1:i))
%    
%    set(fg_gyrox, 'XData', time_history(1:i), 'YData', gyro_history(1, 1:i))
%    set(fg_gyroy, 'XData', time_history(1:i), 'YData', gyro_history(2, 1:i))
%    set(fg_gyroz, 'XData', time_history(1:i), 'YData', gyro_history(3, 1:i))

end

figure;

time_history = time_history - time_history(1);

subplot(2, 1, 1)
hold on
fg_accx = plot(time_history/1000, acc_history(1, :), 'r-');
fg_accy = plot(time_history/1000, acc_history(2, :), 'g-');
fg_accz = plot(time_history/1000, acc_history(3, :), 'b-');
xlabel("time [ms]")
ylabel("acc [m/s2]")
grid on
title("acc")

subplot(2, 1, 2)
hold on
fg_gyrox = plot(time_history/1000, gyro_history(1, :), 'r-');
fg_gyroy = plot(time_history/1000, gyro_history(2, :), 'g-');
fg_gyroz = plot(time_history/1000, gyro_history(3, :), 'b-');
xlabel("time [ms]")
ylabel("gyro [rad/s]")
grid on
title("gyro")


% animation
hfg = figure;
hax = axes(hfg);
load satp.mat
hp_original = plot3(satp(1,:), satp(2,:), satp(3,:), 'k--'); % original satp
hold on
hp_rotated = plot3(satp(1,:), satp(2,:), satp(3,:), 'k-'); % rotated satp
hp_xb = plot3([0 5], [0 0], [0 0], 'r--'); % original x vector
hp_yb = plot3([0 0], [0 5], [0 0], 'g--'); % original y vector
hp_zb = plot3([0 0], [0 0], [0 5], 'b--'); % original z vector
hp_xb_rotated = plot3([0 5], [0 0], [0 0], 'r-'); % rotated x vector
hp_yb_rotated = plot3([0 0], [0 5], [0 0], 'g-'); % rotated y vector
hp_zb_rotated = plot3([0 0], [0 0], [0 5], 'b-'); % rotated z vector
legend(hax, 'original', 'rotated', 'x original', 'y original', 'z original', 'x', 'y', 'z')
set(hax, 'XLim', [-15 15])
set(hax, 'YLim', [-10 10])
set(hax, 'ZLim', [-10 10])
daspect(hax, [1 1 1])

xb_axis = [5, 0, 0]';
yb_axis = [0, 5, 0]';
zb_axis = [0, 0, 5]';

rotated_satp_history = zeros(3, 33, Nsample);
rotated_x_body_history = zeros(3, Nsample);
rotated_y_body_history = zeros(3, Nsample);
rotated_z_body_history = zeros(3, Nsample);

for i=1:Nsample
    current_dcm = quat2dcm(quat_history(:, i)'); % current attitude at ith time point
    active_dcm = transpose(current_dcm); % need to transpose dcm matrix to realize the active rotation
    rotated_satp = active_dcm * satp; % obtain rotated coordinates at ith time point
    rotated_x_body = active_dcm * xb_axis;
    rotated_y_body = active_dcm * yb_axis;
    rotated_z_body = active_dcm * zb_axis;
    
    rotated_satp_history(:, :, i) = rotated_satp;
    rotated_x_body_history(:, i) = rotated_x_body;
    rotated_y_body_history(:, i) = rotated_y_body;
    rotated_z_body_history(:, i) = rotated_z_body;
end

for i = 1:5:Nsample
    rotated_satp = rotated_satp_history(:, :, i);
    rotated_x_body = rotated_x_body_history(:, i);
    rotated_y_body = rotated_y_body_history(:, i);
    rotated_z_body = rotated_z_body_history(:, i);

    set(hp_rotated, 'XData', rotated_satp(1,:), 'YData', rotated_satp(2,:), 'ZData', rotated_satp(3,:)) % update satp coordinates
    set(hp_xb_rotated, 'XData', [0 rotated_x_body(1)], 'YData', [0 rotated_x_body(2)], 'ZData', [0 rotated_x_body(3)]) % uptate rotated x vector coordinates
    set(hp_yb_rotated, 'XData', [0 rotated_y_body(1)], 'YData', [0 rotated_y_body(2)], 'ZData', [0 rotated_y_body(3)]) % update rotated y vector coordinates
    set(hp_zb_rotated, 'XData', [0 rotated_z_body(1)], 'YData', [0 rotated_z_body(2)], 'ZData', [0 rotated_z_body(3)]) % updtae rotated z vector coordinates

    curr_time = time_history(i)/1e6; % sec 
    title(hax, "time: " + num2str(curr_time, '%.2f') + " sec") 
    
    pause(0.01);
end


