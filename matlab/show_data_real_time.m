clear all; close all; clc;
line_widht = 2;

%% Input
data = load('../data/data_real_backward_gate.lvm');

u = data(:, 2); % Set Point
u_pid = data(:, 4); % Acción de control PID
y_backward = data(:, 6); % Backward
tu = data(:, 1);
tye = data(:, 1);
tyb = data(:, 1);

figure; hold on;
plot(tu, u, '--r', 'LineWidth', line_widht);
plot(tye, u_pid, 'b', 'LineWidth', line_widht);
plot(tyb, y_backward, 'g', 'LineWidth', line_widht);

legend('set point', 'Acción de control PID', 'Backward')
xlabel('t(seg)'), ylabel('volts');
title('acción de control');
axis([0 6 0 4]);
