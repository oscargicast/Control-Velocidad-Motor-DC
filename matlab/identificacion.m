clear all; close all; clc;
line_widht = 1.5;

%% Input
data_type = input('signal(gate/step/rampa/seno): ', 's');
validatestring(data_type, {'gate', 'step', 'rampa', 'seno'})
file = strcat('../data/motor_', data_type, '.lvm');
data = load(file);

%% Toolbox de identificacion mediante comandos Matlab
%% DATA experimental proviene desde el circuito 3 Opams
% OPAMS
u = data(:, 2);
y = data(:, 4);
t_real = data(:, 1);

N = length(u);
disp('# de muestras'), disp(N);

tini = 0;
tfin = t_real(end);
t = linspace(tini, tfin, N);

% teorica
figure; hold on;
plot(y, 'r', 'LineWidth', line_widht);
plot(u, 'b', 'LineWidth', line_widht);

ylabel('Amplitude(volts)'), xlabel('N(samples)')
legend('y_{exp}', 'u_{exp}')

figure; hold on;
plot(t, y, 'r', 'LineWidth', line_widht);

%% Toolbox de identificacion

%% 1st. Porceso de la data-objeto - iddata DAT = iddata(Y,U,Ts)
Ts = 0.03; 
idata = iddata(y, u, Ts);

%% 2nd Estructura parametrica ARX(na, nb, nc, nx)
nn = [
    2 1 1; 
    1 2 2; 
    2 2 1;
    1 1 1;
];
colors = ['k', 'g', 'b', 'm'];
size_n=size(nn);
for i=1:size_n(1)
fprintf('\n\n\t\t\t\t\t ARX(%d, %d, %d)', nn(i,1), nn(i,2), nn(i,3)); 
fprintf('\n==========================================================='); 
    
th = arx(idata, [nn(i,1), nn(i,2), nn(i,3)]);
% B numerador, A denominador
% FPE(funcion de prediccion de error)
% th

%% 3rd discreta - funcion de transerencia D(z)
D = tf(th.b, th.a, Ts);
D
% De = tf(th.c, th.a, Ts);
% cmd: d2c
th

%% 4th Funcion de transferencia G(s)
% Gs = d2c(D, 'zoh');
% Ge = d2c(De, 'zoh');
% disp('funcion de trans'), Gs;

[n, d] = tfdata(D, 'v'); 
Gs = d2c(D, 'zoh');
Gs

yc = lsim(Gs, u, t);

% armax
plot(t, yc, strcat(colors(i), '--') , 'LineWidth', line_widht);
% legend(colors(i))
end

ylabel('Amplitude(volts)'), xlabel('t(sec.)');
legend( ...
'y_{exp}', ...
arrayfun(@num2str, nn(1,:), 'UniformOutput', true), ...
arrayfun(@num2str, nn(2,:), 'UniformOutput', true), ...
arrayfun(@num2str, nn(3,:), 'UniformOutput', true), ...
arrayfun(@num2str, nn(4,:), 'UniformOutput', true));




