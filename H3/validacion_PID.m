%% VALIDACION DEL CONTROLADOR PID
% Ball & Beam - Hito 3
% Resultados experimentales del IFT

clear; close all; clc;

load('resultados_simulaciones\Ajuste PID.mat');

lw = 1.5;

%% 1) PARAMETROS INICIALES Y FINALES

theta0 = th(:, 1);
thetaf = th(:, end);

kp0 = theta0(1);
ki0 = theta0(2);
kd0 = theta0(3);

kpf = thetaf(1);
kif = thetaf(2);
kdf = thetaf(3);

fprintf('\nCONTROLADOR INICIAL\n');
fprintf('kp = %.8f\n', kp0);
fprintf('ki = %.8f\n', ki0);
fprintf('kd = %.8f\n', kd0);

fprintf('\nCONTROLADOR FINAL\n');
fprintf('kp = %.8f\n', kpf);
fprintf('ki = %.8f\n', kif);
fprintf('kd = %.8f\n', kdf);

%% 2) COMPARACION DE RESPUESTAS

figure;

plot(t, ref, 'k--', 'LineWidth', lw);
hold on;

plot(t, y0, 'LineWidth', lw);
plot(t, y_final, 'LineWidth', lw);
plot(t, ym, 'LineWidth', lw);

grid on;

xlabel('Tiempo [s]');
ylabel('Posición [m]');

title('Comparación de respuestas experimentales');

legend('Referencia', ...
       'Controlador inicial', ...
       'Controlador final', ...
       'Modelo objetivo', ...
       'Location', 'best');

%% 3) COMPARACION DE ACCIONES DE CONTROL

figure;

plot(t, rad2deg(u0), 'LineWidth', lw);
hold on;

plot(t, rad2deg(u_final), 'LineWidth', lw);

grid on;

xlabel('Tiempo [s]');
ylabel('Acción de control [°]');

title('Comparación de las acciones de control');

legend('Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

%% 4) COMPARACION DE LA FUNCION DE COSTO

N = length(t);

J0 = (sum((y0-ym).^2) + ...
      lambda*sum(u0.^2))/(2*N);

Jf = (sum((y_final-ym).^2) + ...
      lambda*sum(u_final.^2))/(2*N);

reduccion_J = 100*(J0-Jf)/J0;

fprintf('\nCOMPARACION EXPERIMENTAL\n');

fprintf('Costo inicial: %.8f\n', J0);
fprintf('Costo final:   %.8f\n', Jf);
fprintf('Reducción:     %.2f %%\n', reduccion_J);