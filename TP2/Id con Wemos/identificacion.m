clear; close all; clc;

load('datos_cart_beam.mat');

t = datos.t;
ang_barra = datos.q1;
y = datos.q2;
y_der = datos.q2_p;
u = datos.u;
r = datos.r;

%% GRÁFICOS

figure; 

plot(t, r); hold on;
plot(t, y);
grid on;
legend('Referencia', 'Posición');
xlim([0, t(end)]);
ylabel('r, y');
xlabel('Tiempo [s]');

figure;
subplot(2, 1, 1);

plot(t, rad2deg(u), 'LineWidth', 1.5);
grid on;
ylabel('u [°]');
xlabel('Tiempo [s]');xlim([0, t(end)]);
title('Acción de control');

subplot(2, 1, 2);
plot(t, y_der, 'LineWidth', 1.5);
grid on;
ylabel('v [m/s]');
xlabel('Tiempo [s]');xlim([0, t(end)]);
title('Velocidad');


%% 