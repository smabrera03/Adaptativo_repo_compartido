clear; close all; clc;

load('resultados_simulaciones\Ajuste PID.mat');

i  = 1:1:iteraciones;     % iteraciones (costo y ensayos)
i0 = 0:1:iteraciones;     % iteraciones incluyendo el valor inicial (coeficientes)
lw = 1.5;

%% 1) Evolución del costo
figure;
plot(i, Jhist, '-o', 'LineWidth', lw);
grid on;
title('Evolución de la función de costo J');
xlabel('Iteración');
ylabel('J');
xlim([i(1) i(end)]);
xticks(i);

%% 2) Salida del ensayo 1 en cada iteración
colores = lines(iteraciones);

figure;
hold on;
for k = i
    plot(t, buf_y_ensayo1(:, k), 'Color', colores(k, :), 'LineWidth', lw);
end
grid on;
title('Evolución de la respuesta al escalón (ensayo 1) por iteración');
xlabel('Tiempo [s]');
ylabel('Posición [m]');
legend(compose('Iter %d', i), 'Location', 'southeast');
xlim([t(1) t(end)]);

%% 3) Evolución de los coeficientes
nombres  = {'k_p', 'k_i', 'k_d'};
unidades = {' [rad/m]', '', ''};

figure;
for p = 1:3
    subplot(3, 1, p);
    plot(i0, th(p, :), '-o', 'LineWidth', lw);
    grid on;
    ylabel([nombres{p}, unidades{p}]);
    xlim([i0(1) i0(end)]);
    xticks(i0);
end
xlabel('Iteración');
sgtitle('Evolución de los coeficientes del regulador PID');

%%

figure;
plot(t, y_final, 'LineWidth', lw); hold on;
plot(t, ym, 'LineWidth', lw);
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [m]');
title('Comparación de la respuesta buscada y la obtenida');

figure;
plot(t, rad2deg(u_final), 'LineWidth', lw);
grid on;
xlabel('Tiempo [s]');
ylabel('Acción de control [°]');
title('Acción de control obtenida');



