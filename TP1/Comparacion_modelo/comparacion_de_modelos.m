clear; clc, close all;

load('datos_para_comparacion_v2.mat');

entrada = [t(:), ang_servo(:)];

salida = [t(:), pos(:)];

x0 = [pos(1), 0];

T_fin = t(end);

figure;

plot(t, ang_servo, 'k', 'LineWidth', 2);
grid on;

xlabel('Tiempo [s]');
ylim([-15, 25]);
ylabel('Ángulo comandado [°]');
drawnow;

exportgraphics(gcf, 'graficos/entrada_comparacion.pdf', 'ContentType', 'image', 'Resolution', 300);

%% GRAFICOS

x_modelo = double(out.x_modelo);
x_linealizacion = double(out.x_linealizacion);

plot(t, x_modelo, 'LineWidth', 2); hold on;
plot(t, pos, 'k', 'LineWidth', 2);

grid on;
legend('Salida del modelo', 'Salida medida');
ylabel('Posicion [cm]');
xlabel('Tiempo [s]');
drawnow;
exportgraphics(gcf, 'graficos/modelo_vs_medicion.pdf', 'ContentType', 'image', 'Resolution', 300);


figure;
plot(t, x_modelo, 'LineWidth', 2); hold on;
plot(t, x_linealizacion, 'k', 'LineWidth', 2);

grid on;
legend('Salida del modelo', 'Salida del modelo linealizado');
ylabel('Posicion [cm]');
xlabel('Posicion [s]');
drawnow;
exportgraphics(gcf, 'graficos/modelo_vs_linealizacion.pdf', 'ContentType', 'image', 'Resolution', 300);