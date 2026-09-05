clear; clc; close all;
load('medicion_v1.mat');


a = 4.5; 
c = 10.5;
%Dimensiones de la barra en cm

mi_ang_barra = rad2deg( asin(a/c * sin(deg2rad( ang_servo ))) );

figure;

plot(ang_servo, ang_barra, 'x', 'LineWidth', 2);hold on;
plot(ang_servo, mi_ang_barra, 'LineWidth', 2);

grid on;
legend('Medición', 'Teórico');
xlabel('Ángulo del Servo [°]');
xlim([-45, 60]);
ylabel('Ángulo de la barra [°]');
drawnow; 

%Medición con datos de ida y de vuelta
ang_servo_medicion1 = ang_servo;
load('medicion_v2.mat');

%En el gráfico se observa una relación lineal del tipo ang_barra =
%a0 + a1 * ang_servo

p = polyfit(ang_servo, ang_barra, 1); %Ajuste lineal que minimiza el ECM
a1 = p(1);
a0 = p(2);

figure;
plot(ang_servo, ang_barra, 'x', 'LineWidth', 2);hold on;
plot(ang_servo_medicion1, mi_ang_barra, 'LineWidth', 2);
plot(ang_servo_medicion1, a0 + a1 * ang_servo_medicion1, 'k-.', 'LineWidth', 2);

grid on;
legend('Medición', 'Teórico', 'Aproximación lineal', 'Location', 'northwest');
xlabel('Ángulo del Servo [°]');
xlim([-45, 60]);
ylabel('Ángulo de la barra [°]');

fig2 = gcf;

drawnow;
exportgraphics(fig2, 'ang_s_vs_ang_b.pdf', 'ContentType', 'vector');