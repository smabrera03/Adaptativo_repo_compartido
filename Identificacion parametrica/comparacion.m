clear;
p =   -4.8680 + 3.0753i;
k = 11.4345;
P = zpk([], [p conj(p)], k);

load('datos_45_grad.mat');

y_est = lsim(P, u, t);

plot(t, y, 'b', 'LineWidth', 1.5);
hold on;
plot(t, y_est, 'r--', 'LineWidth', 1.5);

grid on;
xlabel('Tiempo [s]');
xlim([10, 30]);
ylabel('Salida [°]');

legend('Medición real', 'Modelo');
title('Sistema real y modelado');
%exportgraphics(gcf, 'Comparacion salidas(servo-barra).png', 'Resolution', 300);

error = y - y_est;
rmse = sqrt(mean(error.^2));
rmse_y = sqrt(mean(y.^2));
fit = (1 - rmse/rmse_y) * 100

%Está comparación sí da bien, por lo que el método tiene sentido