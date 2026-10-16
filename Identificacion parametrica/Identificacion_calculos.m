clc; clear; close all;

load('datos_45_grad.mat');
figure;
plot(t, u); hold on;
plot(t, y);

grid on;
xlabel('t');
legend('u', 'y');

Ts = t(2) - t(1);

data = iddata(y, u, Ts);

%La ecuación en diferencias es y(n) = -a1 * y(n-1) - a2 * y(n-2) + b * u(n-1)
sys_d = arx(data, [2 1 0]); %estos coeficientes salen de ver la ecuación en diferencias.
%2 coeficientes para el polinomio del denominador, uno para el del
%numerador y un retardo de 2 muestras en la entrada.

polos_d = pole(sys_d);

sys_d; %Para ver MSE y otros datos descomentar

figure;
compare(data, sys_d)

polos_c = log(polos_d) / Ts;

k = dcgain(sys_d) * polos_c(1) * polos_c(2);

sys_c = zpk([], polos_c, k);