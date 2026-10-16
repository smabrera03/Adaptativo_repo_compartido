clc; clear; close all;

load('datos_45_grad.mat');

%La ecuación en diferencias es y(n) = -a1 * y(n-1) - a2 * y(n-2) + b * x(n-2)

N = length(y);

Y = y(3:N);

%Matriz X del problema de CM con las versiones atrasadas de la entrada y la
%salida
X = [-y(2:N-1), -y(1:N-2), u(1:N-2)];

theta = X \ Y; %el operador %resuelve cuadrados mínimos.

% Parámetros estimados
a1 = theta(1);
a2 = theta(2);
b  = theta(3);

Ts = 0.02;
num = [b];
den = [1, a1, a2];

sys_d = tf(num, den, Ts);

polos_d = pole(sys_d);

polos_c = log(polos_d) / Ts;

k = dcgain(sys_d) * polos_c(1) * polos_c(2);

sys_c = zpk([], polos_c, k);