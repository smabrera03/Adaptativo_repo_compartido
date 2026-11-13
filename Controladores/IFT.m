%{

Iterative Feedback Tuning
Objetivo: Ajustar un PID, sin conocer la planta

Partimos de un controlador que sepamos que hace estable a la planta,
digamos kp = 1, ki = 0, kd = 0

Iteración:

1) Ensayo 1: aplicamos una referencia r1 y medimos y1, u1
2) Ensayo 2: aplicamos una referencia r2 = r1 - y1, medimos y2, u2
3) Calcular las derivadas dy/dtheta, du/dhteta:

dy/dhteta = 1/R * dR/dtheta * y2
du/dtheta = 1/R * dR/dtheta * u2

1/R * dR/dtheta vendría a ser una especie de filtro, con una entrada y
tantas salidas como parámetros tenga el controlador

4) A partir de estas derivadas calcular el gradiente dJ/dtheta

5) Actualizar theta_k = theta_k-1 - gamma * dJ/dtheta


%}


%% DEFINICIÓN DE PARÁMETROS INICIALES
clc; clear; close all;

kp = 1.0;
ki = 0;
kd = 0;

gamma = 0.5;
lambda = 2.0;
iteraciones = 10;

Tfin = 5; %tiempo de finalización
Ts = 0.05; %¿Dónde se define esto en el código 'adquirir_datos.m'?

N = round(Tfin/Ts);
t = (0:N-1)' * Ts;

ref = 0.1 * (t >= 0.5);

[y0, u0, datos] = simular_respuesta(t, ref, kp, ki, kd);   % ensayo 1

%% Definición del comportamiento objetivo a lazo cerrado

s = tf('s');
tau = 0.5;
T_obj = 1/(tau*s + 1)^2;
ym = lsim(T_obj, ref, t);

lw = 2;
figure;
plot(t, ref, 'LineWidth', lw); hold on;
plot(t, y0, 'LineWidth', lw); hold on;
plot(t, ym, 'LineWidth', lw);
grid on;
xlabel('Tiempo [s]');
ylabel('Posición [m]');
legend('Referencia', 'y con k_p = 1', 'y_{m}', 'Location', 'southeast');

%% ITERACIONES IFT

% Parámetros a ajustar (1 = se ajusta, 0 = queda fijo)
% Para ajustar solo un PI, dejar kd = 0 y ajustar = [1; 1; 0]
ajustar = [1; 1; 1];

% Buffers de señales: cada columna es una iteración
buf_y_ensayo1 = zeros(N, iteraciones);
buf_y_ensayo2 = zeros(N, iteraciones);
buf_u_ensayo1 = zeros(N, iteraciones);
buf_u_ensayo2 = zeros(N, iteraciones);

% Historial de costo y parámetros
Jhist = zeros(1, iteraciones);
th = zeros(3, iteraciones + 1);
th(:, 1) = [kp; ki; kd];

for k = 1:iteraciones

    % --- Ensayo 1: referencia r1 = ref
    [y1, u1] = simular_respuesta(t, ref, kp, ki, kd);

    % --- Ensayo 2: referencia r2 = ref - y1
    [y2, u2] = simular_respuesta(t, ref - y1, kp, ki, kd);

    % Guardar señales
    buf_y_ensayo1(:, k) = y1;
    buf_y_ensayo2(:, k) = y2;
    buf_u_ensayo1(:, k) = u1;
    buf_u_ensayo2(:, k) = u2;

    % --- Costo J (filmina 13)
    e_ym = y1 - ym;
    Jhist(k) = (sum(e_ym.^2) + lambda * sum(u1.^2)) / (2*N);

    % --- Filtro 1/R * dR/dtheta, con A(q) común a ki y kd
    a = [1 + ki + kd, -(1 + 2*kd), kd];   % A(q)
    
    %if any(abs(roots(a)) >= 1)
     %   warning('Iter %d: A(q) tiene raíces fuera del círculo unitario. Filtro inestable.', k);
    %end

    % Sensibilidades de la salida: filtrar y2
    dy_dkp = y2 / kp;
    dy_dki = filter(1, a, y2);
    dy_dkd = filter([1 -2 1],  a, y2);

    % Sensibilidades de la actuación: filtrar u2
    du_dkp = u2 / kp;
    du_dki = filter(1,         a, u2);
    du_dkd = filter([1 -2 1],  a, u2);

    dydth = [dy_dkp, dy_dki, dy_dkd];   % N x 3
    dudth = [du_dkp, du_dki, du_dkd];   % N x 3

    % --- Gradiente dJ/dtheta (filmina 14)
    dJdth = (dydth' * e_ym + lambda * dudth' * u1) / N;   % 3 x 1

    % --- Actualización de parámetros
    theta = [kp; ki; kd] - gamma * (ajustar .* dJdth);
    kp = theta(1);  ki = theta(2);  kd = theta(3);

    th(:, k+1) = theta;

    fprintf('Iter %2d: J = %.6f | kp = %.4f, ki = %.4f, kd = %.4f\n', ...
            k, Jhist(k), kp, ki, kd);
end

fprintf('Parámetros finales: kp = %.4f, ki = %.4f, kd = %.4f\n', kp, ki, kd);


%% Simulo la respuesta del sistema con los últimos coeficientes obtenidos

[y_final, u_final, datos_final] = simular_respuesta(t, ref, kp, ki, kd);

%%

nombre = fullfile('resultados_simulaciones', '---'); % Nombre del archivo donde se guardarán los resultados
save(nombre);

