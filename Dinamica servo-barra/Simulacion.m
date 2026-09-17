clear; clc; close all;

% 1. PARAMETROS
m  = 0.033;       % Masa del carrito [kg]
b  = 0.15;        % Rozamiento efectivo [kg/s]
g  = 9.8;         % Gravedad [m/s^2]

a0 = deg2rad(-3.9);   % Offset servo-barra [rad]
a1 = 0.34;            % Pendiente servo-barra

fs = 50;              % Frecuencia de muestreo [Hz]
Ts = 1/fs;            % Periodo de muestreo [s]

%Coeficientes del denominador de la transferencia
q0 = 33.1549;
q1 = 9.736; 
%Numerador de la transferencia
k = 11.4345;

load("C:\Users\Santiago\Desktop\facu\2026c2\Adaptativo_repo_compartido\TP1\Comparacion_modelo\datos_para_comparacion_v2.mat")
% esta es la medición hecha en el tp1
%pos: posición medida
%ang_servo: entrada de la planta
%ang_barra: angulo medido por la IMU

%Grafico la entrada
plot(t, ang_servo, 'LineWidth', 2); grid on;
ylim([-25, 25]);
xlabel('Tiempo [s]');
ylabel('Ángulo comandado[°]');


% Convertir ángulos de grados a radianes
u_med       = deg2rad(ang_servo);
theta_med   = deg2rad(ang_barra);

% Posición medida
x_med = pos;

% La entrada u del modelo es el ángulo comandado al servo.
%
% Como los datos experimentales están muestreados, se utiliza
% interpolación de orden cero (ZOH).

u_eq = -a0/a1; %valor de equilibrio
u = @(tt) interp1(t, u_med , tt, 'previous', 'extrap'); %Esta función interpola el valor de u en el tiempo tt en base a las mediciones

%OJO: el modelo está en SI ([m] y [rad]), pero las mediciones están en [cm]
%y [°]

%Definición del modelo completo
modelo_completo = @(tt,x) [
    x(2);
    g*sin(x(3)) - (b/m)*x(2);
    x(4);
    -q0*x(3) - q1*x(4) + k*(u(tt) - u_eq)
];

%Definición del modelo simplificado
modelo_simplificado = @(tt, x) [
    x(2);
    g*sin(a0 + a1 * u(tt)) - (b/m)*x(2);
];

% Estado:
% x1 = posición
% x2 = velocidad
% x3 = ángulo de la barra
% x4 = velocidad angular de la barra

x0 = [
    x_med(1)/100;        % posición inicial pasada a [m]
    0;               % velocidad inicial
    theta_med(1);    % ángulo inicial de la barra pasado a [rad]
    0                % velocidad angular inicial
];

opts = odeset('MaxStep', Ts/5);

[t_sim_completo, x_sim] = ode45(modelo_completo, t, x0, opts);


x_modelo_completo = x_sim(:,1) * 100;       % Posición modelada [cm]

[t_sim_simplificado, x_sim] = ode45(modelo_simplificado, t, x0(1:2), opts);
%Nota: el vector de estados tiene solo dos elementos en el modelo
%simplificado, por lo que el estado inicial es x0(1:2)

x_modelo_simplificado = x_sim(:, 1) * 100; %Posición modelada


%% Gráficos
figure;

plot(t, x_med, 'LineWidth', 2);
hold on;
plot(t_sim_completo, x_modelo_completo, '--', 'LineWidth', 2);
hold on;
plot(t_sim_simplificado, x_modelo_simplificado,  'k--', 'LineWidth', 2);

grid on;

xlabel('Tiempo [s]');
ylabel('Posición [cm]');

legend('Medición', 'Modelo Completo', 'Modelo Simplificado', 'Location', 'best');

title('Comparación de la posición');

exportgraphics(gcf, 'completo_vs_simpificado.pdf', 'ContentType', 'image', 'Resolution', 300);

%% Cuentas

polos = roots([1, q1, q0]);

modulo = abs(polos);

omega_n = sqrt(q0);

zeta = q1/(2 * omega_n);
