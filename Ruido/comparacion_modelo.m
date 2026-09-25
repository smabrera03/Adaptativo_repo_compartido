clear; clc; close all;

load('Mediciones/medicion_v8');

% 1. PARAMETROS
m  = 4.0/1000;       % Masa de la bola [kg]
b  = 0.15/3;        % Rozamiento efectivo [kg/s] NOTA: Tuve que reajustar b a ojo
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

%Grafico la entrada
plot(t, ang_servo, 'LineWidth', 2); grid on;
ylim([-25, 25]);
xlabel('Tiempo [s]');
ylabel('Ángulo comandado[°]');


% Convertir ángulos de grados a radianes
u_med       = deg2rad(ang_servo);
theta_med   = deg2rad(ang_barra);

% Posición medida
x_med = pos/100; %paso la salida a metros

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

x0 = [
    x_med(1);        % posición inicial pasada a [m]
    0;               % velocidad inicial
    theta_med(1);    % ángulo inicial de la barra pasado a [rad]
    0                % velocidad angular inicial
];

opts = odeset('MaxStep', Ts/5);

[t_sim_completo, x_sim] = ode45(modelo_completo, t, x0, opts);


x_modelo_completo = x_sim(:,1) * 100;       % Posición modelada [cm]

f_filtro = 2.5; %frecuencia de corte del filtro (en Hz)
orden = 2;
Wn = f_filtro/(fs/2);   % Frecuencia normalizada
[b, a] = butter(orden, Wn, 'low');
% Filtrado
pos_filtrada = filter(b, a, pos);

%% Gráficos
figure;

plot(t, x_med * 100, 'LineWidth', 2);
hold on;
plot(t_sim_completo, x_modelo_completo, '--', 'LineWidth', 2);
hold on;
plot(t, pos_filtrada, 'LineWidth', 2);
grid on;

xlabel('Tiempo [s]');
ylabel('Posición [cm]');

legend('Medición', 'Modelo Completo', 'Location', 'best');

title('Comparación de la posición');

%exportgraphics(gcf, 'completo_vs_simpificado.pdf', 'ContentType', 'image', 'Resolution', 300);

%% Cuentas

polos = roots([1, q1, q0]);

modulo = abs(polos);

omega_n = sqrt(q0);

zeta = q1/(2 * omega_n);
