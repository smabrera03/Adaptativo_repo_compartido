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

load("datos_para_comparacion_v2.mat");
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

%%%% ============================================================
% ANALISIS DE CORRELACION Y ESPECTRAL
% Modelo no lineal completo
% =============================================================

% Entrada y salida
u_analisis = u_med(:);                 % Ángulo servo [rad]
y_analisis = x_modelo_completo(:);     % Posición simulada [cm]

% Quito la media para analizar las variaciones alrededor
% del punto de operación
u0 = detrend(u_analisis, 0);
y0 = detrend(y_analisis, 0);

N = length(u0);

fprintf('\n--- ANALISIS DE SEÑALES ---\n');
fprintf('Número de muestras: %d\n', N);
fprintf('Frecuencia de muestreo: %.2f Hz\n', fs);
fprintf('Periodo de muestreo: %.4f s\n', Ts);


%% ============================================================
% 1. CORRELOGRAMAS
% =============================================================

% Máximo retardo que queremos visualizar
maxLag_s = 5;                   
maxLag = round(maxLag_s/Ts);

% Autocorrelaciones
[Ruu, lags] = xcorr(u0, maxLag, 'coeff');
[Ryy, ~]    = xcorr(y0, maxLag, 'coeff');

% Correlación cruzada entrada-salida
[Ruy, ~] = xcorr(u0, y0, maxLag, 'coeff');

% Retardo en segundos
tau = lags * Ts;


figure;

plot(tau, Ruu, 'LineWidth', 1.5);
grid on;

xlabel('Retardo \tau [s]');
ylabel('R_{uu}(\tau)');
title('Autocorrelación de la entrada');


figure;

plot(tau, Ryy, 'LineWidth', 1.5);
grid on;

xlabel('Retardo \tau [s]');
ylabel('R_{yy}(\tau)');
title('Autocorrelación de la salida');


figure;

plot(tau, Ruy, 'LineWidth', 1.5);
grid on;

xlabel('Retardo \tau [s]');
ylabel('R_{uy}(\tau)');
title('Correlación cruzada entrada-salida');


%% ============================================================
% 2. PERIODOGRAMA
% =============================================================

[Puu_per, f_per] = periodogram(u0, [], N, fs);
[Pyy_per, ~]     = periodogram(y0, [], N, fs);


figure;

semilogy(f_per, Puu_per, 'LineWidth', 1.5);
grid on;

xlabel('Frecuencia [Hz]');
ylabel('PSD');
title('Periodograma de la entrada');


figure;

semilogy(f_per, Pyy_per, 'LineWidth', 1.5);
grid on;

xlabel('Frecuencia [Hz]');
ylabel('PSD');
title('Periodograma de la salida');


%% ============================================================
% 3. ESTIMACION ESPECTRAL MEDIANTE WELCH
% =============================================================

% Longitud de ventana
nwin = min(1024, N);

window = hann(nwin);

% 50 % de solapamiento
noverlap = floor(nwin/2);

% Cantidad de puntos FFT
nfft = max(1024, 2^nextpow2(nwin));


% Autoespectro entrada
[Suu, f] = pwelch( ...
    u0, ...
    window, ...
    noverlap, ...
    nfft, ...
    fs);


% Autoespectro salida
[Syy, ~] = pwelch( ...
    y0, ...
    window, ...
    noverlap, ...
    nfft, ...
    fs);


% Espectro cruzado entrada-salida
[Suy, ~] = cpsd( ...
    u0, ...
    y0, ...
    window, ...
    noverlap, ...
    nfft, ...
    fs);


figure;

semilogy(f, Suu, 'LineWidth', 1.5);
grid on;

xlabel('Frecuencia [Hz]');
ylabel('\Phi_{uu}(f)');
title('Espectro de potencia de la entrada');


figure;

semilogy(f, Syy, 'LineWidth', 1.5);
grid on;

xlabel('Frecuencia [Hz]');
ylabel('\Phi_{yy}(f)');
title('Espectro de potencia de la salida');


figure;

semilogy(f, abs(Suy), 'LineWidth', 1.5);
grid on;

xlabel('Frecuencia [Hz]');
ylabel('|\Phi_{uy}(f)|');
title('Espectro cruzado entrada-salida');


%% ============================================================
% 4. COHERENCIA
% =============================================================

Cuy = mscohere( ...
    u0, ...
    y0, ...
    window, ...
    noverlap, ...
    nfft, ...
    fs);


figure;

plot(f, Cuy, 'LineWidth', 1.5);
grid on;

xlabel('Frecuencia [Hz]');
ylabel('\gamma^2_{uy}(f)');
title('Coherencia entrada-salida');

ylim([0 1]);
xlim([0 fs/2]);