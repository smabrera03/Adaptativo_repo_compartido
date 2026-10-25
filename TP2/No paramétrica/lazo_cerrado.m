close all; clc; clear;

%% PARAMETROS DEL SISTEMA
m  = 0.033;       
b  = 0.15;       
g  = 9.8;         
%Coeficientes del denominador de la transferencia
q0 = 33.1549;
q1 = 9.736; 
%Numerador de la transferencia
k = 11.4345;
a0 = deg2rad(-3.9);   % Offset servo-barra [rad]
a1 = 0.34;            % Pendiente servo-barra


fs = 50;              % Frecuencia de muestreo [Hz]
Ts = 1/fs;            % Periodo de muestreo [s]


N = 10000; %Largo de la muestra a utilizar

t = (0:N-1)'*Ts; %Vector de tiempos

%% TRANSFERENCIA DEL SISTEMA LINEALIZADO (PARA COMPARAR)
s = tf('s');
G = g * k * 1/s * 1/(s + b/m) * 1/(s^2 + q1 * s + q0); % <-- TRANSFERENCIA A IDENTIFICAR
Gd = c2d(G, Ts);

kp = 2;

T = feedback(kp * G, 1); 
Td = c2d(T, Ts);

%% GENERACIÓN DE LA ENTRADA
amplitud = 0.5; %Amplitud del ruido
rng(1); %Semilla para replicabilidad
r = amplitud*sign(randn(N,1)); %Señal de referencia
r = r - mean(r);

r_continua = @(tt) interp1(t,r,tt,'linear','extrap');

%% DEFINICIÓN DEL MODELO NO LINEAL

modelo = @(tt,x) [
    x(2);
    g*sin(x(3)) - (b/m)*x(2);
    x(4);
    -q0*x(3) - q1*x(4) + k* kp * (r_continua(tt) - x(1))
];

x0 = [0; 0; 0; 0];
opts = odeset('MaxStep',Ts/5);

%% GENERACIÓN DE SEÑALES

[t_sol,x_sol] = ode45(modelo,t,x0,opts);

y0 = x_sol(:,1);
e = amplitud/10 * randn(N, 1);
y = y0 + e;
y = y - mean(y);

%Recuperamos u
u = kp * (r - y);

%% GRAFICOS
figure('Position', [150, -100, 800, 600]);

subplot(3, 1, 1);
plot(t_sol, r);

ylim([-1.1*amplitud, 1.1*amplitud]);
ylabel('r [m]');
xlabel('Tiempo [s]');
xlim([0, 20]);
grid on;
title('Referencia');

subplot(3, 1, 2);
plot(t_sol, u);

ylabel('u [rad]');
xlabel('Tiempo[s]');
xlim([0, 20]);
grid on;
title('Acción de control');

subplot(3, 1, 3);
plot(t_sol, y);

ylabel('y [m]');
xlabel('Tiempo [s]');
xlim([0, 20]);
grid on;
title('Posición');

fig1 = gcf;
exportgraphics(fig1, 'NP graficos/LC/r_u_y.pdf', 'ContentType', 'image', 'Resolution', 300);

%% FFT DE LA CORRELACIÓN

figure;
[h_verdadera_T, t_imp] = impulse(T);
plot(t_imp, h_verdadera_T, 'LineWidth', 2); hold on;

lags = 2000;
R = covf([y, r], lags + 1);
h_cor_T = R(2,:)'/R(4,1);

plot(t(1:lags+1), h_cor_T/Ts, 'r--', 'LineWidth', 1);
grid();
xlim([0, 6]);
xlabel('Tiempo [s]');
ylabel('h(t) [m]');
legend('Verdadera', 'Estimada');

fig2 = gcf;
exportgraphics(fig2, 'NP graficos/LC/respuesta al impulso.pdf', 'ContentType', 'image', 'Resolution', 300);

N_fft = 2048;
T_fft = fft(h_cor_T, N_fft);

% Vector de frecuencias [rad/s]
w = (0:N_fft-1)' * fs * 2 * pi /N_fft;

% Nos quedamos con las frecuencias positivas hasta Nyquist
idx = 1:floor(N_fft/2)+1; %el primer componente es 1, así que incluye la continua
w = w(idx);
T_fft = T_fft(idx);

%% ETFE
data = iddata(y, r, Ts);
T_etfe = etfe(data, [], N_fft/2 + 1);

w_etfe = T_etfe.Frequency;

[T_etfe_bode, ~] = freqresp(T_etfe, w_etfe);
T_etfe_bode = squeeze(T_etfe_bode);

%% WELCH

ventana = hamming(256);
noverlap = 64;
%NOTA: Welch divide la señal el segmentos, calcula la PSD de cada uno y
%promedia. Cuanto más cortos son los segmentos, más segmentos hay, más
%cosas se promedian y más menos varianza tiene el resultado.

[Pyr, f_welch] = cpsd(y, r, ventana, noverlap, N_fft, fs, 'twosided');
w_welch = 2 * pi * f_welch;

[Prr, ~] = pwelch(r, ventana, noverlap, f_welch, fs, 'twosided');
T_welch = Pyr./Prr;


T_welch = frd(T_welch, w_welch);
T_welch_bode = squeeze(T_welch.ResponseData);
%% CÓMPUTO DEL BODE TEÓRICO
T_teorica = squeeze(freqresp(T,w));

%% COMPARACIÓN EN BODES
W_MIN = w(2);
W_MAX = 30;
figure;

subplot(2,1,1);
semilogx(w, 20*log10(abs(T_fft)), 'r-', 'LineWidth', 1.5); hold on;
semilogx(w_etfe, 20*log10(abs(T_etfe_bode)), 'k', 'LineWidth', 1.5); hold on;
semilogx(w_welch, 20*log10(abs(T_welch_bode)), 'm', 'LineWidth', 1.5); hold on;
semilogx(w, 20*log10(abs(T_teorica)), 'b--', 'LineWidth', 2);
xlim([W_MIN, W_MAX]);
grid on;
ylabel('Magnitud [dB]');
legend('FFT h\_cor', 'ETFE', 'Welch', 'Verdadero', 'Location', 'SouthWest');
title('Bode de T comparación');

subplot(2,1,2);
semilogx(w, rad2deg(unwrap(angle(T_fft))), 'r-', 'LineWidth', 1.5); hold on;
semilogx(w_etfe, rad2deg(unwrap(angle(T_etfe_bode))), 'k', 'LineWidth', 1.5); hold on;
semilogx(w_welch, rad2deg(unwrap(angle(T_welch_bode))), 'm' , 'LineWidth', 1.5); hold on;
semilogx(w, rad2deg(unwrap(angle(T_teorica))), 'b--', 'LineWidth', 1.5);
xlim([W_MIN, W_MAX]);
ylim([-400, 40]);
grid on;
xlabel('\omega [rad/s]');
ylabel('Fase [°]');

fig3 = gcf;
exportgraphics(fig3, 'NP graficos/LC/T_comparacion rta en frec.pdf', 'ContentType', 'image', 'Resolution', 300);

%% PERO YO QUERÍA G, NO T

G_fft = T_fft./(kp * (1 - T_fft));
G_etfe_bode = T_etfe_bode./(kp * (1 - T_etfe_bode));
G_welch_bode = T_welch_bode./(kp * (1 - T_welch_bode));
G_teorica = squeeze(freqresp(G,w));

%% COMPARACIÓN EN BODES
figure;

subplot(2,1,1);
semilogx(w, 20*log10(abs(G_fft)), 'r-', 'LineWidth', 1.5); hold on;
semilogx(w_etfe, 20*log10(abs(G_etfe_bode)), 'k', 'LineWidth', 1.5); hold on;
semilogx(w_welch, 20 * log(abs(G_welch_bode)), 'm', 'LineWidth', 1.5);
semilogx(w, 20*log10(abs(G_teorica)), 'b--', 'LineWidth', 1.5);
xlim([W_MIN, W_MAX]);
grid on;
ylabel('Magnitud [dB]');
legend('FFT h\_cor', 'ETFE', 'Welch', 'Verdadero', 'Location', 'SouthWest');
title('Bode de G comparación');

subplot(2,1,2);
semilogx(w, rad2deg(unwrap(angle(G_fft))), 'r-', 'LineWidth', 1.5); hold on;
semilogx(w_etfe, rad2deg(unwrap(angle(G_etfe_bode))), 'k', 'LineWidth', 1.5); hold on;
semilogx(w_welch, rad2deg(unwrap(angle(G_welch_bode))), 'm', 'LineWidth', 1.5); hold on;
semilogx(w, rad2deg(unwrap(angle(G_teorica))), 'b--', 'LineWidth', 1.5);
xlim([W_MIN, W_MAX]);
ylim([-400, 0]);
grid on;
xlabel('\omega [rad/s]');
ylabel('Fase [°]');

fig4 = gcf;
exportgraphics(fig4, 'NP graficos/LC/G_comparacion rta en frec.pdf', 'ContentType', 'image', 'Resolution', 300); 



