%% DERIVANDO
clear all; clc, close all;
%%
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
%%

s = tf('s');
% Simular el sistema a lazo abierto, derivar la salida, identificar:
G_der = g * k * 1/(s + b/m) * 1/(s^2 + q1 * s + q0); % <-- TRANSFERENCIA A IDENTIFICAR

amplitud = 0.5; %Amplitud del ruido
u_der = amplitud*sign(randn(N,1)); %Señal de referencia
u_der = u_der - mean(u_der);

u_der_cont = @(tt) interp1(t,u_der,tt,'linear','extrap');
%% DEFINICIÓN DEL MODELO Y GENERACIÓN DE LAS SEÑALES

modelo_der = @(tt,x) [
    x(2);
    g*sin(x(3)) - (b/m)*x(2);
    x(4);
    -q0*x(3) - q1*x(4) + k* u_der_cont(tt)
];

x0 = [0;0;0;0];
opts = odeset('MaxStep',Ts/5);
[t_sol,x_sol] = ode45(modelo_der,t,x0,opts);

y0 = x_sol(:,2); %--> ACÁ TOMO LA DERIVADA
%NOTA: derivo ANTES de poner el ruido
e = amplitud/10 * randn(N, 1);
y = y0 + e;
y = y - mean(y);

%% GRÁFICOS

figure;
subplot(2, 1, 1);
plot(t, u_der, 'LineWidth', 2);
grid on;
ylabel('Entrada [m]');
ylim([-1.1*amplitud, 1.1*amplitud]);
xlabel('Tiempo [s]');

subplot(2, 1, 2);
plot(t, y, 'LineWidth', 2),
grid on;
ylabel('Salida [m/s]');
xlabel('Tiempo [s]');

fig5 = gcf;
exportgraphics(fig5, 'NP graficos/Derivada/Entrada y salida.pdf', 'ContentType', 'image', 'Resolution', 300);

%% FFT DE LA CORRALACIÓN

[h_verdadera_G_der, t_imp] = impulse(G_der);
plot(t_imp, h_verdadera_G_der, 'LineWidth', 2); hold on;

lags = 2000;
R = covf([y, u_der], lags + 1);
h_cor_der = R(2,:)'/R(4,1);
plot(t(1:lags+1), h_cor_der/Ts, 'r--', 'LineWidth', 2);
legend('Estimada', 'Verdadera');
grid on;
xlim([t_imp(1), t_imp(end)]);

fig6 = gcf;

exportgraphics(fig6, 'NP graficos/Derivada/respuesta al impulso.pdf', 'ContentType', 'image', 'Resolution', 300);
%%
N_fft = 2048;
G_fft = fft(h_cor_der, N_fft);

% Vector de frecuencias [rad/s]
w_fft = (0:N_fft-1)' * fs * 2 * pi /N_fft;

% Nos quedamos con las frecuencias positivas hasta Nyquist
idx = 1:floor(N_fft/2)+1;
w_fft = w_fft(idx);
G_fft = G_fft(idx);


%% ETFE

data = iddata(y, u_der, Ts);
G_etfe = etfe(data, [], N_fft/2 + 1);

w_etfe = G_etfe.Frequency;
[G_etfe_bode, ~] = freqresp(G_etfe, w_etfe);
G_etfe_bode = squeeze(G_etfe_bode);


%% WELCH
ventana = hamming(256);
noverlap = 64;

[Pyu, f_welch] = cpsd(y, u_der, ventana, noverlap, N_fft, fs);
w_welch = 2 * pi * f_welch;

[Puu, ~] = pwelch(u_der, ventana, noverlap, f_welch, fs);

G_welch = Pyu./Puu;

G_welch = frd(G_welch, w_welch);
G_welch_bode = squeeze(G_welch.ResponseData);

%% CÓMPUTO DEL BODE TEÓRICO
G_teorica = squeeze(freqresp(G_der,w_fft));

%% COMPARACIÓN EN BODES
W_MIN = w_fft(1);
W_MAX = 30;
figure;

subplot(2,1,1);
semilogx(w_fft, 20*log10(abs(G_fft)), 'r-', 'LineWidth', 1.5); hold on;
semilogx(w_etfe, 20*log10(abs(G_etfe_bode)), 'k', 'LineWidth', 1.5); hold on;
semilogx(w_welch, 20*log10(abs(G_welch_bode)), 'm', 'LineWidth', 1.5); hold on;
semilogx(w_fft, 20*log10(abs(G_teorica)), 'b', 'LineWidth', 1.5);

grid on;
ylabel('Magnitud [dB]');
legend('FFT h\_cor', 'ETFE', 'Welch',  'Verdadero', 'Location', 'SouthWest');
title('Bode de G\_der');
xlim([W_MIN, W_MAX]);

subplot(2,1,2);
semilogx(w_fft, rad2deg(unwrap(angle(G_fft))), 'r-', 'LineWidth', 1.5); hold on;
semilogx(w_etfe, rad2deg(unwrap(angle(G_etfe_bode))), 'k', 'LineWidth', 1.5); hold on;
semilogx(w_welch, rad2deg(unwrap(angle(G_welch_bode))), 'm', 'LineWidth', 1.5); hold on;
semilogx(w_fft, rad2deg(unwrap(angle(G_teorica))), 'b', 'LineWidth', 1.5);

ylim([-400, 40]);
grid on;
xlabel('\omega [rad/s]');
ylabel('Fase [°]');
xlim([W_MIN, W_MAX]);

fig7 = gcf;
exportgraphics(fig7, 'NP graficos/Derivada/comparación rta en frec.pdf', 'ContentType', 'image', 'Resolution', 300);