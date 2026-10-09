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

%% FFT DE LA CORRELACIÓN

lags = 2000;
R = covf([y, r], lags + 1);
h_cor_T = R(2,:)'/R(4,1);
plot(t(1:lags+1), h_cor_T/Ts, 'r--'); hold on;
impulse(T);
legend('Estimada', 'Verdadera');

T_fft = fft(h_cor_T);

% Vector de frecuencias [Hz]
N_fft = length(T_fft);
f = (0:N_fft-1)' * fs/N_fft;

% Nos quedamos con las frecuencias positivas hasta Nyquist
idx = 1:floor(N_fft/2)+1;
f = f(idx);
T_fft = T_fft(idx);

%% ETFE
data = iddata(y, r, Ts);
T_etfe = etfe(data);
w = 2*pi*f;                  % [rad/s]
T_etfe_bode = squeeze(freqresp(T_etfe, w));

%% WELCH

Pyr = cpsd(y, r);
Prr = pwelch(r);
T_welch = Pyr./Prr;
w_welch = (0: length(T_welch) - 1)' * pi/length(T_welch);
T_welch = frd(T_welch, w_welch);
T_welch_bode = squeeze(T_welch.ResponseData);
f_welch = w_welch / (2 * pi);
%% CÓMPUTO DEL BODE TEÓRICO
T_teorica = squeeze(freqresp(T,w));

%% COMPARACIÓN EN BODES
figure;

subplot(2,1,1);
semilogx(f, 20*log10(abs(T_fft)), 'r-', 'LineWidth', 2); hold on;
semilogx(f, 20*log10(abs(T_etfe_bode)), 'k', 'LineWidth', 2); hold on;
semilogx(f, 20*log10(abs(T_teorica)), 'b', 'LineWidth', 2); hold on;
xlim([f(2), f(end)]);
grid on;
ylabel('Magnitud [dB]');
legend('FFT h\_cor', 'ETFE', 'Verdadero', 'Location', 'SouthWest');
title('Bode de T estimada por correlación');

subplot(2,1,2);
semilogx(f, rad2deg(unwrap(angle(T_fft))), 'r-', 'LineWidth', 2); hold on;
semilogx(f, rad2deg(unwrap(angle(T_etfe_bode))), 'k', 'LineWidth', 2); hold on;
semilogx(f, rad2deg(unwrap(angle(T_teorica))), 'b', 'LineWidth', 2);
xlim([f(2), f(end)]);
ylim([-400, 40]);
grid on;
xlabel('Frecuencia [Hz]');
ylabel('Fase [°]');
%%
figure;
bode(T_welch, T_etfe, T)
legend('Welch', 'ETFE', 'Verdadero', 'Location', 'SouthWest');
%EL DE WELCH DA RARO
%% PERO YO QUERÍA G, NO T

G_fft = T_fft./(kp * (1 - T_fft));
G_etfe_bode = T_etfe_bode./(kp * (1 - T_etfe_bode));
G_teorica = squeeze(freqresp(G,w));

%% COMPARACIÓN EN BODES
figure;

subplot(2,1,1);
semilogx(f, 20*log10(abs(G_fft)), 'r-', 'LineWidth', 2); hold on;
semilogx(f, 20*log10(abs(G_etfe_bode)), 'k', 'LineWidth', 2); hold on;
semilogx(f, 20*log10(abs(G_teorica)), 'b', 'LineWidth', 2); hold on;
xlim([f(2), f(end)]);
grid on;
ylabel('Magnitud [dB]');
legend('FFT h\_cor', 'ETFE', 'Verdadero', 'Location', 'SouthWest');
title('Bode de T estimada por correlación');

subplot(2,1,2);
semilogx(f, rad2deg(unwrap(angle(G_fft))), 'r-', 'LineWidth', 2); hold on;
semilogx(f, rad2deg(unwrap(angle(G_etfe_bode))), 'k', 'LineWidth', 2); hold on;
semilogx(f, rad2deg(unwrap(angle(G_teorica))), 'b', 'LineWidth', 2);
xlim([f(2), f(end)]);
ylim([-400, 0]);
grid on;
xlabel('Frecuencia [Hz]');
ylabel('Fase [°]');


