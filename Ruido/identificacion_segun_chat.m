%{
Problema según chat:

En realidad no conozco la entrada, no conozco el ruido blanco que
supuestamente generaría el ruido en la posición medida. Solo conozco la
medición con ruido. 

Chat me propone:
1) Asumir algún tipo de modelo (AR/ARMA)
2) hallar los coeficientes que mejor repliquen la PSD.

Problema: La PSD no se parece a nada que yo haya visto. 
%}

clear; clc, close all;

load('Mediciones/medicion_v1.mat');

Ts = 0.02;
fs = 1/Ts;

N = length(pos);

pos_c = pos - mean(pos);


data = iddata(pos_c, [], Ts); %set de datos

%asumo modelo ar de orden na
na = 2;
modelo_ar = ar(data, na);
%pole(modelo_ar) %¿No da nada?

A = modelo_ar.A;
B = 1;
H = tf(B, A, Ts, 'Variable', 'z^-1');

[Hf, f] = freqz(B, A, 4096, fs);

figure;

semilogx(f, 20*log10(abs(Hf)));

xlabel('Frecuencia [Hz]');
ylabel('|H(f)| [dB]');
title('Respuesta en frecuencia del filtro identificado');
grid on;
xlim([0.01 fs/2]);

%% Validación

rng(1);
w = randn(N,1);

y_modelo = lsim(H, w, t); %entra ruido blanco, salie y_modelo y . ¿Se parecen y_modelo y pos_c?

[P_pos, f] = pwelch(pos_c, [], [], [], fs);
[P_mod, ~] = pwelch(y_modelo, [], [], [], fs);

figure;

semilogx(f, 10*log10(P_pos), ...
    'DisplayName', 'Medición');

hold on;

semilogx(f, 10*log10(P_mod), ...
    'DisplayName', 'Modelo');

xlabel('Frecuencia [Hz]');
ylabel('PSD [dB]');
title('PSD: medición vs modelo');

legend;
grid on;

xlim([0.01 fs/2]);

%No tiene nada que ver con nada