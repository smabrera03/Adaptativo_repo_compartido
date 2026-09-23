clear; clc, close all;

load('Mediciones/medicion_v1.mat');

Ts = 0.02;
fs = 1/Ts;

N = length(pos);

pos_c = pos - mean(pos);

rng(1); %semilla

w = randn(N, 1);

[r_yw,lags] = xcorr(pos_c,w,'biased'); %¿Por qué biased? Entre biased un unbiased el comportamiento solo cambia para lags grandes, así que creo que no importa

h_est = r_yw; %respuesta al impulso como correlación cruzada entre entrada y salida

figure;

stem(lags ,h_est,'.');
xlabel('n');
ylabel('h[n]');
title('Respuesta al impulso estimada mediante correlación cruzada');
N_max = 50;
xlim([-floor(N_max/5), N_max]);
grid on;
