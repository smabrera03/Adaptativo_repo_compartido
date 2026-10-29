clear; close all; clc;

load('datos_prueba.mat');

fprintf('Media de u: %.3f  °\n', mean(u));
fprintf('Media de tita: %.3f  °\n', mean(y));

figure;
subplot(2, 1, 1);
plot(t, u); grid on;
subplot(2, 1, 2);
plot(t, y); grid on;

u = deg2rad(u - mean(u));
y = deg2rad(y - mean(y));


fs = 50;

NFFT = 128;
ventana = hamming(NFFT);
noverlap = NFFT/2; 

[PSD, f] = pwelch(u, [], noverlap, NFFT, fs,  'twosided');
w = 2*pi * f;

figure;
plot(w, PSD, 'LineWidth', 2); grid on;
xlim([0, 2 * pi * 5]);

