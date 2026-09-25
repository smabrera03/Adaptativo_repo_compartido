clear; clc, close all;

load('Mediciones/medicion_v7.mat');

%pos: posición del carro en cm

N = length(pos);
Ts = 0.02;
fs = 1/Ts;

m_pos = mean(pos);
var_pos = var(pos);
std_pos = std(pos);
maximo = max(pos);
minimo = min(pos);
 
fprintf('\nMedia: %.4f\n', m_pos);
fprintf('Varianza: %.6f\n', var_pos);
fprintf('Desvío estándar: %.4f\n', std_pos);
fprintf('Valor máximo: %f\n', maximo);
fprintf('Valor mínimo: %f\n', minimo);

figure;
plot(t, pos);
xlabel('Tiempo [s]'); 
ylabel('Posición medida [cm]');

yline(m_pos, 'k--', 'LineWidth', 2);
yline(m_pos + 3 * std_pos, 'r--', 'LineWidth', 2);
yline(m_pos - 3 * std_pos, 'r--', 'LineWidth', 2);

title('Medición (transitorio ya descartado)');
legend('Medicion', 'Media', 'Media + 3 * Desvío', 'Media - 3 * Desvío');
xlim([0, 30]);
grid on;


pos_c = pos - m_pos; %Señal centrada


% 3) Chequeo cualitativo de estacionariedad por tramos
n_tramos = 5;
L = floor(N/n_tramos);
fprintf('\nChequeo de estacionariedad por tramos:\n');
for i = 1:n_tramos
    idx = (i-1)*L+1 : i*L;
    fprintf('Tramo %d: media = %.4f, var = %.6f\n', i, mean(pos(idx)), var(pos(idx)));
end

% ¿Es estacionario? Debatible 
%¿Debería probar con una muestra más larga?

% 4) Histograma
figure;
histogram(pos, 'Normalization', 'pdf', 'BinWidth', std_pos);
xlabel('Posición'); ylabel('Densidad');
title('Histograma de la medición');
xline(minimo, 'r--');
xline(maximo, 'r--');
grid on;
 
% Comparación contra una gaussiana con la media y varianza estimadas
hold on;
x_vals = linspace(min(pos), max(pos), 10000);
pdf_gauss = normpdf(x_vals, m_pos, std_pos);
plot(x_vals, pdf_gauss, 'r-', 'LineWidth', 1.5);
legend('Histograma', 'Mínimo', 'Máximo', 'Gaussiana ajustada');

%Evidentemente no es gaussiana. ¿Sigue alguna distribución?

% 5) Autocorrelación / Autocovarianza
[r_hat, lags] = xcov(pos_c, 'biased');   % autocovarianza estimada
r_norm = r_hat / r_hat(lags==0);                  % autocorrelación normalizada
 
figure;
subplot(2,1,1);
plot(lags, r_hat);
xlabel('Lag'); ylabel('r(k)');
max_lag = 20;
xlim([-max_lag, max_lag]);
title('Autocovarianza estimada');
grid on;
 
subplot(2,1,2);
plot(lags, r_norm);
xlabel('Lag'); ylabel('\rho(k)');
title('Autocorrelación normalizada');
xlim([-max_lag, max_lag]);
grid on;

% 6) Densidad espectral de potencia (PSD)
 
[P_welch, f_welch] = pwelch(pos_c, fs);
%pwelch divide la muestra en varias secciones y calcula una PSD para cada
%una. El resultado es el promedio de las PSD, lo que dismimuye la varianza.
figure;
semilogx(f_welch, 10*log10(P_welch), 'LineWidth', 2); grid on;
xlabel('Frecuencia [Hz]'); ylabel('PSD [dB]');
xlim([min(f_welch), max(f_welch)]);
%{
¿Qué información me da la PSD?
¿Por qué parece que el ruido tiene principalmente una componente de
continua? ¿Será por la ventana?

%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%¿Cómo sé si el proceso es ESA?
