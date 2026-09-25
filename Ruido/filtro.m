clear; clc; close all;
load('Mediciones/medicion_v7');

figure;
plot(t, pos); grid on;
xlim([10, 15]);


Ts = 0.02;
fs = 1/Ts;

f_filtro = 5; %frecuencia de corte del filtro (en Hz)

orden = 2;
Wn = f_filtro/(fs/2);   % Frecuencia normalizada

[b, a] = butter(orden, Wn, 'low');

% Filtrado
pos_filtrada = filter(b, a, pos);

% Comparación
figure;
plot(t, pos, 'DisplayName', 'Posición original');
hold on;
plot(t, pos_filtrada, 'DisplayName', 'Posición filtrada'); hold on;
yline(mean(pos), 'r--', 'DisplayName', 'Media', 'LineWidth', 2);
grid on;

xlabel('Tiempo [s]');
ylabel('Posición [cm]');
legend;
xlim([100 200]);

std_original = std(pos);
std_filtrada = std(pos_filtrada);

fprintf('Potencia original %.4f\n', std_original);
fprintf('Potencia filtrada %.4f\n', std_filtrada);
fprintf('Proporción %.4f\n', std_filtrada/std_original);
