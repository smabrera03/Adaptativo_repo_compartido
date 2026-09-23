%Análisis del ruido en MATLAB
%No filtrar mediciones antes de caracterizar el ruido
%Calculamos: Frecuencia de muestreo efectiva y porcentaje de lecturas inválidas.
%Media, mediana, desviación estándar y RMS de las fluctuaciones.
%Histograma del ruido.
%Autocorrelación.
%Densidad espectral de potencia mediante pwelch.


%% CARACTERIZACION DEL RUIDO - HC-SR04

clear;
clc;
close all;

%% 1. Cargar datos

T = readtable("ruido_hcsr04_centro.csv");

t = double(T.t_us) * 1e-6;

y = double(T.distancia_cm);

valid = logical(T.valid) & isfinite(y);

%% 2. Verificar adquisicion

dt = diff(t);

fs_efectiva = 1 / median(dt);

porcentaje_invalidas = ...
    100 * sum(~valid) / numel(valid);

fprintf("Muestras totales: %d\n", numel(y));

fprintf("Frecuencia efectiva: %.2f Hz\n", ...
    fs_efectiva);

fprintf("Lecturas invalidas: %.2f %%\n", ...
    porcentaje_invalidas);

%% 3. Conservar mediciones validas

tv = t(valid);

yv = y(valid);

if numel(yv) < 10
    error("No hay suficientes muestras validas.");
end

%% 4. Estadisticos

media = mean(yv);

mediana = median(yv);

ruido = yv - media;

sigma = std(ruido);

rms_ruido = sqrt(mean(ruido.^2));

pico_pico = max(yv) - min(yv);

fprintf("\n--- ESTADISTICOS ---\n");

fprintf("Media: %.4f cm\n", media);

fprintf("Mediana: %.4f cm\n", mediana);

fprintf("Desviacion estandar: %.4f cm\n", sigma);

fprintf("RMS ruido: %.4f cm\n", rms_ruido);

fprintf("Pico a pico: %.4f cm\n", pico_pico);

%% 5. Serie temporal

figure;

plot(t, y, ".");

xlabel("Tiempo [s]");
ylabel("Distancia [cm]");

title("Mediciones del HC-SR04");

grid on;

%% 6. Ruido respecto de la media

figure;

plot(tv, ruido);

xlabel("Tiempo [s]");
ylabel("Ruido [cm]");

title("Fluctuaciones respecto de la media");

grid on;

%% 7. Histograma

figure;

histogram(ruido, 40, ...
    "Normalization", "pdf");

xlabel("Ruido [cm]");
ylabel("Densidad");

title("Distribucion del ruido");

grid on;

%% 8. Autocorrelacion

% Solo utilizar esta estimacion si el muestreo
% es aproximadamente uniforme y las lecturas
% invalidas son escasas.

if all(valid) && ...
   max(abs(dt - median(dt))) < 0.1 * median(dt)

    [R, lags] = xcorr( ...
        ruido, ...
        "coeff" ...
    );

    tau = lags / fs_efectiva;

    figure;

    plot(tau, R);

    xlim([-2 2]);

    xlabel("Retardo [s]");
    ylabel("Autocorrelacion");

    title("Autocorrelacion del ruido");

    grid on;

    %% 9. Densidad espectral de potencia

    figure;

    pwelch( ...
        ruido, ...
        [], ...
        [], ...
        [], ...
        fs_efectiva ...
    );

    title("Densidad espectral del ruido");

else

    warning([ ...
        "Hay lecturas invalidas o muestreo irregular. " ...
        "No se calcula autocorrelacion ni PSD " ...
        "como si la serie fuera uniforme." ...
    ]);

end
