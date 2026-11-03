%{

Probar ARX, IV4 y BJ del mismo orden. 

Incluir el integrador al final 

Evaluar qué tan bien replica el modelo la dinámica de la posición

No tiene tanto senitdo evaluar la respuesta al impuslo en la referencia
porque no estoy poniendo un impulso en la entrada de la planta, sino en la
referencia

%}

%% DATOS Y DEFINICIÓN DE PARÁMETROS
clear; close all; clc;

load('datos_cart_beam.mat');

t = datos.t;
ang_barra = datos.q1;
y = datos.q2;
y_der = datos.q2_p;
u = datos.u;
r = datos.r;

Ts = mean(diff(t));

orden = [3, 2, 1];

N_corte = floor(0.7 * length(y_der));

%Conjuntos de identificación y validación
z_id = iddata(y_der(1:N_corte), u(1:N_corte), Ts);
z_val = iddata(y_der(N_corte+1:end), u(N_corte+1:end), Ts);

z_int = iddata(y(N_corte+1:end), u(N_corte+1:end), Ts);  %Este conjunto de datos contempla al integrador

s = tf('s');
integrador = c2d(1/s, Ts, 'zoh');

%% ARX

modeloARX = arx(z_id, orden);

figure;
subplot(2, 1, 1);
compare(z_val, modeloARX);
grid on;
title('Dinámica de la velocidad');
ylabel('Velocidad[m/s]');

modeloARX_int = integrador * modeloARX;

subplot(2, 1, 2);
compare(z_int, modeloARX_int);
grid on;
title('Dinámica de la posición');
ylabel('Posición [m]');

%% ARX + reducción

modeloARXRED = arx(z_id, [8, 8, 1]);

[Ad, Bd, Cd, Dd] = ssdata(modeloARXRED); %devuelve las matrices de la representación en ee del sistema
[Ab, Bb, Cb, M, T] = dbalreal(Ad,Bd,Cd);
disp("Valores singulares de Hankel: ")
M'
%Mantener solo los primeros 3 estados
[Ar, Br, Cr, Dr] = dmodred(Ab,Bb,Cb,Dd,4:length(M)); %elimina los estados indicados en el último vector
[b,a] = ss2tf(Ar, Br, Cr, 0); %genera los coeficientes de los polinomios
modeloARXRED = idpoly(a,b,1,1,[],[],Ts);
present(modeloARXRED);


figure;
subplot(2, 1, 1);
compare(z_val, modeloARXRED);
grid on;
title('Dinámica de la velocidad');
ylabel('Velocidad[m/s]');

modeloARXRED_int = integrador * modeloARXRED;

subplot(2, 1, 2);
compare(z_int, modeloARXRED_int);
grid on;
title('Dinámica de la posición');
ylabel('Posición [m]');
%% IV4

modeloIV4 = iv4(z_id, orden);

figure;
subplot(2, 1, 1),
compare(z_val, modeloIV4);
grid on;
title('Dinámica de la velocidad');
ylabel('Velociad [m/s]');

modeloIV4_int = integrador * modeloIV4;

subplot(2, 1, 2);
compare(z_int, modeloIV4_int);
grid on;
title('Dinámica de la posición');
ylabel('Posición [m]');

%% BOX-JENKINS

ordenBJ = [2 2 2 3 1];

modeloBJ = bj(z_id, ordenBJ);

figure;
subplot(2, 1, 1);
compare(z_val, modeloBJ);
grid on;
title('Dinámica de la velocidad');
ylabel('Velociad [m/s]');


modeloBJ_int = integrador * modeloBJ;

subplot(2, 1, 2);
compare(z_int, modeloBJ_int);
grid on;
title('Dinámica de la posición');
ylabel('Posción [m]');

%% OE

modeloOE = oe(z_id, [2 3 1]);

figure;
subplot(2, 1, 1);
compare(z_val, modeloOE);
grid on;
title('Dinámica de la velocidad');
ylabel('Velociad [m/s]');


modeloOE_int = integrador * modeloOE;

subplot(2, 1, 2);
compare(z_int, modeloOE_int);
grid on;
title('Dinámica de la posición');
ylabel('Posción [m]');

%% BODES

bode(modeloARX_int, modeloARXRED_int, modeloIV4_int, modeloBJ_int, modeloOE_int)
grid on;
legend('ARX', 'ARX reducido', 'IV4', 'BJ', 'OE', 'Location', 'best');

%% RESUMEN DE RESULTADOS

% Modelos con integrador
modelos_vel = {modeloARX, modeloARXRED, modeloIV4, modeloBJ, modeloOE};
modelos_pos = {modeloARX_int, ...
           modeloARXRED_int, ...
           modeloIV4_int, ...
           modeloBJ_int, ...
           modeloOE_int};

nombres = {'ARX'; ...
           'ARX reducido'; ...
           'IV4'; ...
           'Box-Jenkins'; ...
           'OE'};

% Calcular FIT sobre la dinámica de posición
fit_pos = zeros(length(modelos_pos),1);
fit_vel = zeros(length(modelos_pos),1);

for i = 1:length(modelos_pos)

    [~, fit_i_pos] = compare(z_int, modelos_pos{i});
    [~, fit_i_vel] = compare(z_val, modelos_vel{i});
    fit_pos(i) = fit_i_pos;
    fit_vel(i) = fit_i_vel;
end

% Crear tabla resumen
resultados = table(nombres, fit_pos, fit_vel,...
    'VariableNames', {'Modelo', 'Fit_posicion', 'Fit_velociad'});

% Ordenar de mejor a peor
resultados = sortrows(resultados, 'Fit_posicion', 'descend');

disp(' ');
disp('===== RESUMEN DE MODELOS =====');
disp(resultados);