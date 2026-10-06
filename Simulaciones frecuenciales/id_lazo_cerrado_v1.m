%{

En esta versión se hace la identificación en base al modelo LINEAL


%}

close all; clc; clear;


%%%%%%%%%%%%%%%%%%
%% 0) MODELO LINEAL (PARA COMPARAR RESULTADOS)
%%%%%%%%%%%%%%%%%%

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

%Defino la transferencia del modelo linealizado
s = tf('s');

G = g * k * 1/s * 1/(s + b/m) * 1/(s^2 + q1 * s + q0); % <-- TRANSFERENCIA A IDENTIFICAR
Gd = c2d(G, Ts);

kp = 2;

T = feedback(kp * G, 1); 
Td = c2d(T, Ts);

%%%%%%%%%%%
%% 1) SIMULACIÓN Y GENERACIÓN DE LAS SEÑALES
%%%%%%%%%


amplitud = 0.5; %Amplitud del ruido
rng(1); %Semilla para replicabilidad
r = amplitud*sign(randn(N,1)); %Señal de referencia

y0 = lsim(Td, r, t);

u = kp * (r - y0);

e = 0 * randn(N, 1);
y = y0 + e;

figure;

subplot(3, 1, 1);
plot(t, r)
title('r');

subplot(3, 1, 2);
plot(t, u);
title('u');

subplot(3, 1, 3);
plot(t, y);
title('y');


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 2) OBTENCIÓN DE LA RTA AL IMPULSO POR CORRELACIÓN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lags = 500; %lags
g_T = impulse(Td, lags * Ts);

data = iddata(y, r, Ts);
[h_cor, R, ~] = cra(data, lags, 0, 0);

plot(h_cor/Ts, 'LineWidth', 2); hold on;
plot(g_T, 'LineWidth', 2);
xlim([0 lags]);
legend('Correlación', 'Original');
%NO ENTIENDO POR QUÉ HAY QUE MULTIPLICAR POR 1/Ts

figure;
cra(data, lags, 0, 1); hold on;
plot(h_cor/Ts);
xlim([0 lags]);
legend('Gráfico de cra()', 'h\_cor/Ts');
title('Respuesta estimada por cra');


figure;
cra(data, lags, 0, 2);
title('Autocovarianzas y covarianzas cruzadas');
%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% PRUEBA QUE ME PROPUSO EL CHAT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

lags = 100;

% Covarianzas
R = covf([y r],lags+1);

% Estimación por correlación
h_cor = R(2,:)' / R(4,1);

% Respuesta real
h_real = impulse(Td,lags + 1);

% Comparación
figure
plot(0:lags,h_cor,'LineWidth',1.5)
hold on
plot(0:lags,h_real(1:lags+1),'LineWidth',1.5)
grid on

xlabel('k')
ylabel('h[k]')
legend('Correlación','Respuesta real')
title('Identificación por correlación')

figure
bode(Td)
hold on

data = iddata(y,r,Ts);
T_est = etfe(data);

bode(T_est)

grid on
legend('Td','ETFE')