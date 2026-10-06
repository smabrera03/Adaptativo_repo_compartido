clear; clc, close all;

A = [0, 1; 0, -0.15/(33/1000)];
B = [0; 0.34 * 9.8];
C = [1, 0];
D = 0;

fs = 50;              % Frecuencia de muestreo [Hz]
Ts = 1/fs;            % Periodo de muestreo [s]


N = 10000; %Largo de la muestra a utilizar

t = (0:N-1)'*Ts; %Vector de tiempos

amplitud = 1; %Amplitud del ruido
rng(2); %Semilla para replicabilidad
r = amplitud*sign(randn(N,1)); %Señal de entrada: signo de una gaussiana

sys_c = ss(A,B,C,D);
sys_or_d = c2d(sys_c,Ts,'zoh');
k = 2; %ganancia del controlador
controlador = tf(k); %Obs: con el controlador creo que terminé identificando la respuesta al impulso del sistema realimentado.
%sistema r --> y
sys_ry_c = feedback(controlador * sys_c, 1);
sys_ry = c2d(sys_ry_c,Ts,'zoh');

%sistema r --> u
sys_ru_c = feedback(controlador, sys_c);
sys_ru = c2d(sys_ru_c, Ts, 'ZOH');

% Simular
y = lsim(sys_ry,r,t);

u = lsim(sys_ru, r, t);
% Graficar
figure;
subplot(3,1,1)
plot(t,r)
xlabel('Tiempo [s]')
ylabel('r')

subplot(3,1,2)
plot(t,y)
xlabel('Tiempo [s]')
ylabel('y')

subplot(3, 1, 3);
plot(t, u);
xlabel('Tiempo [s]');
ylabel('u = r - k * y');

%% Simulo rta al impulso

u_imp = zeros(N, 1);
u_imp(1) = 1;
h_real = lsim(sys_or_d, u_imp, t);

figure;
subplot(2,1,1)
stem(t,u_imp)
xlabel('Tiempo [s]')
ylabel('u')

subplot(2,1,2)
stem(t,h_real)
xlabel('Tiempo [s]')
xlim([0, 7]);
ylabel('y')

lags = 9999; %lags a considerar
[h_cor, R, ~] = cra([dtrend(y, 0), dtrend(u, 0)], lags, 0, 0);
% R --> Matriz con autocovarianzas y covarianzas cruzadas
% R(:, 1) --> indices de lag || R(:, 2) --> R_yy || R(:, 3) --> R_uu || R(:, 4) -->
% R_yu ||

% Comparación con la respuesta al impulso del modelo
figure;
plot([h_real(1:lags) h_cor(1:lags)],'LineWidth',2);
xlim([0, 1000]);
grid;
legend('Respuesta al impulso','Estimación por correlación', 'Location', 'southeast')

figure;
cra(R);

%% Identificación no paramétrica en frecuencia

%ETFE: cociente de DFT´s
z = iddata(y,r,1);
H_etfe = etfe(z);

% Welch. Cociente de PSD´s
Pyu = cpsd(y,r);
Puu = pwelch(r);
H_welch = Pyu./Puu;
w_welch = (0:length(H_welch)-1)'*pi/length(H_welch);
H_welch  = frd(H_welch ,w_welch);

% Respuesta en frecuencia del correlograma
N = 256;
H_cor = fft(h_cor,N);
w_cor = (0:N-1)'*2*pi/N;
H_cor = frd(H_cor,w_cor);

figure
bode(w_cor,H_etfe,H_welch, H_cor, sys_c)
grid on;

ylim([-360, 360]);


legend('ETFE','Welch', 'Correlograma', 'Modelo')
title('Espectros')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
