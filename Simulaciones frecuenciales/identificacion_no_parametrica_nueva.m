clear; clc; close all;

%% CAMBIOS
%Se mantiene la estructura.
%Ahora hay una única convención de frecuencias:
% w en rad/s y un único tiempo de muestreo: T_s=0.02 s.
%No se reutiliza N. Antes N = 10000; y después N = 256.
%Ahora es Nmuestras, Nfft_cor y Nfft_welch.
%Saqué por ahora la simulación manual u_imp(1)=1;
%Para verificar los métodos,ver Bode de G_lin.
%La amplitud es 0.01 rad que son 0.573° aprox --> es chiquita!
%Aunque se generan los datos con la transferencia nuestra, tenemos que
%apuntar a sin(theta_b == theta_b. Así, las identificaciones se acercan al
%modelo linealizado.
%Si las curvas siguen sin coincidir, puede deberse a la amplitud.

%% PARÁMETROS DEL SISTEMA
m  = 0.033;       % Masa del carrito [kg]
b  = 0.15;        % Rozamiento efectivo [kg/s]
g  = 9.8;         % Gravedad [m/s^2]

% Dinámica servo-barra
q0 = 33.1549;
q1 = 9.736;
k  = 11.4345;

% Relación estática servo-barra
a0 = deg2rad(-3.9);
a1 = 0.34;

% Muestreo
fs = 50;          % [Hz]
Ts = 1/fs;        % [s]

% Cantidad de muestras
Nmuestras = 10000;

t = (0:Nmuestras-1)'*Ts;


%%GENERACIÓN DE LA ENTRADA ALEATORIA
amplitud = 0.01;      % [rad]

rng(1);               % Semilla para repetibilidad

% Ruido binario aleatorio: +/- amplitud
u = amplitud*sign(randn(Nmuestras,1));

% ode45 necesita poder evaluar u(t) en cualquier instante.
% Se utiliza retención de orden cero.
u_continua = @(tt) interp1(t,u,tt,'previous','extrap');


%% MODELO NO LINEAL
% Estados:
%
% x(1): posición del carrito [m]
% x(2): velocidad [m/s]
% x(3): ángulo de la barra respecto del equilibrio [rad]
% x(4): velocidad angular [rad/s]
%
% Entrada:
% u = desviación del ángulo del servo respecto del equilibrio [rad]

modelo = @(tt,x) [
    x(2);
    g*sin(x(3)) - (b/m)*x(2);
    x(4);
    -q0*x(3) - q1*x(4) + k*u_continua(tt)
];

% Condición inicial: punto de equilibrio
x0 = [0; 0; 0; 0];

opts = odeset('MaxStep',Ts/5);

[t_sol,x_sol] = ode45(modelo,t,x0,opts);

% Salida: posición
y = x_sol(:,1);


%% SEÑALES TEMPORALES
figure;

subplot(2,1,1)

plot(t,u,'LineWidth',1.2)
grid on

xlabel('Tiempo [s]')
ylabel('\Delta u [rad]')
title('Entrada aleatoria')

ylim([-1.1*amplitud 1.1*amplitud])


subplot(2,1,2)

plot(t,y,'LineWidth',1.2)
grid on

xlabel('Tiempo [s]')
ylabel('y [m]')
title('Salida del sistema no lineal')


%% PREPROCESAMIENTO PARA IDENTIFICACIÓN
% Se elimina solamente el valor medio.
% No quitamos tendencia lineal porque el integrador pertenece
% a la dinámica real del sistema.

u_id = detrend(u,0);
y_id = detrend(y,0);


%% MODELO LINEALIZADO TEÓRICO

% Para ángulos pequeños:
% sin(theta) ~= theta
% Entonces:
%           g*k
% G(s) = -------------------------------
%        s(s+b/m)(s^2+q1*s+q0)

s = tf('s');

G_lin = (g*k) / ...
    (s*(s + b/m)*(s^2 + q1*s + q0));


%% IDENTIFICACIÓN MEDIANTE CORRELACIÓN

lags = 200;

% CRA estima la respuesta impulsional a partir de las
% correlaciones entrada-salida.

[h_cor,R,~] = cra([y_id,u_id],lags,20,0);


% Gráficos de las correlaciones

figure;
cra(R);

sgtitle('Funciones de correlación');


% Respuesta al impulso identificada

t_h = (0:length(h_cor)-1)'*Ts;

figure;

plot(t_h,h_cor,'LineWidth',1.5)

grid on

xlabel('Tiempo [s]')
ylabel('h[k]')
title('Respuesta al impulso estimada por correlación')


%% RESPUESTA EN FRECUENCIA A PARTIR DE h_cor

Nfft_cor = 2048;

H_cor_fft = fft(h_cor,Nfft_cor);

% Nos quedamos solamente con frecuencias positivas
Npos = floor(Nfft_cor/2) + 1;

H_cor_fft = H_cor_fft(1:Npos);

% Frecuencia en rad/muestra
Omega_cor = (0:Npos-1)' * 2*pi/Nfft_cor;

% Conversión a rad/s
w_cor = Omega_cor/Ts;

% Eliminamos continua porque G(s) contiene un integrador
H_cor_fft = H_cor_fft(2:end);
w_cor = w_cor(2:end);

H_cor = frd(H_cor_fft,w_cor);


%% ETFE = COCIENTE DE DFT

% IMPORTANTE:
% El tiempo de muestreo correcto es Ts = 0.02 s

datos = iddata(y_id,u_id,Ts);

H_etfe = etfe(datos);


%% ESTIMACIÓN MEDIANTE WELCH

Nfft_welch = 2048;

Lventana = 512;

ventana = hann(Lventana);

solapamiento = floor(Lventana/2);


% PSD de la entrada
[Puu,f_welch] = pwelch( ...
    u_id, ...
    ventana, ...
    solapamiento, ...
    Nfft_welch, ...
    fs);


% Densidad espectral cruzada
%
% Entrada primero, salida después.
[Puy,~] = cpsd( ...
    u_id, ...
    y_id, ...
    ventana, ...
    solapamiento, ...
    Nfft_welch, ...
    fs);


% Estimador H1
H_welch_val = Puy ./ Puu;


% MATLAB devuelve f en Hz.
% Convertimos a rad/s.
w_welch = 2*pi*f_welch;


% Quitamos frecuencia cero por el integrador
H_welch_val = H_welch_val(2:end);
w_welch = w_welch(2:end);


H_welch = frd(H_welch_val,w_welch);


%% COMPARACIÓN DE LOS TRES MÉTODOS CON EL MODELO

%figure;

%bode( ...
 %   G_lin, ...
  %  H_etfe, ...
   % H_welch, ...
    %H_cor);

%grid on

%legend( ...
    %'Modelo linealizado', ...
    %'ETFE', ...
   % 'Welch', ...
  %  'Correlación', ...
 %   'Location','best');

%title('Identificación no paramétrica');

%h = findobj(gcf,'type','line');
%set(h,'linewidth',1.5);


% Banda de frecuencias común
w_min = 0.1;
w_max = 100;

w = logspace(log10(w_min), log10(w_max), 1000)';

%Modelo linealizado
H_modelo = squeeze(freqresp(G_lin,w));

% ETFE
H_etfe_w = squeeze(freqresp(H_etfe,w));

%Welch
H_welch_w = squeeze(freqresp(H_welch,w));

%Correlación
H_cor_w = squeeze(freqresp(H_cor,w));

%MAGNITUD
figure;

subplot(2,1,1)

semilogx(w,20*log10(abs(H_modelo)), ...
    'LineWidth',2);

hold on;

semilogx(w,20*log10(abs(H_etfe_w)), ...
    'LineWidth',1.2);

semilogx(w,20*log10(abs(H_welch_w)), ...
    'LineWidth',1.5);

semilogx(w,20*log10(abs(H_cor_w)), ...
    'LineWidth',1.2);

grid on;

ylabel('Magnitud [dB]');

title('Identificación no paramétrica');

legend( ...
    'Modelo linealizado', ...
    'ETFE', ...
    'Welch', ...
    'Correlación', ...
    'Location','best');


%FASE
subplot(2,1,2)

fase_modelo = unwrap(angle(H_modelo))*180/pi;
fase_etfe   = unwrap(angle(H_etfe_w))*180/pi;
fase_welch  = unwrap(angle(H_welch_w))*180/pi;
fase_cor    = unwrap(angle(H_cor_w))*180/pi;

semilogx(w,fase_modelo,'LineWidth',2);

hold on;

semilogx(w,fase_etfe,'LineWidth',1.2);

semilogx(w,fase_welch,'LineWidth',1.5);

semilogx(w,fase_cor,'LineWidth',1.2);

grid on;

xlabel('\omega [rad/s]');
ylabel('Fase [°]');

legend( ...
    'Modelo linealizado', ...
    'ETFE', ...
    'Welch', ...
    'Correlación', ...
    'Location','best');

%% COHERENCIA

[Cuy,f_coh] = mscohere( ...
    u_id, ...
    y_id, ...
    ventana, ...
    solapamiento, ...
    Nfft_welch, ...
    fs);

w_coh = 2*pi*f_coh;

% Eliminamos continua
Cuy = Cuy(2:end);
w_coh = w_coh(2:end);


figure;

semilogx(w_coh,Cuy,'LineWidth',1.5)

grid on

xlabel('\omega [rad/s]')
ylabel('\gamma^2_{uy}(\omega)')

title('Coherencia entrada-salida')

ylim([0 1]);


%% PSD DE ENTRADA Y SALIDA

[Pyy,f_psd] = pwelch( ...
    y_id, ...
    ventana, ...
    solapamiento, ...
    Nfft_welch, ...
    fs);


figure;

subplot(2,1,1)

semilogy(2*pi*f_welch,Puu,'LineWidth',1.5)

grid on

xlabel('\omega [rad/s]')
ylabel('\Phi_{uu}')

title('PSD de la entrada')


subplot(2,1,2)

semilogy(2*pi*f_psd,Pyy,'LineWidth',1.5)

grid on

xlabel('\omega [rad/s]')
ylabel('\Phi_{yy}')

title('PSD de la salida')