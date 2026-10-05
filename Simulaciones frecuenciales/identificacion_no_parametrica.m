%{

Periodograma: Hacer las DFT´s de las señales de entrada y salida.
Estimar el filtro como su cociente

Correlograma:
Hacer las autocorr. de las señales de entrada y salida.
Hacer la DFT de las autocorr, es decir, las PSD
Estimar el filtro como el cociente de las PSD

Replicar el código '(codigo_de_pablo)no_parametrica.m'

La entrada tiene que ser aleatoria

3 métodos:

1) Correlograma + DFT
2) Cociente de DFT´s (función etfe)
3) Cociente de PSD´s (funciones pwelch y cpsd)

%}

clear; clc; close all;

% 1. PARAMETROS DEL SISTEMA A IDENTIFICAR
m  = 0.033;       % Masa del carrito [kg]
b  = 0.15;        % Rozamiento efectivo [kg/s]
g  = 9.8;         % Gravedad [m/s^2]
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

amplitud = 0.01; %Amplitud del ruido
rng(1); %Semilla para replicabilidad
u = amplitud*sign(randn(N,1)); %Señal de entrada: signo de una gaussiana

u_continua = @(tt) interp1(t,u,tt,'previous','extrap'); 
%esta función extrapola u_k a un valor de tiempo tt arbitrario
%De esta forma se crea una función 'continua' u(t), la cual será evaluada
%por ode45 para resolver la ec. diferencial. 




% 2. DEFINICIÓN DEL SISTEMA
%este modelo será resuelto por ode45

u_eq = -a0/a1; %valor de equilibrio
modelo = @(tt,x) [
    x(2);
    g*sin(x(3)) - (b/m)*x(2);
    x(4);
    -q0*x(3) - q1*x(4) + k*u_continua(tt)
];
%IMPORTANTE:
%entrada: DESVÍO RESPECTO DEL PUNTO DE EQUILIBRIO del ángulo del servo EN RADIANES. 
%salida: posición, x(1) EN METROS

x0 = [0;0;0;0]; %punto de equilibrio.

[t_sol,x_sol] = ode45(modelo,t,x0);

y = x_sol(:,1); %Salida simulada. Unidad: METROS


figure;

subplot(2,1,1)
plot(t,u,'LineWidth',1.2)
grid on
xlabel('Tiempo [s]')
ylabel('\Delta u [rad]')
ylim([-1.1 * amplitud, 1.1 * amplitud]);
title('Entrada aleatoria')

subplot(2,1,2)
plot(t,y,'LineWidth',1.2)
grid on
xlabel('Tiempo [s]')
ylabel('y [m]')
title('Salida del sistema')

%{
Hasta este punto el código generó los vectores u e y. A partir de este
comentario empieza la identificación por 3 métodos distintos.
Se puede usar el código de Pablo, ya que los vectores se llaman igual.
%}

%% IDENTIFICACIÓN DE LA RESPUESTA AL IMPULSO POR CORRELACIÓN

%Primero simulo la respuesta al impulso para comparar con la identificada.

u_imp = zeros(N,1);
u_imp(1) = 1;

u_imp_continuo = @(tt) interp1(t,u_imp,tt,'previous','extrap'); 
modelo_impulso = @(tt,x) [
    x(2);
    g*sin(x(3)) - (b/m)*x(2);
    x(4);
    -q0*x(3) - q1*x(4) + k*u_imp_continuo(tt)
];
[t_sol_impulso,x_sol_impulso] = ode45(modelo_impulso,t,x0);
h_real = x_sol_impulso(:, 1); %respuesta al impulso según el modelo definido en modelo_impulso


%Estimación de la respuesta al impulso
lags = 200; %lags a considerar
[h_cor, R, ~] = cra([y, u], lags, 20, 0);
% R --> Matriz con autocovarianzas y covarianzas cruzadas
% R(1, :) --> indices de lag || R(2, :) --> R_yy || R(3, :) --> R_uu || R(4, :) -->
% R_yu ||

% Comparación con la respuesta al impulso del modelo
figure;
plot([h_real(1:lags) h_cor(1:lags)],'LineWidth',2);
grid;
legend('Respuesta al impulso','Estimación por correlación', 'Location', 'southeast')

figure;
cra(R);
%¿Por qué arranca desde más abajo? --> Preguntarle a Pablo. ¿Está mal?

%% Identificación no paramétrica en frecuencia

%ETFE: cociente de DFT´s
z = iddata(y,u,Ts);
H_etfe = etfe(z);

% Welch. Cociente de PSD´s
Pyu = cpsd(y,u);
Puu = pwelch(u);
H_welch = Pyu./Puu;
w_welch = (0:length(H_welch)-1)'*pi/length(H_welch);
H_welch  = frd(H_welch ,w_welch);

% Respuesta en frecuencia del correlograma
N = 256;
H_cor = fft(h_cor,N);
w_cor = (0:N-1)'*2*pi/N;
H_cor = frd(H_cor,w_cor);

figure
bode(w_cor,H_etfe,H_welch,H_cor)
grid on;

ylim([-360, 360]);

legend('ETFE','Welch','Correlograma','Modelo')
title('Espectros')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);
