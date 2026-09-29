%% Identificación por respuesta al impulso
close all
clear all

% Modelo de referencia
n = 10000;
A = [1 -1.5 0.7];
B = [0 1 0.5];
C = 1;
D = 1;
F = 1;

%th = idpoly(A, B, C);
Ts = 1;
th = poly2th(A,B,C,D,F,1,Ts);

% Ruido de medición
e = 0.1*randn(n,1);

% Respuesta al impulso
m = 30;
u = zeros(n,1);
u(1) = 1;

y_imp = idsim(u,th);          % Sistema sin ruido
y_imp_ruido = idsim([u e],th);

% Respuesta al escalón
u = ones(n,1);
y_esc = idsim([u e],th);

% Diferencia de la respuesta al escalón
% (aproxima la respuesta al impulso)
y_imp_esc = [0; y_esc(2:m+1) - y_esc(1:m)];

% Comparación
plot([y_imp(1:m) y_imp_ruido(1:m) y_imp_esc(1:m)],'LineWidth',2)
grid
legend('Impulso','Impulso + ruido','Diferencia del escalón')


%% Identificación por correlación

% Entrada binaria aleatoria
u = sign(randn(n,1));

% Simulación del sistema
y = idsim([u e],th);

% Correlación entrada-salida
R = covf([y u],m+1);

% Estimación de la respuesta al impulso
h = R(2,:)'/R(4,1);

% Comparación con la respuesta al impulso del modelo
figure
plot([y_imp(1:m) h(1:m)],'LineWidth',2)
grid
legend('Respuesta al impulso','Estimación por correlación')


%% Identificación no paramétrica en frecuencia

u = sign(randn(n,1));
y = idsim([u e],th);

% ETFE
z = iddata(y,u,1);
H_etfe = etfe(z);

% Welch
Pyu = cpsd(y,u);
Puu = pwelch(u);
H_welch = Pyu./Puu;
w_welch = (0:length(H_welch)-1)'*pi/length(H_welch);
H_welch  = frd(H_welch ,w_welch);

% Correlograma
R = covf([y u],m+1);
h_cor = R(2,:)'/R(4,1);

% Respuesta en frecuencia del correlograma
N = 256;
H_cor = fft(h_cor,N);
w_cor = (0:N-1)'*2*pi/N;
H_cor = frd(H_cor,w_cor);

% Comparaciónw_cor
figure
bode(w_cor,H_etfe,H_welch,H_cor,th)
grid
legend('ETFE','Welch','Correlograma','Modelo')
title('Espectros')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);

%% Ajuste por mínimos cuadrados de una transferencia a partir del correlograma
% la magia está en fijar la cantidad de polos (2) y ceros (0) del sistema
% de tiempo continuo
H_id = tfest(H_cor,2,0)
bode(H_id,H_cor,w_cor,th)
legend('TF identificada','Correlograma','Modelo')
h = findobj(gcf,'type','line');
set(h,'linewidth',2);

% Polos identificados en Z
exp(pole(H_id))
% Vs polos del modelo
pole(th)

% Ganancia de continua identificada
dcgain(H_id)
% Vs polos del modelo
dcgain(th)

%% Efecto de aumentar la cantidad de muestras

N = [500 2000 50000];

figure
for k = 1:length(N)

    u = sign(randn(N(k),1));
    eN = 0.1*randn(N(k),1);
    y = idsim([u eN],th);

    % Periodograma
    U = fft(u,256);
    Y = fft(y,256);
    
    H_per = Y./U;

    % Correlograma + FFT
    R = covf([y u],m+1);
    h = R(2,:)'/R(4,1);
    H_cor = fft(h,256);

    subplot(3,1,k)
    w = (0:255)'*2*pi/256;
    semilogx(w,20*log10(abs(H_per(1:256))), ...
         w,20*log10(abs(H_cor)),'LineWidth',2)
    xlim([0.01,pi])
    grid
    title(['N = ' num2str(N(k))])
    legend('Periodograma','Correlograma + FFT','Location','SouthWest')
end

