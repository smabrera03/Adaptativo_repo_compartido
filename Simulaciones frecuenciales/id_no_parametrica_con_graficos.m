clear; clc; close all;

%% IDENTIFICACION NO PARAMETRICA
% MODELO AUMENTADO DE CUARTO ORDEN
% Correlacion (tipo Pablo), ETFE y Welch

%% PARAMETROS
m = 33/1000;
b = 0.15;
g = 9.8;

fs = 50;
Ts = 1/fs;

%% MODELO LINEALIZADO DE CUARTO ORDEN

A = [0  1       0          0;
     0 -b/m     g          0;
     0  0       0          1;
     0  0     -33.1549   -9.736];

B = [0;
     0;
     0;
     11.4345];

C = [1 0 0 0];
D = 0;

sys_c = ss(A,B,C,D);

% Discretizacion
sys_or_d = c2d(sys_c,Ts,'zoh');

disp('Planta continua de cuarto orden:')
G = tf(sys_c)

disp('Planta discreta:')
Gd = tf(sys_or_d)

%% CONTROLADOR PROPORCIONAL
kp = 2;

%% EXCITACION ALEATORIA
n = 10000;
t = (0:n-1)'*Ts;

amplitud = 0.01;

rng(1);
u = amplitud*sign(randn(n,1));

%% IDENTIFICACION EN LAZO ABIERTO
u_LA = u;
y_LA = lsim(sys_or_d,u_LA,t);

% Eliminar medias
u_LA_id = detrend(u_LA,0);
y_LA_id = detrend(y_LA,0);

%% SEÑALES TEMPORALES Lazo Abierto

figure;

subplot(2,1,1)
plot(t,u_LA)
grid on
ylabel('u')
title('Lazo abierto - Entrada aleatoria')

subplot(2,1,2)
plot(t,y_LA)
grid on
xlabel('Tiempo [s]')
ylabel('y')
title('Lazo abierto - Salida')

%% CORRELACION TIPO PABLO - Lazo Abierto
m_lags = 1600;

Rcov_LA = covf([y_LA_id u_LA_id],m_lags+1);

h_LA = Rcov_LA(2,:)'/Rcov_LA(4,1);

%% CORRELACIONES PARA GRAFICAR - Lazo Abierto
[~,Rcra_LA,~] = cra([y_LA_id,u_LA_id],m_lags,20,0);

lags_LA = Rcra_LA(:,1);
tau_LA = lags_LA*Ts;

Ryy_LA = Rcra_LA(:,2);
Ruu_LA = Rcra_LA(:,3);
Ryu_LA = Rcra_LA(:,4);

Ryy_LA_n = Ryy_LA/max(abs(Ryy_LA));
Ruu_LA_n = Ruu_LA/max(abs(Ruu_LA));
Ryu_LA_n = Ryu_LA/max(abs(Ryu_LA));

figure;
plot(tau_LA,Ryy_LA_n,'LineWidth',1.5)
hold on
plot(tau_LA,Ruu_LA_n,'LineWidth',1.5)
plot(tau_LA,Ryu_LA_n,'LineWidth',1.5)
xline(0,'--')
grid on

xlabel('Retardo [s]')
ylabel('Correlacion normalizada')
legend('Ryy','Ruu','Ryu')
title('Correlaciones - Lazo abierto')

%% CORRELACION -> FRECUENCIA Lazo Abierto

Nfft = 2048;

Hcor_LA_fft = fft(h_LA,Nfft);

Npos = floor(Nfft/2)+1;

Hcor_LA_fft = Hcor_LA_fft(1:Npos);

Omega = (0:Npos-1)'*2*pi/Nfft;
w_cor = Omega/Ts;

% Eliminar continua
Hcor_LA_fft = Hcor_LA_fft(2:end);
w_cor = w_cor(2:end);

Hcor_LA = frd(Hcor_LA_fft,w_cor);

%% ETFE Lazo Abierto
datos_LA = iddata(y_LA_id,u_LA_id,Ts);
Hetfe_LA = etfe(datos_LA);

%% WELCH Lazo Abierto
Lventana = 512;
ventana = hann(Lventana);
solapamiento = floor(Lventana/2);

[Puu_LA,f] = pwelch(u_LA_id,ventana,solapamiento,Nfft,fs);

[Puy_LA,~] = cpsd(u_LA_id,y_LA_id,ventana,solapamiento,Nfft,fs);

Hwelch_LA_val = Puy_LA./Puu_LA;

w_welch_LA = 2*pi*f;

Hwelch_LA_val = Hwelch_LA_val(2:end);
w_welch_LA = w_welch_LA(2:end);

Hwelch_LA = frd(Hwelch_LA_val,w_welch_LA);

%% COHERENCIA Lazo Abierto

[C_LA,f_coh] = mscohere(u_LA_id,y_LA_id,ventana,solapamiento,Nfft,fs);

w_coh = 2*pi*f_coh;

%% SIMULACION EN LAZO CERRADO
% Transferencia r -> y
T_ry = feedback(kp*sys_or_d,1);

% Transferencia r -> u
T_ru = feedback(kp,sys_or_d);

% Misma excitacion que Lazo Abierto
r = u;

y_LC = lsim(T_ry,r,t);
u_LC = lsim(T_ru,r,t);

% Comprobacion
u_check = kp*(r-y_LC);

fprintf('Error maximo LC: %.3e\n',max(abs(u_LC-u_check)));

%% SEÑALES TEMPORALES Lazo Cerrado

figure;

subplot(3,1,1)
plot(t,r)
grid on
ylabel('r')
title('Referencia - Lazo cerrado')

subplot(3,1,2)
plot(t,u_LC)
grid on
ylabel('u')
title('Accion de control')

subplot(3,1,3)
plot(t,y_LC)
grid on
xlabel('Tiempo [s]')
ylabel('y')
title('Salida - Lazo cerrado')

%% IDENTIFICACION DE G CON DATOS EN Lazo Cerrado
u_LC_id = detrend(u_LC,0);
y_LC_id = detrend(y_LC,0);

%% CORRELACION TIPO PABLO - Lazo Cerrado

Rcov_LC = covf([y_LC_id u_LC_id],m_lags+1);

h_LC = Rcov_LC(2,:)'/Rcov_LC(4,1);

%% CORRELACIONES PARA GRAFICAR - Lazo Cerrado

[~,Rcra_LC,~] = cra([y_LC_id,u_LC_id],m_lags,20,0);

lags_LC = Rcra_LC(:,1);
tau_LC = lags_LC*Ts;

Ryy_LC = Rcra_LC(:,2);
Ruu_LC = Rcra_LC(:,3);
Ryu_LC = Rcra_LC(:,4);

Ryy_LC_n = Ryy_LC/max(abs(Ryy_LC));
Ruu_LC_n = Ruu_LC/max(abs(Ruu_LC));
Ryu_LC_n = Ryu_LC/max(abs(Ryu_LC));

figure;
plot(tau_LC,Ryy_LC_n,'LineWidth',1.5)
hold on
plot(tau_LC,Ruu_LC_n,'LineWidth',1.5)
plot(tau_LC,Ryu_LC_n,'LineWidth',1.5)
xline(0,'--')
grid on

xlabel('Retardo [s]')
ylabel('Correlacion normalizada')
legend('Ryy','Ruu','Ryu')
title('Correlaciones - Lazo cerrado')

%% CORRELACION -> FRECUENCIA Lazo Cerrado
Hcor_LC_fft = fft(h_LC,Nfft);

Hcor_LC_fft = Hcor_LC_fft(1:Npos);
Hcor_LC_fft = Hcor_LC_fft(2:end);

Hcor_LC = frd(Hcor_LC_fft,w_cor);

%% ETFE Lazo Cerrado
datos_LC = iddata(y_LC_id,u_LC_id,Ts);
Hetfe_LC = etfe(datos_LC);

%% WELCH Lazo Cerrado

[Puu_LC,f] = pwelch(u_LC_id,ventana,solapamiento,Nfft,fs);

[Puy_LC,~] = cpsd(u_LC_id,y_LC_id,ventana,solapamiento,Nfft,fs);

Hwelch_LC_val = Puy_LC./Puu_LC;

w_welch_LC = 2*pi*f;

Hwelch_LC_val = Hwelch_LC_val(2:end);
w_welch_LC = w_welch_LC(2:end);

Hwelch_LC = frd(Hwelch_LC_val,w_welch_LC);

%% COHERENCIA Lazo Cerrado

[C_LC,f_coh_LC] = mscohere(u_LC_id,y_LC_id,ventana,solapamiento,Nfft,fs);

w_coh_LC = 2*pi*f_coh_LC;

%% COMPARACION DE COHERENCIA

figure;

semilogx(w_coh(2:end),C_LA(2:end),'LineWidth',1.5)
hold on
semilogx(w_coh_LC(2:end),C_LC(2:end),'LineWidth',1.5)

grid on
xlabel('Frecuencia [rad/s]')
ylabel('Coherencia')
ylim([0 1.05])
xlim([0.1 100])

legend('Lazo abierto','Lazo cerrado')
title('Comparacion de coherencia')

%% IDENTIFICACION DE T = r -> y
r_id = detrend(r,0);

datos_T = iddata(y_LC_id,r_id,Ts);
Hetfe_T = etfe(datos_T);

%% WELCH r -> y

[Prr,f_T] = pwelch(r_id,ventana,solapamiento,Nfft,fs);

[Pry,~] = cpsd(r_id,y_LC_id,ventana,solapamiento,Nfft,fs);

Hwelch_T_val = Pry./Prr;

w_T = 2*pi*f_T;

Hwelch_T_val = Hwelch_T_val(2:end);
w_T = w_T(2:end);

Hwelch_T = frd(Hwelch_T_val,w_T);

%% GRAFICOS ADICIONALES
% Identificacion de T: referencia r -> salida y_LC
% Se conserva el modelo aumentado de cuarto orden.

lags_T = 500;
% Para comparar coeficientes de respuesta al impulso discretos,
% multiplicamos por Ts la salida de impulse() (expresada por unidad de tiempo).
[h_cra_T, Rcra_T, ~] = cra(datos_T,lags_T,0,0);
h_cra_T = h_cra_T(:);
h_real_T = Ts*impulse(T_ry,(0:lags_T-1)'*Ts);
h_real_T = h_real_T(:);
k_T = (0:min(numel(h_cra_T),numel(h_real_T))-1)';

figure('Name','Impulso T: CRA vs real');
plot(k_T,h_cra_T(1:numel(k_T)),'LineWidth',1.5); hold on;
plot(k_T,h_real_T(1:numel(k_T)),'LineWidth',1.5);
grid on; xlabel('Muestra k'); ylabel('Respuesta al impulso');
legend('CRA','T real','Location','best');
title('Lazo cerrado: respuesta al impulso r -> y');

figure('Name','CRA T: respuesta impulsional');
cra(datos_T,lags_T,0,1);
title('CRA: respuesta al impulso de T (r -> y)');

figure('Name','CRA T: covarianzas');
cra(datos_T,lags_T,0,2);
title('CRA: autocovarianzas y covarianzas cruzadas (r,y)');

% Estimacion explicita tipo Pablo mediante covf
lags_cov_T = 100;
Rcov_T = covf([y_LC_id r_id],lags_cov_T+1);
h_cov_T = Rcov_T(2,:)'/Rcov_T(4,1);
h_real_cov_T = Ts*impulse(T_ry,(0:lags_cov_T)'*Ts);
h_real_cov_T = h_real_cov_T(:);
k_cov_T = (0:lags_cov_T)';
figure('Name','Impulso T: covf vs real');
plot(k_cov_T,h_cov_T,'LineWidth',1.5); hold on;
plot(k_cov_T,h_real_cov_T,'LineWidth',1.5);
grid on; xlabel('Muestra k'); ylabel('Respuesta al impulso');
legend('covf','T real','Location','best');
title('Lazo cerrado: correlacion tipo Pablo vs respuesta real');

figure('Name','Bode T real vs ETFE');
bode(T_ry,Hetfe_T); grid on;
legend('T real','ETFE','Location','best');
title('Transferencia r -> y: modelo real vs ETFE');

%% GRAFICOS COMPARATIVOS
wmin = 0.1;
wmax = 100;

wplot = logspace(log10(wmin),log10(wmax),1000)';

%% G REAL VS IDENTIFICACION Lazo Abierto

G_w = squeeze(freqresp(sys_or_d,wplot));

ETFE_LA_w = squeeze(freqresp(Hetfe_LA,wplot));
Welch_LA_w = squeeze(freqresp(Hwelch_LA,wplot));
Cor_LA_w = squeeze(freqresp(Hcor_LA,wplot));

figure;

subplot(2,1,1)

semilogx(wplot,20*log10(abs(G_w)),'LineWidth',2)
hold on
semilogx(wplot,20*log10(abs(ETFE_LA_w)))
semilogx(wplot,20*log10(abs(Welch_LA_w)))
semilogx(wplot,20*log10(abs(Cor_LA_w)))

grid on
ylabel('Magnitud [dB]')
legend('G real','ETFE','Welch','Correlacion')
title('Planta de cuarto orden - Lazo abierto')

subplot(2,1,2)

semilogx(wplot,unwrap(angle(G_w))*180/pi,'LineWidth',2)
hold on
semilogx(wplot,unwrap(angle(ETFE_LA_w))*180/pi)
semilogx(wplot,unwrap(angle(Welch_LA_w))*180/pi)
semilogx(wplot,unwrap(angle(Cor_LA_w))*180/pi)

grid on
xlabel('Frecuencia [rad/s]')
ylabel('Fase [grados]')

%% G REAL VS IDENTIFICACION CON DATOS Lazo Cerrado

ETFE_LC_w = squeeze(freqresp(Hetfe_LC,wplot));
Welch_LC_w = squeeze(freqresp(Hwelch_LC,wplot));
Cor_LC_w = squeeze(freqresp(Hcor_LC,wplot));

figure;

subplot(2,1,1)

semilogx(wplot,20*log10(abs(G_w)),'LineWidth',2)
hold on
semilogx(wplot,20*log10(abs(ETFE_LC_w)))
semilogx(wplot,20*log10(abs(Welch_LC_w)))
semilogx(wplot,20*log10(abs(Cor_LC_w)))

grid on
ylabel('Magnitud [dB]')
legend('G real','ETFE','Welch','Correlacion')
title('Planta de cuarto orden - Datos LC')

subplot(2,1,2)

semilogx(wplot,unwrap(angle(G_w))*180/pi,'LineWidth',2)
hold on
semilogx(wplot,unwrap(angle(ETFE_LC_w))*180/pi)
semilogx(wplot,unwrap(angle(Welch_LC_w))*180/pi)
semilogx(wplot,unwrap(angle(Cor_LC_w))*180/pi)

grid on
xlabel('Frecuencia [rad/s]')
ylabel('Fase [grados]')

%% TRANSFERENCIA T = r -> y

T_w = squeeze(freqresp(T_ry,wplot));

ETFE_T_w = squeeze(freqresp(Hetfe_T,wplot));
Welch_T_w = squeeze(freqresp(Hwelch_T,wplot));

figure;

subplot(2,1,1)

semilogx(wplot,20*log10(abs(T_w)),'LineWidth',2)
hold on
semilogx(wplot,20*log10(abs(ETFE_T_w)))
semilogx(wplot,20*log10(abs(Welch_T_w)))

grid on
ylabel('Magnitud [dB]')
legend('T real','ETFE','Welch')
title('Transferencia r -> y - Cuarto orden')

subplot(2,1,2)

semilogx(wplot,unwrap(angle(T_w))*180/pi,'LineWidth',2)
hold on
semilogx(wplot,unwrap(angle(ETFE_T_w))*180/pi)
semilogx(wplot,unwrap(angle(Welch_T_w))*180/pi)

grid on
xlabel('Frecuencia [rad/s]')
ylabel('Fase [grados]')