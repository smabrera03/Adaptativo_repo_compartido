clear; clc; close all;
clear; clc; close all;

% IDENTIFICACION NO PARAMETRICA
% Planta con integrador + polo: a partir de mi código y el de pablo
%% PARAMETROS
m = 33/1000;
b = 0.15;
g = 9.8;
a1 = 0.34;

fs = 50;
Ts = 1/fs;

A = [0, 1;
     0, -b/m];

B = [0;
     a1*g];

C = [1, 0];
D = 0;

sys_c = ss(A,B,C,D);
sys_or_d = c2d(sys_c,Ts,'zoh');
%Definí planta lineal!
kp = 2;      % Ganancia del controlador definida por Santi
%% EXCITACION ALEATORIA
n = 10000;
t = (0:n-1)'*Ts;
amplitud = 0.01;
rng(1);
u = amplitud*sign(randn(n,1));

u_LA = u;
y_LA = lsim(sys_or_d,u_LA,t);
%% IDENTIFICACION EN LAZO ABIERTO
% Quitamos las medias

u_LA_id = detrend(u_LA,0);
y_LA_id = detrend(y_LA,0);

%% CORRELACION
m_lags = 1600;

R_LA = covf([y_LA_id u_LA_id],m_lags+1);

% Estimación de la respuesta al impulso
h_LA = R_LA(2,:)'/R_LA(4,1);

%% CORRELACION -> RESPUESTA EN FRECUENCIA

Nfft = 2048;

H_cor_fft = fft(h_LA,Nfft);

Npos = floor(Nfft/2)+1;
H_cor_fft = H_cor_fft(1:Npos);

Omega = (0:Npos-1)'*2*pi/Nfft;
w_cor = Omega/Ts;

% Quitamos continua
H_cor_fft = H_cor_fft(2:end);
w_cor = w_cor(2:end);

H_cor = frd(H_cor_fft,w_cor);

%% ETFE
datos_LA = iddata(y_LA_id,u_LA_id,Ts);
Hetfe_LA = etfe(datos_LA);

%% WELCH
Lventana = 512;
ventana = hann(Lventana);
solapamiento = floor(Lventana/2);

[Puu_LA,f] = pwelch(u_LA_id,ventana,solapamiento,Nfft,fs);

[Puy_LA,~] = cpsd(u_LA_id,y_LA_id, ...
                  ventana,solapamiento,Nfft,fs);

Hwelch_LA_val = Puy_LA./Puu_LA;

w_welch_LA = 2*pi*f;

% Quitamos continua
Hwelch_LA_val = Hwelch_LA_val(2:end);
w_welch_LA = w_welch_LA(2:end);

Hwelch_LA = frd(Hwelch_LA_val,w_welch_LA);

%% COHERENCIA
[C_LA,f_coh] = mscohere(u_LA_id,y_LA_id,ventana,solapamiento,Nfft,fs);
w_coh = 2*pi*f_coh;

%% LAZO CERRADO

T_ry = feedback(kp*sys_or_d,1);

% Transferencia referencia -> acción de control
T_ru = feedback(kp,sys_or_d);

r = amplitud*sign(randn(n,1));

y_LC = lsim(T_ry,r,t);
u_LC = lsim(T_ru,r,t);

% Verificación
u_check = kp*(r-y_LC);

fprintf('Error máximo: %.3e\n', max(abs(u_LC-u_check)));

%% SIMULACION EN LAZO CERRADO
% Utilizamos la misma realización aleatoria, pero ahora como referencia.
% Referencia -> salida
T_ry = feedback(kp*sys_or_d,1);

% Referencia -> acción de control
T_ru = feedback(kp,sys_or_d);

r = u;

y_LC = lsim(T_ry,r,t);
u_LC = lsim(T_ru,r,t);

%% Comprobación de u = Kp(r-y)

u_check = kp*(r-y_LC);

fprintf('Error maximo en u = Kp(r-y): %.3e\n', ...
        max(abs(u_LC-u_check)));

%% Señales temporales

figure;

subplot(3,1,1)
plot(t,r,'LineWidth',1)
grid on
ylabel('r')
title('Lazo cerrado - Referencia aleatoria')

subplot(3,1,2)
plot(t,u_LC,'LineWidth',1)
grid on
ylabel('u')
title('Acción de control')

subplot(3,1,3)
plot(t,y_LC,'LineWidth',1)
grid on
xlabel('Tiempo [s]')
ylabel('y')
title('Salida')

%% IDENTIFICACION DE G DESDE DATOS EN LAZO CERRADO
%           u_LC ---> ? ---> y_LC
% Ver si recuperamos G.

u_LC_id = detrend(u_LC,0);
y_LC_id = detrend(y_LC,0);

%% CORRELACION

[h_LC,R_LC,~] = cra([y_LC_id,u_LC_id],m_lags,20,0);

%% Correlaciones normalizadas

lags_LC = R_LC(:,1);
tau_LC = lags_LC*Ts;

Ryy_LC = R_LC(:,2);
Ruu_LC = R_LC(:,3);
Ryu_LC = R_LC(:,4);

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
xlabel('Retardo \tau [s]')
ylabel('Correlación normalizada')
legend('R_{yy}','R_{uu}','R_{yu}','Location','best');
title('Lazo cerrado - Correlaciones');

%% Correlación -> frecuencia
Hcor_LC_fft = fft(h_LC,Nfft);

Hcor_LC_fft = Hcor_LC_fft(1:Npos);

Hcor_LC_fft = Hcor_LC_fft(2:end);

Hcor_LC = frd(Hcor_LC_fft,w_cor);

%% ETFE

datos_LC = iddata(y_LC_id,u_LC_id,Ts);
Hetfe_LC = etfe(datos_LC);

%% WELCH
[Puu_LC,f] = pwelch(u_LC_id,ventana,solapamiento,Nfft,fs);

[Puy_LC,~] = cpsd(u_LC_id,y_LC_id,ventana,solapamiento,Nfft,fs);

Hwelch_LC_val = Puy_LC./Puu_LC;

w_welch_LC = 2*pi*f;

% Quitamos continua
Hwelch_LC_val = Hwelch_LC_val(2:end);
w_welch_LC = w_welch_LC(2:end);

Hwelch_LC = frd(Hwelch_LC_val,w_welch_LC);

%% COHERENCIA
[C_LC,f_coh_LC] = mscohere(u_LC_id,y_LC_id,ventana,solapamiento,Nfft,fs);
w_coh_LC = 2*pi*f_coh_LC;

%% COMPARACION DE COHERENCIA
figure;
semilogx(w_coh(2:end),C_LA(2:end),'LineWidth',1.5);
hold on
semilogx(w_coh_LC(2:end),C_LC(2:end),'LineWidth',1.5);
grid on
xlabel('\omega [rad/s]')
ylabel('\gamma^2_{uy}')
ylim([0 1.05])
xlim([0.1 100])
legend('Lazo abierto','Lazo cerrado','Location','best');
title('Coherencia entrada-salida');

%% IDENTIFICACION r -> y EN LAZO CERRADO
% Queremos recuperar:
%             Kp G
%       T = ----------
%            1+Kp G

r_id = detrend(r,0);
datos_T = iddata(y_LC_id,r_id,Ts);
Hetfe_T = etfe(datos_T);

[Prr,f_T] = pwelch(r_id,ventana,solapamiento,Nfft,fs);

[Pry,~] = cpsd(r_id,y_LC_id,ventana,solapamiento,Nfft,fs);

Hwelch_T_val = Pry./Prr;
w_T = 2*pi*f_T;

Hwelch_T_val = Hwelch_T_val(2:end);
w_T = w_T(2:end);

Hwelch_T = frd(Hwelch_T_val,w_T);

%% GRÁFICOS DE COMPARACIÓN
% Banda útil para visualizar
wmin = 0.1;
wmax = 100;

wplot = logspace(log10(wmin),log10(wmax),1000)';

%% Planta G identificada en LAZO ABIERTO
G_w = squeeze(freqresp(sys_or_d,wplot));
ETFE_LA_w = squeeze(freqresp(Hetfe_LA,wplot));
Welch_LA_w = squeeze(freqresp(Hwelch_LA,wplot));
Cor_LA_w = squeeze(freqresp(Hcor_LA,wplot));

figure;
subplot(2,1,1)
semilogx(wplot,20*log10(abs(G_w)),'LineWidth',2);
hold on
semilogx(wplot,20*log10(abs(ETFE_LA_w)),'LineWidth',1);
semilogx(wplot,20*log10(abs(Welch_LA_w)),'LineWidth',1.5);
semilogx(wplot,20*log10(abs(Cor_LA_w)),'LineWidth',1);
grid on
ylabel('Magnitud [dB]')
legend('G real','ETFE','Welch','Correlación','Location','best');
title('Identificación de G - Lazo abierto');

subplot(2,1,2)
semilogx(wplot,angle(G_w)*180/pi,'LineWidth',2);
hold on
semilogx(wplot,angle(ETFE_LA_w)*180/pi,'LineWidth',1);
semilogx(wplot,angle(Welch_LA_w)*180/pi,'LineWidth',1.5);
semilogx(wplot,angle(Cor_LA_w)*180/pi,'LineWidth',1);
grid on
xlabel('\omega [rad/s]')
ylabel('Fase [°]')

%% Planta G identificada con datos de LAZO CERRADO

ETFE_LC_w = squeeze(freqresp(Hetfe_LC,wplot));
Welch_LC_w = squeeze(freqresp(Hwelch_LC,wplot));
Cor_LC_w = squeeze(freqresp(Hcor_LC,wplot));


figure;
subplot(2,1,1)
semilogx(wplot,20*log10(abs(G_w)),'LineWidth',2);
hold on
semilogx(wplot,20*log10(abs(ETFE_LC_w)),'LineWidth',1);
semilogx(wplot,20*log10(abs(Welch_LC_w)),'LineWidth',1.5);
semilogx(wplot,20*log10(abs(Cor_LC_w)),'LineWidth',1);
grid on
ylabel('Magnitud [dB]')
legend('G real','ETFE','Welch','Correlación','Location','best');
title('Identificación de G con datos en lazo cerrado');


subplot(2,1,2)
semilogx(wplot,angle(G_w)*180/pi,'LineWidth',2);
hold on
semilogx(wplot,angle(ETFE_LC_w)*180/pi,'LineWidth',1);
semilogx(wplot,angle(Welch_LC_w)*180/pi,'LineWidth',1.5);
semilogx(wplot,angle(Cor_LC_w)*180/pi,'LineWidth',1);
grid on
xlabel('\omega [rad/s]')
ylabel('Fase [°]')

%% Transferencia r -> y

T_w = squeeze(freqresp(T_ry,wplot));

ETFE_T_w = squeeze(freqresp(Hetfe_T,wplot));
Welch_T_w = squeeze(freqresp(Hwelch_T,wplot));

figure;
subplot(2,1,1)
semilogx(wplot,20*log10(abs(T_w)),'LineWidth',2);
hold on
semilogx(wplot,20*log10(abs(ETFE_T_w)),'LineWidth',1);
semilogx(wplot,20*log10(abs(Welch_T_w)),'LineWidth',1.5);
grid on
ylabel('Magnitud [dB]')
legend('T real','ETFE','Welch','Location','best');
title('Identificación de la transferencia r \rightarrow y');

subplot(2,1,2)
semilogx(wplot,angle(T_w)*180/pi,'LineWidth',2);
hold on
semilogx(wplot,angle(ETFE_T_w)*180/pi,'LineWidth',1);
semilogx(wplot,angle(Welch_T_w)*180/pi,'LineWidth',1.5);
grid on
xlabel('\omega [rad/s]')
ylabel('Fase [°]')