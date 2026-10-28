clear; close all; clc;
load('datos_prueba.mat');
u = deg2rad(u - mean(u));
y = deg2rad(y - mean(y));
fs = 50;
Ts = 1/fs;

figure('Position',[50 -50 800 600]);

subplot(2, 1, 1);
plot(t, u, 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('u [rad]');
title('Entrada');
xlim([0, max(t)/4]);

subplot(2, 1, 2),
plot(t, y, 'LineWidth', 2);
grid on;
xlabel('Tiempo [s]');
ylabel('y [rad]');
title('Salida');
xlim([0, max(t)/4]);

exportgraphics(gcf, 'graficos id con medicion/entrada y salida.pdf', 'ContentType', 'image', 'Resolution', 300);
%% IDENTIFICACIÓN PREVIA
s = tf('s');
k = 11.4345;
q0 = 33.1549;
q1 = 9.736; 
G = k /(s^2 + q1 * s + q0); % --> Transferencia a identificar.
%NOTA: Tomar esta transferencia como referencia, no como objetivo. 
%El resultado puede ser distinto a esta, pero no demasiado
Gd = c2d(G, Ts, 'zoh'); 
Gd.Variable = 'z^-1';
present(Gd);
%% PASO 1: Identificación no paramétrica por Welch

NFFT = 512; %La estimación depende fuertemente de este valor. 
ventana = hann(NFFT);
noverlap = NFFT/2;

[Pyu,f] = cpsd(y,u,ventana,noverlap,NFFT,fs, 'twosided');
[Puu,~] = cpsd(u,u,ventana,noverlap,NFFT,fs, 'twosided');
w = 2 * pi * f;

G_welch = Pyu./Puu;

idx = w>0;

w = w(idx);
G_welch = G_welch(idx);

G_welch = frd(G_welch, w);
G_welch_bode = squeeze(G_welch.ResponseData);

%% CÓMPUTO DEL BODE TEÓRICO
G_teorica = squeeze(freqresp(Gd,w));

%% COMPARACIÓN EN BODES
W_MIN = w(1);
W_MAX = 50;
figure;

subplot(2,1,1);
semilogx(w, 20*log10(abs(G_welch_bode)), 'm', 'LineWidth', 1.5); hold on;
semilogx(w, 20*log10(abs(G_teorica)), 'b--', 'LineWidth', 1.5);

grid on;
ylabel('Magnitud [dB]');
legend('Welch',  'Estimacion original', 'Location', 'SouthWest');
title('Estimación por periodograma');
xlabel('\omega [rad/s]');
xlim([W_MIN, W_MAX]);

subplot(2,1,2);
semilogx(w, rad2deg(unwrap(angle(G_welch_bode))), 'm', 'LineWidth', 1.5); hold on;
semilogx(w, rad2deg(unwrap(angle(G_teorica))), 'b--', 'LineWidth', 1.5);

ylim([-400, 40]);
grid on;
xlabel('\omega [rad/s]');
ylabel('Fase [°]');
xlim([W_MIN, W_MAX]);

fig1 = gcf;
exportgraphics(fig1, 'graficos id con medicion/bode periodograma.pdf', 'ContentType', 'image', 'Resolution', 300);
%% ¿Está mal? Puede que no.
%{
Pareciera haber detectado un polo doble en 17.8 rad/s. 
Idea: esto puede tener que ver con el hecho de que la entrada es muy
amplia (+/- 20°, 40° de amplitud). Puede que se saturara la velocidad del
servo.

O simplemente la identificación original se hizo en otras condiciones.

¿Tiene sentido que la identificación sea sensible a las condiciones del
experimento?
%}

%% PASO 2: IDENTIFICACIÓN  POR ARX + REDUCCÓN

%Dividimos en conjuntos de identificación y testeo
N = length(u);
Nid = floor(0.75*N);
z_id  = iddata(y(1:Nid),u(1:Nid),Ts);
z_val = iddata(y(Nid+1:end),u(Nid+1:end),Ts);

orden = [8, 8, 1];
m_arx = arx(z_id, orden);

%% COMPARACIÓN EN BODES
w_plot = logspace(-1,2,length(w));

H_orig = squeeze(freqresp(Gd,w_plot));
H_arx  = squeeze(freqresp(m_arx,w_plot));


H_welch = interp1(w,G_welch_bode,w_plot,'pchip');

figure;
subplot(2,1,1)

semilogx(w_plot,20*log10(abs(H_welch)), 'm','LineWidth',1.5);hold on;
semilogx(w_plot,20*log10(abs(H_orig)),'b--','LineWidth',2); hold on;
semilogx(w_plot,20*log10(abs(H_arx)),'r','LineWidth',1.5);
grid on
ylabel('Magnitud [dB]')
xlim([1e-1 1e2])
xlabel('\omega [rad/s]')

legend('Welch','Original','ARX', 'Location','SouthWest');

title('Comparación de respuestas en frecuencia');

subplot(2,1,2)

semilogx(w_plot, unwrap(angle(H_welch))*180/pi,'m','LineWidth',1.5); hold on;
semilogx(w_plot,unwrap(angle(H_orig))*180/pi,'b--','LineWidth',2); hold on;
semilogx(w_plot, unwrap(angle(H_arx))*180/pi,'r','LineWidth',1.5);

grid on
xlabel('\omega [rad/s]')
ylabel('Fase [°]')
xlim([1e-1 1e2])

% OBS: Welch y arx identifican lo mismo

fig2 = gcf;
exportgraphics(fig2, 'graficos id con medicion/Bode arx.pdf', 'ContentType', 'image', 'Resolution', 300);
%% Reducción
[Ad, Bd, Cd, Dd] = ssdata(m_arx); %devuelve las matrices de la representación en ee del sistema
[Ab, Bb, Cb, M, T] = dbalreal(Ad,Bd,Cd);
disp("Valores singulares de Hankel: ")
M'
%Debería quedarme solo con los primeros 2 estados
[Ar, Br, Cr, Dr] = dmodred(Ab,Bb,Cb,Dd,3:length(M)); %elimina los estados indicados en el último vector
[b,a] = ss2tf(Ar, Br, Cr, 0); %genera los coeficientes de los polinomios
m_arx_red = idpoly(a,b,1,1,[],[],Ts);
present(Gd);
present(m_arx_red);

%% Comparación con la primer identificación
H_arx_red = squeeze(freqresp(m_arx_red, w_plot));

figure;
subplot(2, 1, 1);
semilogx(w_plot, 20 * log10(abs(H_arx)), 'r', 'LineWidth', 1.5); hold on;
semilogx(w_plot, 20 * log10(abs(H_arx_red)), 'm', 'LineWidth', 1.5); hold on;
semilogx(w_plot, 20 * log10(abs(H_orig)), 'b--', 'LineWidth', 1.5);
grid on;
legend('Sin reducir', 'Reducido', 'Original', 'Location', 'SouthWest');
xlabel('\omega [rad/s]');
ylabel('Magnitud [dB]');
title('Bode original, ARX y ARX reducido');

subplot(2, 1, 2);
semilogx(w_plot, rad2deg(unwrap(angle(H_arx))), 'r', 'LineWidth', 1.5); hold on;
semilogx(w_plot, rad2deg(unwrap(angle(H_arx_red))), 'm', 'LineWidth', 1.5); hold on;
semilogx(w_plot, rad2deg(unwrap(angle(H_orig))), 'b--', 'LineWidth', 1.5);
grid on;
xlabel('\omega [rad/s]');
ylabel('Fase [°]');
ylim([-360, 30]);

fig3 = gcf;
exportgraphics(fig3, 'graficos id con medicion/bode arx reducido.pdf', 'ContentType', 'image', 'Resolution', 300);
%% COMPARACIÓN EN EL CONJUNTO DE VALIDACIÓN

figure('Position',[50 -50 800 600]);

subplot(2, 1, 1);
compare(z_val, Gd);
grid on;
set(findall(gca,'Type','line'),'LineWidth',2);
xlabel('Tiempo [s]');
ylabel('y [rad]');
title('Salida según identificación original');

subplot(2, 1, 2);
compare(z_val, m_arx_red);
grid on;
set(findall(gca,'Type','line'),'LineWidth',2);
xlabel('Tiempo [s]');
ylabel('y [rad]');
title('Salida según identificación por ARX + reducción');

fig4 = gcf;
exportgraphics(fig4, 'graficos id con medicion/Comparacion orig. vs reduciodo.pdf', 'ContentType', 'image', 'Resolution', 300);
% Se observa que para el conjunto de datos recolectado, el sistema original es muy lento

%%
figure;
resid(z_val, m_arx_red);
grid on;
title('Residuos');
ylim([-0.5, 1.25]);
ylabel('Amplitud');

fig5 = gcf;
exportgraphics(fig5, 'graficos id con medicion/Residuos.pdf', 'ContentType', 'image', 'Resolution', 300);