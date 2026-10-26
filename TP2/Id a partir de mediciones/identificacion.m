clear; close all; clc;
load('datos_prueba.mat');
fprintf('Media de u: %.3f  °\n', mean(u));
fprintf('Media de tita: %.3f  °\n', mean(y));
figure;
subplot(2, 1, 1);
plot(t, u); grid on;
xlim([0, 10]);
subplot(2, 1, 2);
plot(t, y); grid on;
xlim([0, 10]);
u = deg2rad(u - mean(u));
y = deg2rad(y - mean(y));
fs = 50;
Ts = 1/fs;
%%
s = tf('s');
k = 11.4345;
q0 = 33.1549;
q1 = 9.736; 
G = k /(s^2 + q1 * s + q0); % --> Transferencia a identificar.
%NOTA: Tomar esta transferencia como referencia, no como objetivo. 
%El resultado puede ser distinto a esta, pero no demasiado
Gd = c2d(G, Ts, 'zoh'); 
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
title('Bode de G\_der');
xlim([W_MIN, W_MAX]);

subplot(2,1,2);
semilogx(w, rad2deg(unwrap(angle(G_welch_bode))), 'm', 'LineWidth', 1.5); hold on;
semilogx(w, rad2deg(unwrap(angle(G_teorica))), 'b--', 'LineWidth', 1.5);

ylim([-400, 40]);
grid on;
xlabel('\omega [rad/s]');
ylabel('Fase [°]');
xlim([W_MIN, W_MAX]);

%% REVISAR. ¿Por qué da mal?
%Pareciera haber detectado un polo doble en 17.8 rad/s. 
%Idea: esto puede tener que ver con el hecho de que la entrada es muy
%amplia (+/- 20°, 40° de amplitud). Puede que se saturara la velocidad del
%servo

figure;

subplot(2, 1, 1);
plot(t, y, 'LineWidth', 2);
grid on;
xlim([0, 7]);
title('¿Se saturó la velocidad?');

subplot(2, 1, 2);
plot(t(1:end-1), diff(y)/Ts, 'LineWidth', 2); 
grid on;
xlim([0, 7]);

%% IDENTIFICACIÓN  POR ARX + REDUCCÓN

%Dividimos en conjuntos de identificación y testeo

N = length(u);
Nid = floor(0.75*N);
z_id  = iddata(y(1:Nid),u(1:Nid),Ts);
z_val = iddata(y(Nid+1:end),u(Nid+1:end),Ts);

%%

NN = struc(1:10, 1:10, 1:5); %Matriz con los posibles modelos a testear

V = arxstruc(z_id, z_val, NN); %Calcula los scores de rendimiento para los modelos en NN

orden = selstruc(V); %Muestra los scores y permite elegir un modelo

m_arx = arx(z_id, orden); %Obs: esta identificación también agarra a los polos de 17.8 rad/s. 
%La muestra tomada no sirve. Hacer algo con menos amplitud. 

%%
figure;
pzmap(Gd, m_arx);
legend('Original', 'Reidentificado');

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

%% Comparación con el modelo sin reducción
figure;
bode(m_arx_red, m_arx); grid on;
legend('Reducido', 'Sin reducir');

figure;
pzmap(m_arx_red, m_arx);
legend('Reducido', 'Sin reducir');

%% Comparación con la primer identificación
figure;
bode(m_arx_red, Gd); grid on;
legend('Reducido', 'Original');

figure;
pzmap(m_arx_red, Gd);
legend('Reducido', 'Original');
