%% IDENTIFICACION PARAMETRICA - BALL & BEAM
% Modelo linealizado aumentado de cuarto orden.
% LA: entrada u -> velocidad numerica (sin integrador).
% LC: referencia r -> posicion y (transferencia de lazo cerrado).
% Requiere System Identification Toolbox y Control System Toolbox.
clc; close all; clearvars;

%% 1) Modelo y parametros
m = 0.033; b = 0.15; g = 9.8;
q0 = 33.1549; q1 = 9.736; ks = 11.4345;
fs = 50; Ts = 1/fs;
N = 10000;
t = (0:N-1)'*Ts;
A = [0 1 0 0;
     0 -b/m g 0;
     0 0 0 1;
     0 0 -q0 -q1];
B = [0; 0; 0; ks];
C = [1 0 0 0]; D = 0;
G_c = ss(A,B,C,D);
G_d = c2d(G_c,Ts,'zoh');

%% 2) Entrada aleatoria binaria reproducible
rng(1);
amp = 0.01;
r = amp*sign(randn(N,1));
u_LA = r;

%% 3) LA: posicion, derivada numerica y transferencia exacta equivalente
x_LA = lsim(G_d,u_LA,t);
% Diferencia hacia atras: v[k]=(x[k]-x[k-1])/Ts, k=2,...,N.
v_LA = diff(x_LA)/Ts;
u_LA_v = u_LA(2:end);
% Modelo teorico de la misma diferencia, NO velocidad fisica exacta.
z = tf('z',Ts);
G_v_d = minreal((1-z^(-1))/Ts * tf(G_d),1e-7);
% Para mantener la referencia temporal, las muestras son k=2,...,N.

%% 4) Division de datos: 60%% identificacion, 40%% validacion
N_LA = length(v_LA);
Nid_LA = floor(0.6*N_LA);
z_id_LA = iddata(v_LA(1:Nid_LA),u_LA_v(1:Nid_LA),Ts);
z_val_LA = iddata(v_LA(Nid_LA+1:end),u_LA_v(Nid_LA+1:end),Ts);

%% 5) Identificacion no parametrica para referencia
G_spa_LA = spa(z_id_LA);
G_etfe_LA = etfe(z_id_LA);

figure('Name','LA - Identificacion espectral');
bode(G_v_d,G_spa_LA,G_etfe_LA);
legend('Teorica (diferencia)','SPA','ETFE','Location','best');
grid on; title('Lazo abierto: u -> velocidad estimada');

%% 6) Modelo parametrico LA: tercer orden (integrador eliminado)
% iv4 usa [na nb nk] para el modelo de polinomios tipo ARX.
% El orden 3 es una hipotesis inicial que se valida con los datos.
orden_LA = [3 3 1];
G_iv4_LA = iv4(z_id_LA,orden_LA);
disp('Modelo IV4 - LA (u -> velocidad):');
present(G_iv4_LA);

%% COMPARACION PARAMETRICA Y NO PARAMETRICA - LA

figure('Name','LA - Parametrico vs no parametrico');

bode(G_v_d, G_spa_LA, G_etfe_LA, G_iv4_LA);
grid on;

legend('Modelo teorico', 'SPA', 'ETFE', 'IV4', ...
       'Location', 'best');

title('Lazo abierto: comparacion frecuencial');

figure('Name','LA - Validacion');
compare(z_val_LA,G_iv4_LA,G_v_d);
grid on; title('LA: validacion sobre 40% no usado en identificacion');

figure('Name','LA - Polos y ceros');
pzmap(G_iv4_LA,G_v_d); grid on;
legend('IV4','Teorica','Location','best');

figure('Name','LA - Residuos');
resid(z_val_LA,G_iv4_LA);

%Duda: ¿Por que la correlación cruzada en este gráfico no cae a cero? ¿El
%modelo no es lo suficientemente complejo?

%% 7) LC: controlador proporcional y simulacion consistente en discreto
Kp = 2;
T_ry_d = feedback(Kp*G_d,1);  % referencia -> posicion
T_ru_d = feedback(Kp,G_d);    % referencia -> accion de control
if ~isstable(T_ry_d)
    error('El lazo cerrado no es estable con Kp = %.3g.',Kp);
end
y_LC = lsim(T_ry_d,r,t);
u_LC = lsim(T_ru_d,r,t);

figure('Name','LC - Senales');
subplot(3,1,1); plot(t,r); grid on; ylabel('r'); title('Lazo cerrado');
subplot(3,1,2); plot(t,u_LC); grid on; ylabel('u');
subplot(3,1,3); plot(t,y_LC); grid on; ylabel('y'); xlabel('Tiempo [s]');

%% 8) Identificacion LC: referencia -> posicion (modelo de orden 4)
Nid_LC = floor(0.6*N);
z_id_LC = iddata(y_LC(1:Nid_LC),r(1:Nid_LC),Ts);
z_val_LC = iddata(y_LC(Nid_LC+1:end),r(Nid_LC+1:end),Ts);
G_spa_LC = spa(z_id_LC);
G_etfe_LC = etfe(z_id_LC);
orden_LC = [4 4 1];
G_iv4_LC = iv4(z_id_LC,orden_LC);
disp('Modelo IV4 - LC (r -> y):');
present(G_iv4_LC);

figure('Name','LC - Espectral y parametrico');
bode(T_ry_d,G_spa_LC,G_etfe_LC,G_iv4_LC);
legend('T teorica','SPA','ETFE','IV4','Location','best');
grid on; title('LC: referencia -> posicion');

figure('Name','LC - Validacion');
compare(z_val_LC,G_iv4_LC,T_ry_d);
grid on; title('LC: validacion sobre 40% no usado en identificacion');

figure('Name','LC - Polos y ceros');
pzmap(G_iv4_LC,T_ry_d); grid on;
legend('IV4','T teorica','Location','best');

figure('Name','LC - Residuos');
OPT = residOptions('MaxLag', 100);
resid(z_val_LC,G_iv4_LC, OPT);

%Los residuos dan mal, hay correlación entre entrada y salida
%% 9) Comparacion opcional: planta u -> y en LC
% Esta estimacion directa puede sesgarse con ruido bajo realimentacion.
% En simulacion sin ruido se incluye solo como exploracion.
z_id_G_LC = iddata(y_LC(1:Nid_LC),u_LC(1:Nid_LC),Ts);
z_val_G_LC = iddata(y_LC(Nid_LC+1:end),u_LC(Nid_LC+1:end),Ts);
G_iv4_G_LC = iv4(z_id_G_LC,[4 4 1]);
figure('Name','LC - Planta u -> y (exploratorio)');
bode(G_d,G_iv4_G_LC);
legend('Planta teorica','IV4 con datos realimentados','Location','best');
grid on; title('Identificacion directa de planta en LC: interpretar con cautela');
figure('Name','LC - Validacion planta (exploratorio)');
compare(z_val_G_LC,G_iv4_G_LC,G_d);
grid on;

%% 10) Resumen
fprintf('\nMuestras LA: %d identificacion, %d validacion.\n',Nid_LA,N_LA-Nid_LA);
fprintf('Muestras LC: %d identificacion, %d validacion.\n',Nid_LC,N-Nid_LC);
fprintf('fs=%.1f Hz, Ts=%.3f s, duracion=%.1f s, amplitud=%.3f, Kp=%.2f\n', ...
        fs,Ts,N*Ts,amp,Kp);
