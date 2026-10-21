%% IDENTIFICACION PARAMETRICA - BALL & BEAM
% Modelo linealizado aumentado de cuarto orden.
%
% 1) LA: u -> velocidad estimada
% 2) LC: r -> posicion medida
% 3) LC: u -> posicion medida (identificacion de planta)
%
% Incluye:
% - Ruido de medicion
% - Realimentacion de la posicion medida
% - Validacion independiente con señal binaria mas lenta
%
% Requiere Control System Toolbox y System Identification Toolbox.

clc; close all; clearvars;

%% 1) MODELO Y PARAMETROS

m = 0.033;
b = 0.15;
g = 9.8;

q0 = 33.1549;
q1 = 9.736;
ks = 11.4345;

fs = 50;
Ts = 1/fs;

N = 10000;
t = (0:N-1)'*Ts;

amp = 0.01;
Kp = 2;

% Ruido de posicion [m].
% Valor inicial de prueba; NO corresponde a una medicion real.
sigma_n = 2e-4;

% Modelo continuo
A = [0 1 0 0;
     0 -b/m g 0;
     0 0 0 1;
     0 0 -q0 -q1];

B = [0; 0; 0; ks];
C = [1 0 0 0];
D = 0;

G_c = ss(A,B,C,D);

% Discretizacion ZOH
G_d = c2d(G_c,Ts,'zoh');

z = tf('z',Ts);

% Transferencia equivalente a la diferencia finita
G_v_d = minreal( ...
    (1-z^(-1))/Ts * tf(G_d),1e-7);

%% 2) SEÑALES DE IDENTIFICACION

rng(1);

% Excitacion binaria aleatoria
r_id = amp*sign(randn(N,1));

% Ruido independiente de medicion
rng(2);
n_LA = sigma_n*randn(N,1);

rng(3);
n_LC = sigma_n*randn(N,1);

%% 3) LAZO ABIERTO CON RUIDO

u_LA = r_id;

% Posicion fisica
x_LA = lsim(G_d,u_LA,t);

% Posicion medida
x_LA_med = x_LA + n_LA;

% Velocidad fisica del modelo
C_v = [0 1 0 0];
G_vel_d = ss(G_d.A,G_d.B,C_v,0,Ts);

v_LA_real = lsim(G_vel_d,u_LA,t);

% Velocidad estimada por diferencia finita
v_LA = diff(x_LA_med)/Ts;

% Entrada alineada temporalmente
u_LA_v = u_LA(2:end);
t_LA = t(2:end);

figure('Name','LA - Velocidad con ruido');

plot(t_LA,v_LA,'DisplayName','Velocidad estimada');
hold on;
plot(t_LA,v_LA_real(2:end),'DisplayName','Velocidad fisica');
grid on;

xlabel('Tiempo [s]');
ylabel('Velocidad [m/s]');
title('LA: efecto del ruido sobre la derivacion');
legend('Location','best');

%% 4) DATOS LA: IDENTIFICACION

N_LA = length(v_LA);
Nid_LA = floor(0.6*N_LA);

z_id_LA = iddata( ...
    v_LA(1:Nid_LA), ...
    u_LA_v(1:Nid_LA),Ts);

z_val_LA = iddata( ...
    v_LA(Nid_LA+1:end), ...
    u_LA_v(Nid_LA+1:end),Ts);

% Identificacion no parametrica
G_spa_LA = spa(z_id_LA);
G_etfe_LA = etfe(z_id_LA);

% Identificacion parametrica
orden_LA = [3 3 1];
G_iv4_LA = iv4(z_id_LA,orden_LA);

disp(' ');
disp('MODELO IV4 - LA');
present(G_iv4_LA);

figure('Name','LA - Comparacion frecuencial');
bode(G_v_d,G_spa_LA,G_etfe_LA,G_iv4_LA);
legend('Teorica','SPA','ETFE','IV4','Location','best');
grid on;

figure('Name','LA - Validacion 40%');
compare(z_val_LA,G_iv4_LA,G_v_d);
grid on;

figure('Name','LA - Polos y ceros');
pzmap(G_iv4_LA,G_v_d);
grid on;
legend('IV4','Teorica','Location','best');

figure('Name','LA - Residuos');
resid(z_val_LA,G_iv4_LA);

%% 5) LAZO CERRADO CON RUIDO

% Transferencias discretas
T_ry_d = feedback(Kp*G_d,1);
T_ru_d = feedback(Kp,G_d);

% Sensibilidad
S_d = feedback(1,Kp*G_d);

if ~isstable(T_ry_d)
    error('El lazo cerrado no es estable.');
end

% Posicion fisica
% Y = T*R - G*Kp*S*N
y_LC = lsim(T_ry_d,r_id,t) - lsim(G_d*Kp*S_d,n_LC,t);

% Posicion medida
y_LC_med = y_LC + n_LC;

% Accion de control aplicada a la planta
u_LC = Kp*(r_id-y_LC_med);

% Comprobacion de consistencia del lazo
y_LC_check = lsim(G_d,u_LC,t);

fprintf('\nError de consistencia LC: %.3e\n', max(abs(y_LC-y_LC_check)));

figure('Name','LC - Senales con ruido');

subplot(3,1,1);
plot(t,r_id);
grid on;
ylabel('r');
title('Lazo cerrado con ruido de medicion');

subplot(3,1,2);
plot(t,u_LC);
grid on;
ylabel('u');

subplot(3,1,3);
plot(t,y_LC_med);
grid on;
ylabel('y medida');
xlabel('Tiempo [s]');

%% 6) IDENTIFICACION LC: REFERENCIA -> POSICION

Nid_LC = floor(0.6*N);

z_id_LC = iddata(y_LC_med(1:Nid_LC),r_id(1:Nid_LC),Ts);

z_val_LC = iddata(y_LC_med(Nid_LC+1:end),r_id(Nid_LC+1:end),Ts);

G_spa_LC = spa(z_id_LC);
G_etfe_LC = etfe(z_id_LC);

orden_LC = [4 4 1];

G_iv4_LC = iv4(z_id_LC,orden_LC);

disp(' ');
disp('MODELO IV4 - LC: r -> y');
present(G_iv4_LC);

figure('Name','LC - Referencia a posicion');
bode(T_ry_d,G_spa_LC,G_etfe_LC,G_iv4_LC);
legend('T teorica','SPA','ETFE','IV4','Location','best');
grid on;

figure('Name','LC - Validacion r -> y');
compare(z_val_LC,G_iv4_LC,T_ry_d);
grid on;

figure('Name','LC - Polos y ceros');
pzmap(G_iv4_LC,T_ry_d);
grid on;
legend('IV4','T teorica','Location','best');

figure('Name','LC - Residuos r -> y');
resid(z_val_LC,G_iv4_LC);

%% 7) IDENTIFICACION DE LA PLANTA EN LAZO CERRADO
% Entrada: accion de control u_LC
% Salida: posicion medida y_LC_med
%
% IMPORTANTE:
% Bajo realimentacion, u_LC esta correlacionada con
% el ruido de medicion. La estimacion debe evaluarse
% con cautela.

z_id_G_LC = iddata(y_LC_med(1:Nid_LC),u_LC(1:Nid_LC),Ts);

z_val_G_LC = iddata(y_LC_med(Nid_LC+1:end),u_LC(Nid_LC+1:end),Ts);

orden_G_LC = [4 4 1];

G_iv4_G_LC = iv4(z_id_G_LC,orden_G_LC);

disp(' ');
disp('MODELO IV4 - PLANTA DESDE LC: u -> y');
present(G_iv4_G_LC);

figure('Name','LC - Planta identificada u -> y');

bode(G_d,G_iv4_G_LC);
legend('Planta teorica','IV4 con datos LC', ...
    'Location','best');
grid on;

figure('Name','LC - Validacion planta u -> y');

compare(z_val_G_LC,G_iv4_G_LC);
grid on;

figure('Name','LC - Polos y ceros planta');
pzmap(G_d,G_iv4_G_LC);
grid on;
legend('Planta teorica','IV4','Location','best');

figure('Name','LC - Residuos planta');
resid(z_val_G_LC,G_iv4_G_LC);

%% 8) NUEVA SEÑAL INDEPENDIENTE DE VALIDACION

Nval = 4000;
tval = (0:Nval-1)'*Ts;

% Tiempo de permanencia: 10 muestras = 0.2 segundos
muestras_por_nivel = 10;

rng(10);

niveles = amp*sign(randn(ceil(Nval/muestras_por_nivel),1));

r_val = repelem(niveles,muestras_por_nivel);
r_val = r_val(1:Nval);

% Nuevas realizaciones de ruido
rng(11);
n_LA_val = sigma_n*randn(Nval,1);

rng(12);
n_LC_val = sigma_n*randn(Nval,1);

figure('Name','Señales de excitacion');

subplot(2,1,1);
stairs(t(1:500),r_id(1:500));
grid on;
ylabel('r');
title('Excitacion de identificacion');

subplot(2,1,2);
stairs(tval(1:500),r_val(1:500));
grid on;
ylabel('r val');
xlabel('Tiempo [s]');
title('Excitacion independiente de validacion');

%% 9) VALIDACION INDEPENDIENTE - LA

% Simulacion con nueva entrada
x_LA_val = lsim(G_d,r_val,tval);

% Posicion medida
x_LA_val_med = x_LA_val + n_LA_val;

% Velocidad estimada
v_LA_val = diff(x_LA_val_med)/Ts;

u_LA_val = r_val(2:end);

z_val_ind_LA = iddata(v_LA_val,u_LA_val,Ts);

figure('Name','LA - Validacion independiente');
compare(z_val_ind_LA,G_iv4_LA,G_v_d);
grid on;

%% 10) VALIDACION INDEPENDIENTE - LC

% Posicion fisica con ruido en realimentacion
y_LC_val = lsim(T_ry_d,r_val,tval) - lsim(G_d*Kp*S_d,n_LC_val,tval);

% Posicion medida
y_LC_val_med = y_LC_val + n_LC_val;

% Control real
u_LC_val = Kp*(r_val-y_LC_val_med);

% Validacion de transferencia referencia -> posicion
z_val_ind_LC = iddata(y_LC_val_med,r_val,Ts);

figure('Name','LC - Validacion independiente r -> y');
compare(z_val_ind_LC,G_iv4_LC,T_ry_d);
grid on;

% Validacion de planta u -> posicion
z_val_ind_G_LC = iddata(y_LC_val_med,u_LC_val,Ts);

figure('Name','LC - Validacion independiente planta');
compare(z_val_ind_G_LC,G_iv4_G_LC);
grid on;

%% 11) RESUMEN

fprintf('RESUMEN DEL ENSAYO\n');

fprintf('Frecuencia de muestreo: %.1f Hz\n',fs);
fprintf('Periodo de muestreo: %.3f s\n',Ts);
fprintf('Muestras identificacion total: %d\n',N);
fprintf('Duracion identificacion: %.1f s\n',N*Ts);
fprintf('Muestras validacion independiente: %d\n',Nval);
fprintf('Duracion validacion: %.1f s\n',Nval*Ts);
fprintf('Amplitud: %.4f\n',amp);
fprintf('Ganancia proporcional: %.2f\n',Kp);
fprintf('Ruido posicion sigma: %.2e m\n',sigma_n);

fprintf('\nModelos estimados:\n');
fprintf('LA: u -> velocidad, IV4 [3 3 1]\n');
fprintf('LC: r -> posicion, IV4 [4 4 1]\n');
fprintf('LC: u -> posicion, IV4 [4 4 1]\n');

fprintf('\nNOTA: los ajustes deben analizarse\n');
fprintf('junto con los residuos y la respuesta frecuencial.\n');