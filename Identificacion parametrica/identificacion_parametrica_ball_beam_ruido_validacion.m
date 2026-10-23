%% BALL & BEAM - Identificacion parametrica con ruido y validacion independiente
% LA: u -> velocidad numerica (IV4)
% LC: r -> posicion medida (IV4)
% LC: u -> posicion medida, estimacion directa de la PLANTA (IV4, exploratoria)
% Requiere Control System Toolbox y System Identification Toolbox.
clearvars; close all; clc;
rng(1);

%% 1. Planta linealizada y discretizacion ZOH
m = 0.033; b = 0.15; g = 9.8;
q0 = 33.1549; q1 = 9.736; ks = 11.4345;
fs = 50; Ts = 1/fs; Kp = 2;
Nid = 6000; Nval = 4000;
t_id = (0:Nid-1)'*Ts;
t_val = (0:Nval-1)'*Ts;
amp = 0.01;
sigma_y = 2e-5;  % [m] desviacion estandar del ruido de posicion (supuesto)

Ac = [0 1 0 0; 0 -b/m g 0; 0 0 0 1; 0 0 -q0 -q1];
Bc = [0;0;0;ks]; Cc = [1 0 0 0]; Dc = 0;
G_c = ss(Ac,Bc,Cc,Dc);
G_d = c2d(G_c,Ts,'zoh');
z = tf('z',Ts);
G_v_d = minreal((1-z^-1)*tf(G_d)/Ts,1e-7);
T_ry_d = feedback(Kp*G_d,1); % r -> posicion REAL
S_d = feedback(1,Kp*G_d);    % ruido sensor -> posicion MEDIDA
if ~isstable(T_ry_d)
    error('El lazo cerrado no es estable con Kp=%.3g',Kp);
end

%% 2. Excitacion de identificacion y referencia de validacion independiente
% Identificacion: senal binaria aleatoria (no es PRBS estrictamente).
r_id = amp*sign(randn(Nid,1));
% Validacion: escalones de referencia con niveles aleatorios, permanencia
% de 2 a 5 segundos y transiciones suavizadas (mas cercanos a maniobras).
r_val = zeros(Nval,1);
k = 1;
while k <= Nval
    dwell = randi([round(2/Ts),round(5/Ts)]);
    kfin = min(Nval,k+dwell-1);
    r_val(k:kfin) = amp*(2*rand-1);
    k = kfin+1;
end
% Filtro causal discreto para limitar cambios bruscos de referencia.
alpha = exp(-Ts/0.25);
r_val = filter(1-alpha,[1 -alpha],r_val);

%% 3. LA: simulacion, medicion ruidosa y derivada numerica
% Generamos DOS ensayos independientes con condiciones iniciales nulas.
u_LA_id = r_id;
u_LA_val = r_val;
x_LA_id = lsim(G_d,u_LA_id,t_id);
x_LA_val = lsim(G_d,u_LA_val,t_val);
y_LA_id = x_LA_id + sigma_y*randn(Nid,1);
y_LA_val = x_LA_val + sigma_y*randn(Nval,1);
v_LA_id = diff(y_LA_id)/Ts;
v_LA_val = diff(y_LA_val)/Ts;
% La muestra v(k) usa y(k)-y(k-1); conservamos u(k) para k>=2.
z_id_LA = iddata(v_LA_id,u_LA_id(2:end),Ts);
z_val_LA = iddata(v_LA_val,u_LA_val(2:end),Ts);

%% 4. Identificacion LA u -> velocidad
orden_LA = [3 3 1];
G_iv4_LA = iv4(z_id_LA,orden_LA);
G_spa_LA = spa(z_id_LA);
G_etfe_LA = etfe(z_id_LA);
figure('Name','LA - Bode');
bode(G_v_d,G_spa_LA,G_etfe_LA,G_iv4_LA); grid on;
legend('Teorica','SPA','ETFE','IV4','Location','best');
title('LA: u -> velocidad numerica con ruido');
figure('Name','LA - Validacion independiente');
compare(z_val_LA,G_iv4_LA,G_v_d); grid on;
title('LA: validacion con referencia de niveles y permanencias');
figure('Name','LA - Polos y ceros');
pzmap(G_v_d,G_iv4_LA); grid on; legend('Teorica','IV4');
figure('Name','LA - Residuos');
resid(z_val_LA,G_iv4_LA);

%% 5. LC: simulacion CON ruido dentro de la realimentacion
% y_real = T_ry*r - T_ry*n
% y_med  = T_ry*r + S*n
% u      = Kp*(r-y_med)
% El ruido del sensor entra al controlador, NO solo a la salida registrada.
n_id = sigma_y*randn(Nid,1);
n_val = sigma_y*randn(Nval,1);
y_LC_id = lsim(T_ry_d,r_id,t_id) + lsim(S_d,n_id,t_id);
y_LC_val = lsim(T_ry_d,r_val,t_val) + lsim(S_d,n_val,t_val);
u_LC_id = Kp*(r_id-y_LC_id);
u_LC_val = Kp*(r_val-y_LC_val);
% Para visualizar la posicion fisica real, no la medida:
x_LC_id = y_LC_id-n_id;
x_LC_val = y_LC_val-n_val;

figure('Name','LC - Senales de identificacion');
subplot(3,1,1); plot(t_id,r_id); grid on; ylabel('r [m]');
subplot(3,1,2); plot(t_id,u_LC_id); grid on; ylabel('u');
subplot(3,1,3); plot(t_id,y_LC_id); grid on; ylabel('y med [m]'); xlabel('t [s]');
figure('Name','LC - Validacion: referencia mas realista');
subplot(3,1,1); plot(t_val,r_val); grid on; ylabel('r [m]');
subplot(3,1,2); plot(t_val,u_LC_val); grid on; ylabel('u');
subplot(3,1,3); plot(t_val,y_LC_val); grid on; ylabel('y med [m]'); xlabel('t [s]');

%% 6. LC: identificar transferencia de referencia a posicion medida
z_id_LC = iddata(y_LC_id,r_id,Ts);
z_val_LC = iddata(y_LC_val,r_val,Ts);
orden_LC = [4 4 1];
G_iv4_LC = iv4(z_id_LC,orden_LC);
G_spa_LC = spa(z_id_LC);
G_etfe_LC = etfe(z_id_LC);
figure('Name','LC - Bode r a y');
bode(T_ry_d,G_spa_LC,G_etfe_LC,G_iv4_LC); grid on;
legend('Teorica','SPA','ETFE','IV4','Location','best');
title('LC: referencia -> posicion medida');
figure('Name','LC - Validacion independiente r a y');
compare(z_val_LC,G_iv4_LC,T_ry_d); grid on;
figure('Name','LC - Polos y ceros r a y');
pzmap(T_ry_d,G_iv4_LC); grid on; legend('Teorica','IV4');
figure('Name','LC - Residuos r a y');
resid(z_val_LC,G_iv4_LC,residOptions('MaxLag',100));

%% 7. NUEVO: identificacion directa de la PLANTA en LC, u -> y
% La entrada es el control REALMENTE aplicado, no la referencia r.
% Con ruido dentro del lazo, u y el ruido de y estan correlacionados:
% esta estimacion directa es EXPLORATORIA, no necesariamente insesgada.
z_id_planta = iddata(y_LC_id,u_LC_id,Ts);
z_val_planta = iddata(y_LC_val,u_LC_val,Ts);
orden_planta = [4 4 1];
G_iv4_planta = iv4(z_id_planta,orden_planta);
% Comparacion adicional ARX para apreciar posible sesgo.
G_arx_planta = arx(z_id_planta,orden_planta);
figure('Name','Planta - Bode u a y en LC');
bode(G_d,G_iv4_planta,G_arx_planta); grid on;
legend('Planta teorica','IV4','ARX','Location','best');
title('Estimacion directa de planta u -> y con realimentacion');
figure('Name','Planta - Polos y ceros');
pzmap(G_d,G_iv4_planta,G_arx_planta); grid on;
legend('Teorica','IV4','ARX');
figure('Name','Planta - Validacion independiente');
compare(z_val_planta,G_iv4_planta,G_arx_planta,G_d); grid on;
title('Planta u -> y: interpretar el fit con cautela');
figure('Name','Planta - Residuos IV4');
resid(z_val_planta,G_iv4_planta,residOptions('MaxLag',100));

%% 8. Resultados y notas
fprintf('\n--- CONDICIONES ---\n');
fprintf('Ts=%.3f s, fs=%.1f Hz, Nid=%d, Nval=%d, Kp=%.2f\n',Ts,fs,Nid,Nval,Kp);
fprintf('Amplitud excitacion=%.4g, sigma ruido posicion=%.3g m\n',amp,sigma_y);
fprintf('Ruido velocidad aprox. sigma_v=%.4g m/s\n',sqrt(2)*sigma_y/Ts);
disp('IV4 LA u->v:'); G_iv4_LA
 disp('IV4 LC r->y:'); G_iv4_LC
 disp('IV4 planta u->y (exploratorio):'); G_iv4_planta
 disp('ARX planta u->y (exploratorio):'); G_arx_planta
% Importante: la validacion usa un ensayo NUEVO y un tipo de referencia
% distinto, pero no sustituye ensayos sobre la planta fisica.
% En LC, una identificacion consistente de planta con ruido puede requerir
% metodos especificos para realimentacion o variables instrumentales
% basadas en senales exogenas, como la referencia.

%% DIAGNOSTICO DE ESTABILIDAD

modelos = {G_iv4_LA, G_iv4_LC, ...
           G_iv4_planta, G_arx_planta};

nombres = {'IV4 LA', 'IV4 LC', ...
           'IV4 planta', 'ARX planta'};

for i = 1:length(modelos)

    p = pole(modelos{i});

    fprintf('\n===== %s =====\n', nombres{i});

    disp('Polos:');
    disp(p);

    fprintf('Modulo maximo: %.6f\n', max(abs(p)));

    if isstable(modelos{i})
        fprintf('Modelo estable\n');
    else
        fprintf('MODELO NO ESTABLE\n');
    end

end