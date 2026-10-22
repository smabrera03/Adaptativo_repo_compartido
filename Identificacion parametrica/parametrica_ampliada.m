%% IDENTIFICACION PARAMETRICA - BALL & BEAM
% Comparacion ARX, IV4 y RLS
% Planta: entrada u -> velocidad estimada v

close all; clear; clc;

rng(1);

%% 1. PARAMETROS DE LA PLANTA

m  = 0.033;
b  = 0.15;
g  = 9.8;

q0 = 33.1549;
q1 = 9.736;
ks = 11.4345;

fs = 50;
Ts = 1/fs;

N = 10000;
t = (0:N-1)'*Ts;

s = tf('s');

% Transferencia continua: control -> posicion
Gx = (g*ks)/(s*(s+b/m)*(s^2+q1*s+q0));

% Discretizacion
Gd = c2d(Gx,Ts,'zoh');

% Transferencia discreta exacta:
% control -> diferencia hacia atras de posicion
z = tf('z',Ts);
Gv = minreal((1-z^-1)/Ts * Gd);

disp('Modelo teorico de velocidad:');
Gv

%% 2. SIMULACION DEL EXPERIMENTO

A = 0.01;

% Entrada binaria aleatoria
u = A*sign(randn(N,1));

% Simulacion de posicion
x = lsim(Gd,u,t);

% Ruido de medicion de posicion
sigma_n = 1e-6;

n = sigma_n*randn(N,1);

x_med = x + n;

% Velocidad estimada mediante diferencia hacia atras
v_med = [0; diff(x_med)/Ts];

% Se elimina la primera muestra por la diferenciacion
u_id = u(2:end);
v_id = v_med(2:end);

%% 3. DATOS DE IDENTIFICACION Y VALIDACION

Ndatos = length(v_id);
Nid = floor(0.6*Ndatos);

z_id = iddata(v_id(1:Nid),u_id(1:Nid),Ts);

z_val = iddata(v_id(Nid+1:end),u_id(Nid+1:end),Ts);

%% 4. IDENTIFICACION ARX

% Orden esperado: 3
na = 3;
nb = 3;
nk = 1;

m_arx = arx(z_id,[na nb nk]);

%% 5. IDENTIFICACION IV4

m_iv4 = iv4(z_id,[na nb nk]);

%% 6. IDENTIFICACION RLS

% Modelo:
% y(k) = -a1*y(k-1)-...-a3*y(k-3)
%        +b1*u(k-1)+...+b3*u(k-3)

y = v_id(1:Nid);
u_rls = u_id(1:Nid);

npar = na+nb;

theta = zeros(npar,1);
P = 1000*eye(npar);

lambda = 1;

theta_hist = zeros(Nid,npar);
error_rls = zeros(Nid,1);

for k = max(na,nk+nb-1)+1:Nid

    phi = [
        -y(k-1:-1:k-na);
         u_rls(k-nk:-1:k-nk-nb+1)
    ];

    y_hat = phi'*theta;

    error_rls(k) = y(k)-y_hat;

    K = (P*phi)/(lambda+phi'*P*phi);

    theta = theta+K*error_rls(k);

    P = (P-K*phi'*P)/lambda;

    theta_hist(k,:) = theta';

end

% Reconstruccion del modelo RLS
A_rls = [1; theta(1:na)];
B_rls = [zeros(nk,1); theta(na+1:end)];

m_rls = idpoly(A_rls',B_rls',[],[],[],[],Ts);

%% 7. COMPARACION EN FRECUENCIA

figure;

bode(Gv,m_arx,m_iv4,m_rls);
grid on;

legend('Teorico','ARX','IV4','RLS');

title('Ball & Beam - Comparacion de Bode');

%% 8. MAPA DE POLOS Y CEROS

figure;

pzmap(Gv,m_arx,m_iv4,m_rls);
grid on;

legend('Teorico','ARX','IV4','RLS');

title('Comparacion de polos y ceros');

%% 9. VALIDACION TEMPORAL

figure;

compare(z_val,m_arx,m_iv4,m_rls);
grid on;

title('Validacion de modelos');

%% 10. ANALISIS DE RESIDUOS

figure;
resid(z_val,m_arx);
title('Residuos ARX');

figure;
resid(z_val,m_iv4);
title('Residuos IV4');

figure;
resid(z_val,m_rls);
title('Residuos RLS');

%% 11. EVOLUCION DE PARAMETROS RLS

figure;

plot(t(2:Nid+1),theta_hist,'LineWidth',1.2);
grid on;

xlabel('Tiempo [s]');
ylabel('Parametros estimados');

legend('a1','a2','a3','b1','b2','b3');

title('Convergencia de parametros - RLS');

%% 12. RESULTADOS

disp('ARX');
present(m_arx);

disp('IV4');
present(m_iv4);

disp('RLS');
present(m_rls);