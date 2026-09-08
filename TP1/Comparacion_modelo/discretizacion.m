clear;
clc;
close all;

% 1. PARAMETROS
m  = 0.033;       % Masa del carrito [kg]
b  = 0.15;        % Rozamiento efectivo [kg/s]
g  = 9.8;         % Gravedad [m/s^2]

a0 = deg2rad(-3.9);   % Offset servo-barra [rad]
a1 = 0.34;            % Pendiente servo-barra

fs = 50;              % Frecuencia de muestreo [Hz]
Ts = 1/fs;            % Periodo de muestreo [s]

fprintf('Ts = %.4f s\n',Ts);


%% 2. PUNTO DE OPERACION

% Barra horizontal:
% 0 = a0 + a1*ue
ue = -a0/a1;       % [rad]

fprintf('Punto de equilibrio del servo:\n');
fprintf('ue = %.4f rad\n',ue);
fprintf('ue = %.2f grados\n',rad2deg(ue));


%% 3. ENTRADA DEL ENSAYO

% Perturbacion respecto del equilibrio
Delta_u_deg = 2;                  % [grados]
Delta_u = deg2rad(Delta_u_deg);   % [rad]

% Comando REAL aplicado al modelo no lineal
u_real = ue + Delta_u;

% Entrada del modelo LINEALIZADO
u_hat = Delta_u;

fprintf('\nEntrada utilizada:\n');
fprintf('u real = %.2f grados\n',rad2deg(u_real));
fprintf('u_hat  = %.2f grados\n',rad2deg(u_hat));


%% 4. TIEMPO DE SIMULACION

tf = 3;

t = 0:Ts:tf;

N = length(t);


%% 5. MODELO NO LINEAL CONTINUO
% x1 = posicion
% x2 = velocidad
% x1_dot = x2
% x2_dot = g*sin(a0+a1*u) - (b/m)*x2

modeloNL = @(tt,x) [
    x(2);
    g*sin(a0 + a1*u_real) - (b/m)*x(2)
];

% Condiciones iniciales
x0 = [0;0];

% Simulacion continua
[t_NL_cont,x_NL_cont] = ode45( ...
    modeloNL, ...
    [0 tf], ...
    x0);

y_NL_cont = x_NL_cont(:,1);


%% 6. MODELO NO LINEAL DISCRETO - EULER

x_NL_disc = zeros(2,N);

x_NL_disc(:,1) = x0;

for k = 1:N-1

    % Estado actual
    x1 = x_NL_disc(1,k);
    x2 = x_NL_disc(2,k);

    % Angulo de barra
    theta_b = a0 + a1*u_real;

    % Euler hacia adelante

    x_NL_disc(1,k+1) = ...
        x1 + Ts*x2;

    x_NL_disc(2,k+1) = ...
        x2 + Ts*( ...
        g*sin(theta_b) ...
        - (b/m)*x2 );

end

y_NL_disc = x_NL_disc(1,:);


%% 7. MODELO LINEAL CONTINUO

A = [
    0       1;
    0      -b/m
];

B = [
    0;
    a1*g
];

C = [1 0];

D = 0;

sys_cont = ss(A,B,C,D);

disp(' ');
disp('Modelo continuo:');
sys_cont


%% 8. MODELO LINEAL DISCRETO - ZOH

sys_ZOH = c2d(sys_cont,Ts,'zoh');

disp(' ');
disp('Modelo discreto ZOH:');
sys_ZOH

disp('Matrices ZOH:');

A_ZOH = sys_ZOH.A
B_ZOH = sys_ZOH.B
C_ZOH = sys_ZOH.C
D_ZOH = sys_ZOH.D


%% 9. MODELO LINEAL DISCRETO - TUSTIN

sys_Tustin = c2d(sys_cont,Ts,'tustin');

disp(' ');
disp('Modelo discreto Tustin:');
sys_Tustin

disp('Matrices Tustin:');

A_Tustin = sys_Tustin.A
B_Tustin = sys_Tustin.B
C_Tustin = sys_Tustin.C
D_Tustin = sys_Tustin.D


%% 10. ENTRADA PARA LOS MODELOS LINEALES

u_lineal = u_hat*ones(size(t));


%% 11. SIMULACION LINEAL CONTINUA

y_L_cont = lsim( ...
    sys_cont, ...
    u_lineal, ...
    t);


%% 12. SIMULACION ZOH

y_ZOH = lsim( ...
    sys_ZOH, ...
    u_lineal, ...
    t);


%% 13. SIMULACION TUSTIN

y_Tustin = lsim( ...
    sys_Tustin, ...
    u_lineal, ...
    t);


%% 14. GRAFICO GENERAL

figure;

plot( ...
    t_NL_cont, ...
    y_NL_cont*100, ...
    'LineWidth',1.8);

hold on;

stairs( ...
    t, ...
    y_NL_disc*100, ...
    'LineWidth',1.3);

plot( ...
    t, ...
    y_L_cont*100, ...
    '--', ...
    'LineWidth',1.8);

stairs( ...
    t, ...
    y_ZOH*100, ...
    'LineWidth',1.3);

stairs( ...
    t, ...
    y_Tustin*100, ...
    'LineWidth',1.3);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion del carrito [cm]');

legend( ...
    'No lineal continuo', ...
    'No lineal discreto - Euler', ...
    'Lineal continuo', ...
    'Lineal discreto - ZOH', ...
    'Lineal discreto - Tustin', ...
    'Location','best');

title(sprintf( ...
    'Comparacion de modelos - Delta u = %.1f grados', ...
    Delta_u_deg));


%% 15. COMPARACION SOLO LINEAL VS NO LINEAL

figure;

plot( ...
    t_NL_cont, ...
    y_NL_cont*100, ...
    'LineWidth',1.8);

hold on;

plot( ...
    t, ...
    y_L_cont*100, ...
    '--', ...
    'LineWidth',1.8);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion del carrito [cm]');

legend( ...
    'Modelo no lineal', ...
    'Modelo linealizado', ...
    'Location','best');

title(sprintf( ...
    'Modelo lineal vs no lineal - Delta u = %.1f grados', ...
    Delta_u_deg));


%% 16. COMPARACION DE DISCRETIZACION LINEAL

figure;

plot( ...
    t, ...
    y_L_cont*100, ...
    'LineWidth',1.8);

hold on;

stairs( ...
    t, ...
    y_ZOH*100, ...
    'LineWidth',1.3);

stairs( ...
    t, ...
    y_Tustin*100, ...
    'LineWidth',1.3);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion del carrito [cm]');

legend( ...
    'Continuo', ...
    'ZOH', ...
    'Tustin', ...
    'Location','best');

title('Comparacion de metodos de discretizacion');


%% 17. COMPARACION CONTINUO VS DISCRETO NO LINEAL

figure;

plot( ...
    t_NL_cont, ...
    y_NL_cont*100, ...
    'LineWidth',1.8);

hold on;

stairs( ...
    t, ...
    y_NL_disc*100, ...
    'LineWidth',1.3);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion del carrito [cm]');

legend( ...
    'No lineal continuo', ...
    'No lineal discreto - Euler', ...
    'Location','best');

title('Discretizacion del modelo no lineal');