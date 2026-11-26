%% VALIDACION DE ESTABILIDAD Y PERTURBACIONES
% Ball & Beam - Hito 3
%
% Validacion mediante simulacion de los
% controladores inicial y final del IFT.
%
% Requiere Control System Toolbox.

clear; close all; clc;

load('resultados_simulaciones\Ajuste PID.mat');

lw = 1.5;

%% 1) PARAMETROS DE LOS CONTROLADORES

theta0 = th(:, 1);
thetaf = th(:, end);

kp0 = theta0(1);
ki0 = theta0(2);
kd0 = theta0(3);

kpf = thetaf(1);
kif = thetaf(2);
kdf = thetaf(3);

%% 2) MODELO NOMINAL DE LA PLANTA

m = 0.033;
b = 0.15;
g = 9.8;

q0 = 33.1549;
q1 = 9.736;
ks = 11.4345;

Ts = mean(diff(t));

s = tf('s');
z = tf('z', Ts);

G = g*ks / ...
    (s*(s+b/m)*(s^2+q1*s+q0));

Gz = c2d(G, Ts, 'zoh');

%% 3) CONTROLADORES DISCRETOS

R0 = kp0;

Rf = kpf * ...
    (1 + kif/(1-z^-1) + kd*(z^-1)*0);

% Estructura PID utilizada en IFT

Rf = kpf * ...
    (1 + kif/(1-z^-1) + kdf*(1-z^-1));

%% 4) TRANSFERENCIAS DE LAZO CERRADO

% Referencia - salida

T0 = feedback(Gz*R0, 1);
Tf = feedback(Gz*Rf, 1);

% Perturbacion de entrada - salida

D0 = feedback(Gz, R0);
Df = feedback(Gz, Rf);

% Sensibilidad

S0 = feedback(1, Gz*R0);
Sf = feedback(1, Gz*Rf);

%% 5) ANALISIS DE ESTABILIDAD

p0 = pole(T0);
pf = pole(Tf);

fprintf('\n--- ESTABILIDAD ---\n');

disp('Polos del controlador inicial:');
disp(p0);

disp('Polos del controlador final:');
disp(pf);

fprintf('Modulo maximo inicial: %.8f\n', ...
    max(abs(p0)));

fprintf('Modulo maximo final: %.8f\n', ...
    max(abs(pf)));

estable0 = isstable(T0);
establef = isstable(Tf);

fprintf('Estabilidad inicial: %d\n', estable0);
fprintf('Estabilidad final:   %d\n', establef);

%% 6) MAPA DE POLOS

ang = linspace(0, 2*pi, 500);

figure;

plot(cos(ang), sin(ang), 'k--', ...
    'LineWidth', lw);

hold on;

plot(real(p0), imag(p0), 'bx', ...
    'MarkerSize', 10, 'LineWidth', 2);

plot(real(pf), imag(pf), 'rx', ...
    'MarkerSize', 10, 'LineWidth', 2);

grid on;
axis equal;

xlabel('Parte real');
ylabel('Parte imaginaria');

title('Polos de lazo cerrado');

legend('Circulo unitario', ...
       'Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

%% 7) RESPUESTA A LARGO PLAZO

Tfin = 30;

tv = (0:Ts:Tfin)';

rv = 0.1*(tv >= 0.5);

y0v = lsim(T0, rv, tv);
yfv = lsim(Tf, rv, tv);

figure;

plot(tv, rv, 'k--', 'LineWidth', lw);
hold on;

plot(tv, y0v, 'LineWidth', lw);
plot(tv, yfv, 'LineWidth', lw);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion [m]');

title('Respuesta simulada a largo plazo');

legend('Referencia', ...
       'Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

%% 8) DEFINICION DE LA PERTURBACION

% Pulso angular de 1 grado.
% Se aplica entre los 5 y 6 segundos.

Ad = deg2rad(1);

td = 5;
duracion = 1;

du = Ad*(tv >= td & tv < td+duracion);

figure;

stairs(tv, rad2deg(du), 'LineWidth', lw);

grid on;

xlabel('Tiempo [s]');
ylabel('Perturbacion [grados]');

title('Perturbacion aplicada a la planta');

%% 9) RESPUESTA A LA PERTURBACION

% Contribucion de la perturbacion a la salida.

yd0 = lsim(D0, du, tv);
ydf = lsim(Df, du, tv);

figure;

plot(tv, yd0, 'LineWidth', lw);
hold on;

plot(tv, ydf, 'LineWidth', lw);

yline(0, 'k--', 'HandleVisibility', 'off');

grid on;

xlabel('Tiempo [s]');
ylabel('Variacion de posicion [m]');

title('Respuesta a una perturbacion de entrada');

legend('Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

%% 10) RESPUESTA TOTAL CON PERTURBACION

% Superposicion valida para el modelo lineal.

yp0 = y0v + yd0;
ypf = yfv + ydf;

figure;

plot(tv, rv, 'k--', 'LineWidth', lw);
hold on;

plot(tv, yp0, 'LineWidth', lw);
plot(tv, ypf, 'LineWidth', lw);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion [m]');

title('Seguimiento en presencia de perturbaciones');

legend('Referencia', ...
       'Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

%% 11) INDICADORES DE RECHAZO

max_d0 = max(abs(yd0));
max_df = max(abs(ydf));

fprintf('\n--- RECHAZO DE PERTURBACIONES ---\n');

fprintf('Maxima desviacion inicial: %.8f m\n', ...
    max_d0);

fprintf('Maxima desviacion final: %.8f m\n', ...
    max_df);

if ~establef
    warning(['El controlador final no estabiliza ' ...
             'el modelo nominal. No puede ' ...
             'garantizarse el rechazo de ' ...
             'perturbaciones a largo plazo.']);
end