%% DISENO DE CONTROLADOR IMC - BALL & BEAM

clc; clear; close all;

%% 1. PARAMETROS DE LA PLANTA

m = 0.033;
b = 0.15;
g = 9.8;

q0 = 33.1549;
q1 = 9.736;
ks = 11.4345;

Ts = 0.05;

s = tf('s');

%% 2. MODELO CONTINUO Y DISCRETO

Gc = g*ks/(s*(s+b/m)*(s^2+q1*s+q0));

Gd = c2d(Gc,Ts,'zoh');

% Primera aproximacion:
% utilizar el modelo nominal para el diseno.
% Luego reemplazar por el modelo identificado.

Gd_id = Gd;

disp('Planta discreta:');
Gd

disp('Polos de la planta:');
disp(pole(Gd_id));

disp('Ceros de la planta:');
disp(zero(Gd_id));

%% 3. MODELO OBJETIVO

beta = 0.5;

% Modelo objetivo de segundo orden
F = 1/(beta*s+1)^2;

Fd = c2d(F,Ts,'zoh');

disp('Modelo objetivo discreto:');
Fd

%% 4. CONTROLADOR IMC
% Expresion utilizada por el profesor

Cd = minreal(Fd/(1-Fd)/Gd_id);

disp('Controlador IMC candidato:');
Cd

%% 5. VERIFICACIONES

if ~isproper(Cd)
    error('El controlador obtenido no es causal.');
end

% Transferencias internas del lazo cerrado

S = feedback(1,Cd*Gd);
T = feedback(Cd*Gd,1);

UR = feedback(Cd,Gd);
GS = feedback(Gd,Cd);

estable = isstable(S) && ...
          isstable(T) && ...
          isstable(UR) && ...
          isstable(GS);

disp('Polos del controlador:');
disp(pole(Cd));

disp('Polos del lazo cerrado:');
disp(pole(T));

if ~estable

    warning(['La inversion directa no garantiza ' ...
             'estabilidad interna.']);

    figure;
    pzmap(Cd);
    grid on;
    title('Polos y ceros del controlador candidato');

    return;

end

%% 6. SIMULACION TEMPORAL

t = (0:Ts:10)';

ref = 0.1*(t >= 0.5);

ym = lsim(Fd,ref,t);

y_imc = lsim(T,ref,t);

u_imc = lsim(UR,ref,t);

%% 7. GRAFICOS

figure;

plot(t,ref,'k--','LineWidth',1.5);
hold on;

plot(t,ym,'LineWidth',2);
plot(t,y_imc,'LineWidth',1.5);

grid on;

xlabel('Tiempo [s]');
ylabel('Posicion [m]');

legend('Referencia',...
       'Modelo objetivo',...
       'Respuesta IMC',...
       'Location','southeast');

title('Seguimiento de referencia mediante IMC');

figure;

plot(t,rad2deg(u_imc),'LineWidth',1.5);

grid on;

xlabel('Tiempo [s]');
ylabel('Accion de control [grados]');

title('Accion de control IMC');

figure;

pzmap(T);
grid on;

title('Polos y ceros del sistema realimentado');