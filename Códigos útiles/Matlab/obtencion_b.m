clear;
clc;
close all;

%% Datos experimentales
datos = readtable("archivo.xx");

t = datos.t_ms/1000;              % Pasamos a segs
t = t - t(1);

theta_b = deg2rad(datos.theta_b_deg); % [rad] Pasamos los ángulos a rads
x_med = datos.x_cm/100;               % [m] Pasamos a metros

%% Parámetros
m = xx;    % Masa en kg
g = 9.8;      % [m/s^2]

x0 = x_med(1);
v0 = 0;

%% Valores de b a probar (todos son modificables)
valores_b = [0.1 0.2 0.3 0.4 0.5];

%% Ángulo medido como entrada
theta = @(tt) interp1(t,theta_b,tt,"linear","extrap");

%Acá es porque MATLAB solo tiene el ángulo en los instantes en los que Arduino tomó una muestra.
%interp1 interpola entre las mediciones
%% Gráfico
figure;

plot(t,x_med*100,"k","LineWidth",2);
hold on;

for b = valores_b

    modelo = @(tt,z) [
        z(2);
        g*sin(theta(tt)) - (b/m)*z(2)
    ];

    [~,z] = ode45(modelo,t,[x0;v0]);

    plot(t,z(:,1)*100,"LineWidth",1.2);

end
%z1 = x, z_2 = dot{x}= v

grid on;

xlabel("Tiempo [s]");
ylabel("Posición del carrito [cm]");

legend( ...
    "Medición", ...
    "b = 0.1 kg/s", ...
    "b = 0.2 kg/s", ...
    "b = 0.3 kg/s", ...
    "b = 0.4 kg/s", ...
    "b = 0.5 kg/s", ...
    "Location","best");

title("Estimación aproximada del coeficiente de rozamiento");