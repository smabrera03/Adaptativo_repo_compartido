clear;
clc;
close all;

% Datos experimentales
%datos = readtable("archivo.xx");
load("b_derecha.mat");

t = t - t(1);

theta_b = deg2rad(ang_barra); % [rad] Pasamos los ángulos a rads
theta_s = deg2rad(ang_servo);
a0 = deg2rad(-3.9);
a1 = 0.34;
x_med = pos/100;               % [m] Pasamos a metros

% Parámetros
m = 33/1000;    % Masa en kg
g = 9.8;      % [m/s^2]

x0 = x_med(1);
v0 = 0;

% Valores de b a probar (todos son modificables)
valores_b = [0.2, 0.25, 0.3];
%Nota: En taller de control nos había dado como 0.33, pero probablemente
%estaba mal y fuera ~0.1
% Ángulo medido como entrada
theta_b = @(tt) interp1(t, theta_b, tt, "linear", "extrap");
%Esta función devuelve el valor de theta_b para el tiempo tt interpolando
%las mediciones del vector theta_b

%theta_s = @(tt) interp1(t,theta_s,tt,"linear","extrap");
%Se puede hacer lo mismo con el ángulo comandado al servo, pero eso es
%parte de la otra transferencia. 

%Acá es porque MATLAB solo tiene el ángulo en los instantes en los que Arduino tomó una muestra.
%interp1 interpola entre las mediciones
% Gráfico
figure;

plot(t,x_med*100,"k","LineWidth",2);
hold on;

leyendas = ["Medición"];

for b = valores_b

    modelo = @(tt,z) [
        z(2);
        g*sin(theta_b(tt)) - (b/m)*z(2)
    ];

    [~,z] = ode45(modelo,t,[x0;v0]);

    plot(t,z(:,1)*100,"LineWidth",1.2);
    
    leyendas(end+1) = sprintf("b = %.3f kg/s",b);
end
%z1 = x, z_2 = dot{x}= v

grid on;

xlabel("Tiempo [s]");
ylabel("Posición del carrito [cm]");

legend(leyendas,"Location","best");

title("Estimación aproximada del coeficiente de rozamiento");

figura = gcf;

%exportgraphics(figura, 'Estimación b (izquierda).pdf', 'ContentType', 'image', 'Resolution', 300);

%{
RESULTADOS:
Con la medición b_izquierda obtengo b = 0.09 kg/s
Con la medición b_derecha obtengo b = 0.25 kg/s
Muy asimétrico

Algo más o menos en el medio que represente a los 2:
RESULTADO FINAL:
b = 0.15 kg/s
%}
