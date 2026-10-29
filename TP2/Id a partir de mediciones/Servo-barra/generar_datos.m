close all;
u = double(out.u);
y = double(out.y);
pos = double(out.pos);
t = double(out.tout);

T_MAX = 1 * 30; %Límite de tiempo

indx = t <= T_MAX;

u = u(indx);
y = y(indx);
pos = pos(indx);
t = t(indx);

save('datos_val2.mat', 'u', 'y', 'pos', 't'); %para volcar los datos en un archivo
%Para recuperar: load('datos_sin_carrito.mat');