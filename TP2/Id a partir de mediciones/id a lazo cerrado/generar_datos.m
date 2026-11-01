close all;
r = double(out.r);
u = double(out.u);
ang_barra = double(out.ang_barra);
y = double(out.y);
t = double(out.tout);

T_MIN = 10; %Descartar los primeros 10 s xq el carro puede haber empezado en cualquier lado
T_MAX = 3 * 60; %Límite de tiempo

indx = (T_MIN <= t) & (t < T_MAX);

r = r(indx);
u = u(indx);
ang_barra = ang_barra(indx);
y = y(indx);
t = t(indx);

save('datos_LC_v1.mat', 'r', 'u','ang_barra', 'y', 't'); %para volcar los datos en un archivo

