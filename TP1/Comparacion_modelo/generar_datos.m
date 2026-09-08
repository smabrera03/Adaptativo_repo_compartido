t = double(out.tout);
ang_servo = double(out.ang_servo);
ang_barra = double(out.ang_barra);
pos = double(out.pos);

T_CORTE = 2; %tiempo para cortar los datos
indx = t < T_CORTE;
t = t(indx);
ang_servo = ang_servo(indx);
ang_barra = ang_barra(indx);
pos = pos(indx);

save('b_derecha', 't', 'ang_servo', 'ang_barra','pos'); %para volcar los datos en un archivo
%Para recuperar: load('datos_sin_carrito.mat');
