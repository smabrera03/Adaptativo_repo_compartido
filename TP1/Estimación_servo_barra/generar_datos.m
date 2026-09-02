t = double(out.tout);
ang_servo = double(out.ang_servo);
ang_barra = double(out.ang_barra);

save('medicion_v2', 't', 'ang_servo', 'ang_barra'); %para volcar los datos en un archivo
%Para recuperar: load('datos_sin_carrito.mat');
