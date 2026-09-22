t = double(out.tout);

T_MIN = 10; %Tiempo mínimo en segundos
T_MAX = 110;

indx = (T_MIN <= t) & (t < T_MAX);

t = t(indx) - T_MIN;

pos = double(out.pos);
pos = pos(indx);

save('Mediciones/medicion_v1', 't', 'pos'); %para volcar los datos en un archivo
