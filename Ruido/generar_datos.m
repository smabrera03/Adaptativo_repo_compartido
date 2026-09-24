%{

###################################################################333
                                IMPORTANTE:
Cambiar el nombre de la ruta para cada medición. Si no, los datos se
sobreescriben.

Siempre se guardan la posición, el ángulo comandado al servo y el ángulo
medido en la barra, pero los ángulos no son relevantes en las mediciones
hechas con la barra quieta.
%}



ruta = 'Mediciones/medicion_v6';

t = double(out.tout);

T_MIN = 60; %Tiempo mínimo en segundos
T_MAX = 360;

indx = (T_MIN <= t) & (t < T_MAX);

t = t(indx) - T_MIN;

pos = double(out.pos);
pos = pos(indx);

ang_servo = double(out.ang_servo);
ang_servo = ang_servo(indx);

ang_barra = double(out.ang_barra);
ang_barra = ang_barra(indx);

save(ruta, 't', 'pos', 'ang_servo', 'ang_barra');