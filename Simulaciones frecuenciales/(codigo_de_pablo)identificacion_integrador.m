clc; close all; clear
% Sistema a identificar
s = tf('s');
G = 1/(s*(s+1));
Ts = 1;

% Sistema discretizado
Gd = c2d(G,Ts)

% Simulación
% Cantidad de muestras
n = 10000;

% Entrada binaria aleatoria
u = sign(randn(n,1));

% Respuesta del sistema a la entrada
y0 = lsim(Gd,u);

% Ruido de medición
e = 0.1*randn(n,1);

% Salida medida
y = y0 + e;

% Identificación
% Correlación entrada-salida

% Horizonte
m = 100;

R = covf([y u],m+1);

% Estimación de la respuesta al impulso
h = R(2,:)'/R(4,1);

g = impulse(Gd,m);
plot([h g],'Linewidth',1.5)
title('Cualquier cosa!')
% Nos da cualquier cosa porque no el proceso no es ESA
% ya que la planta tiene el integrador

%% Vamos a lazo cerrado
% PRBS / referencia binaria
r = sign(randn(n,1));      
% ruido de medición
e = 0.1*randn(n,1);        

Kp = 0.5;

% Lazo cerrado
T = feedback(Kp*Gd,1);

% Simular r -> y
thT = idtf(T);
y0 = idsim(r,thT);
y = y0 + e;

% Recupero la acción de control (Simulé el lazo cerrado: u me quedó perdida
% adentro del sistema)
u = Kp*(r-y0);

R = covf([y u],m+1);
h = R(2,:)'/R(4,1);

figure()
plot([h g],'Linewidth',1.5)
title("No se parecen!")
% Tengo correlación entre u,e => se pierde el principio estadístico que me
% pide que sean independientes!

% Veamos qué podría haber identificado en esta experiencia ...
g_T = impulse(T,m);
R = covf([y r],m+1);
h = R(2,:)'/R(4,1);

figure()
plot([h g_T],'Linewidth',1.5)
title("OK ... Pero NO es la planta que quería estimar!")
% Ahora sí ... cumplimos con ESA y los procesos son independientes
% Pero no es la planta que queríamos estimar!

% Desarmamos el lazo para obtener g(k)?
% Vale la pena, porque la respuesta al impulso de la planta no puede ser
% truncada ya que termina en un valor no nulo. El error que se produce así es
% inmenso.

%% Vamos con periodogramas
% Acá el truco es que no pasamos por la respuesta impulsional
data = iddata(y,r,Ts);
T_est = etfe(data);

% etfe me deveulve un objeto. Se deben extraer los datos para computar la
% respuesta de G
Tresp = T_est.ResponseData;
% Calculo la respuesta numérica de la estimación del bode de G
Gresp = Tresp ./ (Kp*(1-Tresp));
% Armo el objeto G_est para representarlo en el bode
G_est = idfrd(Gresp,T_est.Frequency,T_est.Ts);

figure()
bode(Gd,G_est)
legend('G','Periodograma')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on

% Ahora coincide!
% Puede haber una diferencia de 360 grados en la fase, pero esto no es un error
% Es solo phase wrapping
% Una nota interesante: 
% - en bajas frecuencias es muy impreciso.
% - en frecuencias medias va mejor
% - en altas los perdemos (naturalmente)
% El problema de las bajas es que en el denominador tenemos 1-Tresp, con
% Tresp yendo a 1 (recordemos que tenemos un integrador puro=> al cerrar el
% lazo tenemos dcgain=1)
% Por lo tanto amplificamos enormemente las incertidumbres


%% Vamos con correlogramas
T_est_cr = spa(data);

% Comparemos antes cómo quedaron las identificaciones del sistema de LC
figure()
bode(T,T_est,T_est_cr)
legend('LC','Periodograma','Correlograma')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on

% Ahora sí: descubramos la planta por correlograma!
Tresp = T_est_cr.ResponseData;
% Calculo la respuesta numérica de la estimación del bode de G
Gresp = Tresp ./ (Kp*(1-Tresp));
% Armo el objeto G_est para representarlo en el bode
G_est_cr = idfrd(Gresp,T_est_cr.Frequency,T_est_cr.Ts);

figure()
bode(Gd,G_est_cr)
legend('G','Correlograma')
h = findobj(gcf,'type','line');
set(h,'linewidth',1.5);
grid on

