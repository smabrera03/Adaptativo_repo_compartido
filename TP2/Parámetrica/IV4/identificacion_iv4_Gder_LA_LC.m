%% IDENTIFICACION PARAMETRICA BALL & BEAM - IV4
% Basado en lazo_abierto.m, Derivada.m y lazo_cerrado.m.
% Simulacion NO LINEAL con ode45; modelo linealizado solo para comparar.
% LA y LC: identificacion u -> velocidad x(2) (Gder).
% LC: referencia r excita el lazo; controlador P continuo Kp=2 sobre posicion.
% En LC, como en el codigo original, el controlador utiliza posicion SIN ruido;
% el ruido de velocidad se agrega a la medicion al finalizar la simulacion.
% Tres ensayos independientes: identificacion, seleccion y prueba final.
% Requiere Control System Toolbox y System Identification Toolbox.
% Metodo: IV4: variables instrumentales, estructura ARX
clearvars; close all; clc; rng(1);

%% 1) PARAMETROS Y MODELOS LINEALES TEORICOS
m=0.033; b=0.15; g=9.8;
q0=33.1549; q1=9.736; k=11.4345;
fs=50; Ts=1/fs; Kp=2;
Nid=6000; Nsel=3000; Ntest=4000;
ampLA=0.01; ampLC=0.01; % pequenas amplitudes: aproximacion lineal
sigma_v=2e-5; % ruido ADITIVO de velocidad [m/s], supuesto
s=tf('s');
G=(g*k)/(s*(s+b/m)*(s^2+q1*s+q0));
Gder=(g*k)/((s+b/m)*(s^2+q1*s+q0));
Gder_d=c2d(Gder,Ts,'zoh');

%% 2) TRES ENTRADAS INDEPENDIENTES (semilla fija para reproducibilidad)
% Entrada binaria aleatoria: igual principio que en los codigos no parametricos.
% Nota: LC tiene referencia interpolada linealmente y control continuo;
% la comparacion con Gder_d (ZOH) es una referencia aproximada.
uLAid=ampLA*sign(randn(Nid,1));
uLAsel=ampLA*sign(randn(Nsel,1));
uLAtest=ampLA*sign(randn(Ntest,1));
rLCid=ampLC*sign(randn(Nid,1));
rLCsel=ampLC*sign(randn(Nsel,1));
rLCtest=ampLC*sign(randn(Ntest,1));

%% 3) GENERACION DE DATOS CON ODE45 (MODELO NO LINEAL)
% LA: ode45 integra g*sin(theta), salida es el estado velocidad x(2).
vLAid=simular_nolineal(uLAid,Ts,m,b,g,q0,q1,k,Kp,false);
vLAsel=simular_nolineal(uLAsel,Ts,m,b,g,q0,q1,k,Kp,false);
vLAtest=simular_nolineal(uLAtest,Ts,m,b,g,q0,q1,k,Kp,false);
% LC: obtener posicion y velocidad de la misma simulacion no lineal.
[yLCid,vLCid]=simular_nolineal(rLCid,Ts,m,b,g,q0,q1,k,Kp,true);
[yLCsel,vLCsel]=simular_nolineal(rLCsel,Ts,m,b,g,q0,q1,k,Kp,true);
[yLCtest,vLCtest]=simular_nolineal(rLCtest,Ts,m,b,g,q0,q1,k,Kp,true);
% Accion de control muestreada, calculada con posicion SIN ruido.
uLCid=Kp*(rLCid-yLCid);
uLCsel=Kp*(rLCsel-yLCsel);
uLCtest=Kp*(rLCtest-yLCtest);
% Ruido aditivo independiente sobre la velocidad medida.
vLAid=vLAid+sigma_v*randn(Nid,1);
vLAsel=vLAsel+sigma_v*randn(Nsel,1);
vLAtest=vLAtest+sigma_v*randn(Ntest,1);
vLCid=vLCid+sigma_v*randn(Nid,1);
vLCsel=vLCsel+sigma_v*randn(Nsel,1);
vLCtest=vLCtest+sigma_v*randn(Ntest,1);

%% 4) ORGANIZACION EN IDDATA
zLAid=iddata(vLAid,uLAid,Ts);
zLAsel=iddata(vLAsel,uLAsel,Ts);
zLAtest=iddata(vLAtest,uLAtest,Ts);
zLCid=iddata(vLCid,uLCid,Ts);
zLCsel=iddata(vLCsel,uLCsel,Ts);
zLCtest=iddata(vLCtest,uLCtest,Ts);

%% 5) IDENTIFICACION IV4
% Orden LA y LC: [3 4 1], estructura de tercer orden para Gder.
modeloLA=iv4(zLAid,[3 4 1]);
modeloLC=iv4(zLCid,[3 4 1]);

disp('========================================');
disp('IDENTIFICACION IV4 - LA: u -> velocidad');
present(modeloLA);
disp('IDENTIFICACION IV4 - LC: u -> velocidad');
present(modeloLC);

%% 6) ESTABILIDAD Y VALIDACION INDEPENDIENTE
pLA=pole(modeloLA); pLC=pole(modeloLC);
estableLA=isstable(modeloLA); estableLC=isstable(modeloLC);
fprintf('\nIV4 LA: max|p|=%.6f, estable=%d\n',max(abs(pLA)),estableLA);
fprintf('IV4 LC: max|p|=%.6f, estable=%d\n',max(abs(pLC)),estableLC);
if estableLA
    fselLA=ajuste_sim(zLAsel,modeloLA);
    ftestLA=ajuste_sim(zLAtest,modeloLA);
    fprintf('LA: fit seleccion=%.2f %% | prueba final=%.2f %%\n',fselLA,ftestLA);
    figure('Name','IV4 LA validacion');
    compare(zLAtest,modeloLA,Gder_d); grid on;
    title('IV4: LA u -> velocidad, prueba independiente');
    figure('Name','IV4 LA Bode');
    bode(Gder_d,modeloLA); grid on;
    legend('Lineal teorico','IV4');
    figure('Name','IV4 LA residuos');
    resid(zLAtest,modeloLA);
else
    warning('IV4 LA inestable: no se simula ni se ejecuta resid.');
end
if estableLC
    fselLC=ajuste_sim(zLCsel,modeloLC);
    ftestLC=ajuste_sim(zLCtest,modeloLC);
    fprintf('LC: fit seleccion=%.2f %% | prueba final=%.2f %%\n',fselLC,ftestLC);
    figure('Name','IV4 LC validacion');
    compare(zLCtest,modeloLC,Gder_d); grid on;
    title('IV4: LC u -> velocidad, prueba independiente');
    figure('Name','IV4 LC Bode');
    bode(Gder_d,modeloLC); grid on;
    legend('Lineal teorico','IV4');
    figure('Name','IV4 LC residuos');
    resid(zLCtest,modeloLC);
else
    warning('IV4 LC inestable: no se simula ni se ejecuta resid.');
end

%% 7) GRAFICOS DE ENTRADA Y SALIDA (COMO EN LOS NO PARAMETRICOS)
test_t=(0:Ntest-1)'*Ts;
figure('Name','IV4 senales LA');
subplot(2,1,1); plot(test_t,uLAtest); grid on; ylabel('u [rad]'); title('LA: entrada');
subplot(2,1,2); plot(test_t,vLAtest); grid on; ylabel('v [m/s]'); xlabel('Tiempo [s]');
figure('Name','IV4 senales LC');
subplot(2,1,1); plot(test_t,uLCtest); grid on; ylabel('u [rad]'); title('LC: accion de control');
subplot(2,1,2); plot(test_t,vLCtest); grid on; ylabel('v [m/s]'); xlabel('Tiempo [s]');

%% FUNCIONES LOCALES: NO COLOCAR INSTRUCCIONES DE SCRIPT DEBAJO
function [salida,velocidad]=simular_nolineal(entrada,Ts,m,b,g,q0,q1,k,Kp,esLC)
    N=numel(entrada); tiempo=(0:N-1)'*Ts;
    % LA: ZOH, como lazo_abierto.m. LC: interpolacion lineal, como lazo_cerrado.m.
    if esLC
        entrada_cont=@(tt) interp1(tiempo,entrada,tt,'linear','extrap');
    else
        entrada_cont=@(tt) interp1(tiempo,entrada,tt,'previous','extrap');
    end
    if esLC
        dinamica=@(tt,x)[x(2);g*sin(x(3))-(b/m)*x(2);x(4); ...
            -q0*x(3)-q1*x(4)+k*Kp*(entrada_cont(tt)-x(1))];
    else
        dinamica=@(tt,x)[x(2);g*sin(x(3))-(b/m)*x(2);x(4); ...
            -q0*x(3)-q1*x(4)+k*entrada_cont(tt)];
    end
    opts=odeset('MaxStep',Ts/5);
    [~,estados]=ode45(dinamica,tiempo,[0;0;0;0],opts);
    if esLC
        salida=estados(:,1); % posicion (para calcular u real)
        velocidad=estados(:,2); % velocidad para identificar Gder
    else
        salida=estados(:,2); % velocidad real, ANTES de agregar ruido
        velocidad=salida;
    end
end

function f=ajuste_sim(datos,modelo)
    pred=compare(datos,modelo);
    y=datos.OutputData(:); yh=pred.OutputData(:);
    if any(~isfinite(yh)) || numel(y)~=numel(yh)
        f=NaN; return;
    end
    den=norm(y-mean(y));
    if den<=eps
        f=NaN;
    else
        f=100*(1-norm(y-yh)/den);
    end
end
