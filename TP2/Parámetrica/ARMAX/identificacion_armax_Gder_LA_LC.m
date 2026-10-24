%% IDENTIFICACION PARAMETRICA BALL & BEAM - ARMAX, Gder EN LA Y LC
% Adaptado del codigo original: planta no lineal ode45.
% LA: accion u -> velocidad v; LC: accion REAL u=Kp*(r-y) -> velocidad v.
% Controlador P continuo Kp=2 con posicion SIN ruido en la realimentacion.
% Ruido aditivo SOLO en la velocidad registrada, despues de simular.
% Tres ensayos independientes: identificacion, seleccion, prueba final.
% Requiere Control System Toolbox y System Identification Toolbox.
clearvars; close all; clc; rng(1);

%% 1) PARAMETROS Y TRANSFERENCIA TEORICA
m=0.033; b=0.15; g=9.8;
q0=33.1549; q1=9.736; k=11.4345;
fs=50; Ts=1/fs; Kp=2;
Nid=6000; Nsel=3000; Ntest=4000;
ampLA=0.01; ampLC=0.01;
sigma_v=2e-5; % ruido de velocidad [m/s]
s=tf('s');
Gder=(g*k)/((s+b/m)*(s^2+q1*s+q0));
Gder_d=c2d(Gder,Ts,'zoh');

%% 2) ENTRADAS ALEATORIAS INDEPENDIENTES
uLAid=ampLA*sign(randn(Nid,1));
uLAsel=ampLA*sign(randn(Nsel,1));
uLAtest=ampLA*sign(randn(Ntest,1));
rLCid=ampLC*sign(randn(Nid,1));
rLCsel=ampLC*sign(randn(Nsel,1));
rLCtest=ampLC*sign(randn(Ntest,1));

%% 3) SIMULACION NO LINEAL CON ODE45
% LA: devuelve velocidad; se mantiene el experimento original.
vLAid=simular_nolineal(uLAid,Ts,m,b,g,q0,q1,k,Kp,false);
vLAsel=simular_nolineal(uLAsel,Ts,m,b,g,q0,q1,k,Kp,false);
vLAtest=simular_nolineal(uLAtest,Ts,m,b,g,q0,q1,k,Kp,false);
% LC: devuelve velocidad y accion de control, calculada con posicion SIN ruido.
[vLCid,uLCid]=simular_nolineal(rLCid,Ts,m,b,g,q0,q1,k,Kp,true);
[vLCsel,uLCsel]=simular_nolineal(rLCsel,Ts,m,b,g,q0,q1,k,Kp,true);
[vLCtest,uLCtest]=simular_nolineal(rLCtest,Ts,m,b,g,q0,q1,k,Kp,true);
% Se preserva el orden de generacion del ruido LA; LC ahora mide velocidad.
vLAid=vLAid+sigma_v*randn(Nid,1);
vLAsel=vLAsel+sigma_v*randn(Nsel,1);
vLAtest=vLAtest+sigma_v*randn(Ntest,1);
vLCid=vLCid+sigma_v*randn(Nid,1);
vLCsel=vLCsel+sigma_v*randn(Nsel,1);
vLCtest=vLCtest+sigma_v*randn(Ntest,1);

%% 4) DATOS PARA IDENTIFICACION: AMBOS LAZOS u -> velocidad
zLAid=iddata(vLAid,uLAid,Ts);
zLAsel=iddata(vLAsel,uLAsel,Ts);
zLAtest=iddata(vLAtest,uLAtest,Ts);
zLCid=iddata(vLCid,uLCid,Ts);
zLCsel=iddata(vLCsel,uLCsel,Ts);
zLCtest=iddata(vLCtest,uLCtest,Ts);

%% 5) IDENTIFICACION ARMAX
% Se mantiene LA [3 4 2 1]. LC usa el mismo orden de planta (tercer orden).
opciones=armaxOptions;
opciones.SearchOptions.MaxIterations=100;
modeloLA=armax(zLAid,[3 4 2 1],opciones);
modeloLC=armax(zLCid,[3 4 2 1],opciones);
disp('========================================');
disp('IDENTIFICACION ARMAX - LA: u -> velocidad');
present(modeloLA);
disp('IDENTIFICACION ARMAX - LC: u -> velocidad');
present(modeloLC);

%% 6) ESTABILIDAD Y VALIDACION INDEPENDIENTE
pLA=pole(modeloLA); pLC=pole(modeloLC);
estableLA=isstable(modeloLA); estableLC=isstable(modeloLC);
fprintf('\nARMAX LA: max|p|=%.6f, estable=%d\n',max(abs(pLA)),estableLA);
fprintf('ARMAX LC: max|p|=%.6f, estable=%d\n',max(abs(pLC)),estableLC);
if estableLA
    fselLA=ajuste_sim(zLAsel,modeloLA);
    ftestLA=ajuste_sim(zLAtest,modeloLA);
    fprintf('LA: fit seleccion=%.2f %% | prueba final=%.2f %%\n',fselLA,ftestLA);
    figure('Name','ARMAX LA validacion');
    compare(zLAtest,modeloLA,Gder_d); grid on;
    title('ARMAX: LA u -> velocidad, prueba independiente');
    figure('Name','ARMAX LA Bode');
    bode(Gder_d,modeloLA); grid on;
    legend('Lineal teorico Gder','ARMAX');
    figure('Name','ARMAX LA residuos');
    resid(zLAtest,modeloLA);
else
    warning('ARMAX LA inestable: no se simula ni se ejecuta resid.');
end
if estableLC
    fselLC=ajuste_sim(zLCsel,modeloLC);
    ftestLC=ajuste_sim(zLCtest,modeloLC);
    fprintf('LC: fit seleccion=%.2f %% | prueba final=%.2f %%\n',fselLC,ftestLC);
    figure('Name','ARMAX LC validacion');
    compare(zLCtest,modeloLC,Gder_d); grid on;
    title('ARMAX: LC u -> velocidad, prueba independiente');
    figure('Name','ARMAX LC Bode');
    bode(Gder_d,modeloLC); grid on;
    legend('Lineal teorico Gder','ARMAX');
    figure('Name','ARMAX LC residuos');
    resid(zLCtest,modeloLC);
else
    warning('ARMAX LC inestable: no se simula ni se ejecuta resid.');
end

%% 7) GRAFICOS DE ENTRADA Y SALIDA
test_t=(0:Ntest-1)'*Ts;
figure('Name','ARMAX senales LA');
subplot(2,1,1); plot(test_t,uLAtest); grid on; ylabel('u [rad]'); title('LA: entrada');
subplot(2,1,2); plot(test_t,vLAtest); grid on; ylabel('v [m/s]'); xlabel('Tiempo [s]');
figure('Name','ARMAX senales LC');
subplot(3,1,1); plot(test_t,rLCtest); grid on; ylabel('r [m]'); title('LC: referencia');
subplot(3,1,2); plot(test_t,uLCtest); grid on; ylabel('u [rad]'); title('Accion de control');
subplot(3,1,3); plot(test_t,vLCtest); grid on; ylabel('v [m/s]'); xlabel('Tiempo [s]');

%% FUNCIONES LOCALES: NO COLOCAR INSTRUCCIONES DE SCRIPT DEBAJO
function [salida,accion]=simular_nolineal(entrada,Ts,m,b,g,q0,q1,k,Kp,esLC)
    N=numel(entrada); tiempo=(0:N-1)'*Ts;
    % LA: ZOH. LC: interpolacion lineal de referencia como codigo original.
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
    salida=estados(:,2); % velocidad real, ANTES del ruido de medicion
    if esLC
        accion=Kp*(entrada-estados(:,1)); % u(t_k) exacta en instantes muestreados
    else
        accion=entrada;
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
