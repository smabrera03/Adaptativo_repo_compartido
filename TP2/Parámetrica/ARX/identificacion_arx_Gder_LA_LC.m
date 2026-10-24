%% IDENTIFICACION PARAMETRICA BALL & BEAM - ARX
% Planta no lineal continua (ode45).
% LA y LC: identificacion u -> velocidad (Gder).
% LC mantiene control proporcional de POSICION Kp=2.
% Tres ensayos independientes: identificacion, seleccion y prueba.
% Ruido aditivo a velocidad DESPUES de la simulacion.
% Requiere Control System Toolbox y System Identification Toolbox.
clearvars; close all; clc; rng(1);

%% 1) PARAMETROS Y REFERENCIA TEORICA
m=0.033; b=0.15; g=9.8;
q0=33.1549; q1=9.736; k=11.4345;
fs=50; Ts=1/fs; Kp=2;
Nid=6000; Nsel=3000; Ntest=4000;
ampLA=0.01; ampLC=0.01;
sigma_v=2e-5; % ruido de velocidad [m/s]
s=tf('s');
Gder=(g*k)/((s+b/m)*(s^2+q1*s+q0));
Gder_d=c2d(Gder,Ts,'zoh');

%% 2) ENTRADAS INDEPENDIENTES
uLAid=ampLA*sign(randn(Nid,1));
uLAsel=ampLA*sign(randn(Nsel,1));
uLAtest=ampLA*sign(randn(Ntest,1));
rLCid=ampLC*sign(randn(Nid,1));
rLCsel=ampLC*sign(randn(Nsel,1));
rLCtest=ampLC*sign(randn(Ntest,1));

%% 3) SIMULACION NO LINEAL Y REGISTRO DE SEÑALES
[~,vLAid]=simular_nolineal(uLAid,Ts,m,b,g,q0,q1,k,Kp,false);
[~,vLAsel]=simular_nolineal(uLAsel,Ts,m,b,g,q0,q1,k,Kp,false);
[~,vLAtest]=simular_nolineal(uLAtest,Ts,m,b,g,q0,q1,k,Kp,false);
[yLCid,vLCid]=simular_nolineal(rLCid,Ts,m,b,g,q0,q1,k,Kp,true);
[yLCsel,vLCsel]=simular_nolineal(rLCsel,Ts,m,b,g,q0,q1,k,Kp,true);
[yLCtest,vLCtest]=simular_nolineal(rLCtest,Ts,m,b,g,q0,q1,k,Kp,true);
% Accion de control realmente aplicada en los instantes de muestreo.
% El controlador utiliza posicion sin ruido, como en el codigo original.
uLCid=Kp*(rLCid-yLCid);
uLCsel=Kp*(rLCsel-yLCsel);
uLCtest=Kp*(rLCtest-yLCtest);
% Mediciones de velocidad con ruido independiente.
vLAid=vLAid+sigma_v*randn(Nid,1);
vLAsel=vLAsel+sigma_v*randn(Nsel,1);
vLAtest=vLAtest+sigma_v*randn(Ntest,1);
vLCid=vLCid+sigma_v*randn(Nid,1);
vLCsel=vLCsel+sigma_v*randn(Nsel,1);
vLCtest=vLCtest+sigma_v*randn(Ntest,1);

%% 4) CONJUNTOS DE IDENTIFICACION, SELECCION Y PRUEBA
zLAid=iddata(vLAid,uLAid,Ts);
zLAsel=iddata(vLAsel,uLAsel,Ts);
zLAtest=iddata(vLAtest,uLAtest,Ts);
zLCid=iddata(vLCid,uLCid,Ts);
zLCsel=iddata(vLCsel,uLCsel,Ts);
zLCtest=iddata(vLCtest,uLCtest,Ts);

%% 5) IDENTIFICACION ARX: MISMA ESTRUCTURA PARA LA Y LC
% Gder es de tercer orden. Se mantiene [3 4 1] para comparabilidad.
orden=[3 4 1];
modeloLA=arx(zLAid,orden);
modeloLC=arx(zLCid,orden);
disp('========================================');
disp('IDENTIFICACION ARX - LA: u -> velocidad');
present(modeloLA);
disp('IDENTIFICACION ARX - LC: u -> velocidad');
present(modeloLC);

%% 6) ESTABILIDAD Y VALIDACION INDEPENDIENTE
modelos={modeloLA,modeloLC};
datosSel={zLAsel,zLCsel};
datosTest={zLAtest,zLCtest};
nombres={'LA','LC'};
for j=1:2
    modelo=modelos{j};
    p=pole(modelo);
    estable=isstable(modelo);
    fprintf('ARX %s: max|p|=%.6f, estable=%d\n',nombres{j},max(abs(p)),estable);
    if estable
        fsel=ajuste_sim(datosSel{j},modelo);
        ftest=ajuste_sim(datosTest{j},modelo);
        fprintf('%s: fit seleccion=%.2f %% | prueba final=%.2f %%\n',nombres{j},fsel,ftest);
        figure('Name',['ARX ' nombres{j} ' validacion']);
        compare(datosTest{j},modelo,Gder_d); grid on;
        title(['ARX: ' nombres{j} ' u -> velocidad, prueba independiente']);
        figure('Name',['ARX ' nombres{j} ' Bode']);
        bode(Gder_d,modelo); grid on;
        legend('Lineal teorico Gder','ARX');
        figure('Name',['ARX ' nombres{j} ' residuos']);
        resid(datosTest{j},modelo);
    else
        warning('ARX %s inestable: se omiten simulacion y residuos.',nombres{j});
    end
end

%% 7) SEÑALES DE PRUEBA
te=(0:Ntest-1)'*Ts;
figure('Name','ARX senales LA');
subplot(2,1,1); plot(te,uLAtest); grid on; ylabel('u [rad]'); title('LA: entrada');
subplot(2,1,2); plot(te,vLAtest); grid on; ylabel('v [m/s]'); xlabel('Tiempo [s]');
figure('Name','ARX senales LC');
subplot(3,1,1); plot(te,rLCtest); grid on; ylabel('r [m]'); title('LC: referencia');
subplot(3,1,2); plot(te,uLCtest); grid on; ylabel('u [rad]'); title('Accion de control');
subplot(3,1,3); plot(te,vLCtest); grid on; ylabel('v [m/s]'); xlabel('Tiempo [s]');

%% FUNCIONES LOCALES (SIEMPRE AL FINAL DEL SCRIPT)
function [posicion,velocidad]=simular_nolineal(entrada,Ts,m,b,g,q0,q1,k,Kp,esLC)
    N=numel(entrada); tiempo=(0:N-1)'*Ts;
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
    posicion=estados(:,1);
    velocidad=estados(:,2);
end

function f=ajuste_sim(datos,modelo)
    simulado=compare(datos,modelo);
    y=datos.OutputData(:); yh=simulado.OutputData(:);
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
