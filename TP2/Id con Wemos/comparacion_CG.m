%{

Comparar el mismo modelo, pero de distintos ordenes

Probar un ARX de alto orden para reducirlo

No considerar integrador

%}

clear; close all; clc;

load('datos_cart_beam.mat');

%% IDENTIFICACION PARAMETRICA BALL & BEAM - ARX - DISTINTOS ORDENES
% Mismo esquema de los codigos originales de H2, con datos experimentales.
% Entrada: accion realmente aplicada u_wemos [rad].
% Salida: velocidad del carro [m/s], SIN integrador de posicion.
% Ordenes 2, 3, 4, 5 y 6; el orden fisico esperado es 3.
% Un registro experimental dividido en identificacion, seleccion y prueba.
% Requiere Control System Toolbox y System Identification Toolbox.
% ARX estima sus coeficientes por minimos cuadrados.
clearvars; close all; clc;

%% 1) PARAMETROS Y REFERENCIA TEORICA
m=0.033; b=0.15; g=9.8;
q0=33.1549; q1=9.736; k=11.4345;
ordenes=[2 3 4 5 6];  % orden fisico de referencia: 3

ordenFisico = 3;  % Orden teorico de la planta de velocidad
ordenAlto = 6;    % Orden inicial para reduccion

s=tf('s');
Gder=(g*k)/((s+b/m)*(s^2+q1*s+q0));

%% 2) CARGA DE DATOS EXPERIMENTALES Y SEÑALES
S=load('datos_cart_beam.mat');
if isfield(S,'T')
    % Formato experimental original especificado por el grupo:
    % T(:,1) timestamp_us; T(:,2) q1; T(:,3) q2;
    % T(:,4) q2_p_f; T(:,5) u_wemos; T(:,6) r; T(:,7) q1_ref.
    T=S.T;
    assert(isnumeric(T) && size(T,2)>=7,'T debe tener 7 columnas.');
    timestamp_us=double(T(:,1));
    q1=double(T(:,2)); q2=double(T(:,3));
    v=double(T(:,4)); u=double(T(:,5));
    r=double(T(:,6)); q1_ref=double(T(:,7)); %#ok<NASGU>
    % micros() de 32 bits: diff con modulo para eventual desbordamiento.
    dt_us=mod(diff(timestamp_us),2^32);
    Ts=median(dt_us)/1e6;
    fprintf('Fuente: matriz T; velocidad filtrada T(:,4).\n');
elseif isfield(S,'datos') && istable(S.datos)
    % Compatibilidad con el .mat adjunto: tabla con otros nombres.
    datos=S.datos;
    necesarias={'t','q1','q2','q2_p','u','r'};
    assert(all(ismember(necesarias,datos.Properties.VariableNames)), ...
        'Faltan columnas esperadas en la tabla datos.');
    q1=double(datos.q1); q2=double(datos.q2);
    v=double(datos.q2_p); u=double(datos.u);
    r=double(datos.r); %#ok<NASGU>
    Ts=median(diff(double(datos.t)));
    fprintf('Fuente: tabla datos; q2_p es velocidad filtrada (confirmado).\n');
else
    error('El archivo debe contener una matriz T o la tabla datos.');
end
u=u(:); v=v(:); q1=q1(:); q2=q2(:);
assert(all(isfinite(u)) && all(isfinite(v)) && isfinite(Ts) && Ts>0, ...
    'Hay muestras no finitas o un periodo de muestreo invalido.');
N=numel(v); fs=1/Ts;
assert(N>=100,'Se necesitan suficientes muestras para comparar ordenes.');
fprintf('Muestras=%d | Ts=%.6f s | fs=%.3f Hz\n',N,Ts,fs);
Gder_d=c2d(Gder,Ts,'zoh'); % referencia nominal, no modelo experimental

%% 3) CONJUNTOS DE IDENTIFICACION, SELECCION Y PRUEBA
% Un unico registro experimental: division cronologica 60/20/20.
% No se inventan ensayos independientes LA y LC.
i1=1:floor(0.60*N);
i2=(i1(end)+1):floor(0.80*N);
i3=(i2(end)+1):N;
zId=iddata(v(i1),u(i1),Ts);
zSel=iddata(v(i2),u(i2),Ts);
zTest=iddata(v(i3),u(i3),Ts);
fprintf('Identificacion=%d | Seleccion=%d | Prueba=%d\n', ...
    numel(i1),numel(i2),numel(i3));

%% 4) IDENTIFICACION ARX: DISTINTOS ORDENES
% Estructura [na nb nk] = [n n+1 1], como el original [3 4 1].
modelos=cell(numel(ordenes),1);
fitSel=nan(numel(ordenes),1);
estable=false(numel(ordenes),1);
maxPolo=nan(numel(ordenes),1);
for j=1:numel(ordenes)
    n=ordenes(j);
    estructura=[n n+1 1];
    modelos{j}=arx(zId,estructura);
    p=pole(modelos{j});
    maxPolo(j)=max(abs(p));
    estable(j)=isstable(modelos{j});
    fprintf('\n========================================\n');
    fprintf('ARX orden %d | estructura [%d %d %d]\n',n,estructura);
    present(modelos{j});
    fprintf('max|p|=%.6f | estable=%d\n',maxPolo(j),estable(j));
    if estable(j)
        fitSel(j)=ajuste_sim(zSel,modelos{j});
        fprintf('Fit seleccion=%.2f %%\n',fitSel(j));
    else
        warning('ARX orden %d inestable: no se considera para seleccion.',n);
    end
end

%% 5) SELECCION DEL ORDEN Y VALIDACION INDEPENDIENTE
% Se elige usando SOLO seleccion. La prueba final se consulta despues.
[mejorFit,idx]=max(fitSel);
if ~isfinite(mejorFit)
    error('Ningun modelo estable produjo un ajuste valido en seleccion.');
end
modeloElegido=modelos{idx};
ordenElegido=ordenes(idx);
fitTest=ajuste_sim(zTest,modeloElegido);
fprintf('\n========================================\n');
fprintf('ARX SELECCIONADO: orden %d | seleccion %.2f %% | prueba %.2f %%\n', ...
    ordenElegido,mejorFit,fitTest);

% Tabla: la prueba final solo se informa para el modelo elegido.
resultado=table(ordenes(:),estable,maxPolo,fitSel, ...
    'VariableNames',{'Orden','Estable','MaxAbsPolo','FitSeleccion'});
disp(resultado);

%% 6) GRAFICOS DE VALIDACION Y COMPARACION FRECUENCIAL
% Los titulos distinguen seleccion, prueba final y referencia teorica.
figure('Name','ARX | Seleccion del orden del modelo');
plot(ordenes,fitSel,'o-','LineWidth',1.5); grid on;
xlabel('Orden del modelo n'); ylabel('Ajuste en seleccion (fit) [%]');
title('ARX por minimos cuadrados: ajuste segun el orden del modelo');

figure('Name','ARX | Validacion temporal con datos experimentales no utilizados');
compare(zTest,modeloElegido,Gder_d); grid on;
title(sprintf('ARX (orden %d): velocidad medida vs. modelos - prueba final',ordenElegido));

figure('Name','ARX | Respuesta en frecuencia de la planta u a velocidad');
bode(Gder_d,modeloElegido); grid on;
legend('Modelo lineal teorico de orden 3', ...
       sprintf('ARX experimental de orden %d',ordenElegido));

figure('Name','ARX | Analisis de residuos en prueba final');
resid(zTest,modeloElegido);
sgtitle(sprintf('ARX (orden %d): residuos sobre datos de prueba',ordenElegido));

%% 7) COMPARACION ADICIONAL: ARX DE ORDEN 3, 5 Y 6
% Analisis posterior a la seleccion. No modifica el orden elegido.
ordenesComparar=[3 5 6];
indicesComparar=zeros(size(ordenesComparar));
for h=1:numel(ordenesComparar)
    indicesComparar(h)=find(ordenes==ordenesComparar(h),1);
end

fprintf('\n========================================\n');
fprintf('COMPARACION ADICIONAL ARX - PRUEBA FINAL\n');
if all(estable(indicesComparar))
    modelosComparar=modelos(indicesComparar);
    fitComparar=nan(size(ordenesComparar));
    for h=1:numel(ordenesComparar)
        fitComparar(h)=ajuste_sim(zTest,modelosComparar{h});
        fprintf('Orden %d: ajuste de prueba = %.2f %%\n', ...
            ordenesComparar(h),fitComparar(h));
    end
    fprintf('Diferencia orden 5 - orden 3 = %.2f puntos porcentuales\n', ...
        fitComparar(2)-fitComparar(1));
    fprintf('Diferencia orden 6 - orden 5 = %.2f puntos porcentuales\n', ...
        fitComparar(3)-fitComparar(2));

    figure('Name','ARX | Comparacion temporal de ordenes 3, 5 y 6 - prueba final');
    compare(zTest,modelosComparar{:}); grid on;
    title('ARX por minimos cuadrados: velocidad medida y modelos de orden 3, 5 y 6');

    figure('Name','ARX | Bode teorico y modelos experimentales de orden 3, 5 y 6');
    bode(Gder_d,modelosComparar{:}); grid on;
    legend('Modelo lineal teorico de orden 3', ...
           'ARX experimental de orden 3', ...
           'ARX experimental de orden 5', ...
           'ARX experimental de orden 6');
else
    warning('No se comparan ordenes 3, 5 y 6: al menos uno es inestable.');
end

%% 8) SENALES EXPERIMENTALES DE PRUEBA
te=(0:numel(i3)-1)'*Ts;
figure('Name','ARX | Entrada, velocidad y posicion medidas - prueba final');
subplot(3,1,1); plot(te,u(i3)); grid on;
ylabel('u [rad]'); title('Entrada: accion de control aplicada por el Wemos');
subplot(3,1,2); plot(te,v(i3)); grid on;
ylabel('v [m/s]'); title('Salida identificada: velocidad filtrada del carro');
subplot(3,1,3); plot(te,q2(i3)); grid on;
ylabel('q2 [m]'); xlabel('Tiempo desde el inicio de la prueba [s]');
title('Posicion medida del carro (solo referencia; no se identifica)');


%% IDENTIFICACION PARAMETRICA BALL & BEAM - VARIABLES INSTRUMENTALES EXPLICITAS
% Inspirado en el ejemplo de IV en lazo cerrado del profesor.
% Datos experimentales: entrada u aplicada [rad], salida velocidad filtrada [m/s].
% Se comparan ordenes 2,3,4,5,6 y dos familias de instrumentos:
% (1) retardos de referencia r; (2) regresores simulados con modelo auxiliar LS.
% No se simulan datos con ode45 ni se considera el integrador de posicion.
% NOTA: los instrumentos basados en referencia requieren excitacion suficiente;
% la independencia del ruido y la validez de los instrumentos deben evaluarse.

%% 3) REGISTROS: AUXILIAR, IV, SELECCION Y PRUEBA
% Del 60%% inicial, 30%% para LS auxiliar y 30%% para IV.
% El 20%% siguiente selecciona el orden; el 20%% final queda para prueba.
na=floor(.30*N); ni=floor(.60*N); ns=floor(.80*N);
iAux=1:na; iIV=(na+1):ni; iSel=(ni+1):ns; iTest=(ns+1):N;
zSel=iddata(v(iSel),u(iSel),Ts);
zTest=iddata(v(iTest),u(iTest),Ts);
fprintf('Aux LS=%d | IV=%d | seleccion=%d | prueba=%d\n', ...
    numel(iAux),numel(iIV),numel(iSel),numel(iTest));

%% 4) IDENTIFICACION POR ORDEN: LS AUXILIAR E IV EXPLICITO
% theta=[-a1 ... -ana b1 ... b_nb]'; estructura [na nb nk]=[n n+1 1].
% Y(k)=Phi(k,:)*theta+e(k).
metodos={'IV_referencia','IV_modelo_auxiliar'};
modelos=cell(numel(ordenes),2);
fitSel=nan(numel(ordenes),2);
maxPolo=nan(numel(ordenes),2);
estable=false(numel(ordenes),2);
condInstrumental=nan(numel(ordenes),2);
for j=1:numel(ordenes)
    n=ordenes(j); nb=n+1; nk=1; p=n+nb;
    fprintf('\n========== ORDEN %d | [%d %d %d] ==========\n',n,n,nb,nk);

    % LS auxiliar con el primer segmento del registro.
    [Yaux,PhiAux]=regresores_arx(v(iAux),u(iAux),n,nb,nk);
    thetaAux=PhiAux\Yaux;
    fprintf('Parametros LS auxiliar: '); fprintf('%.5g ',thetaAux); fprintf('\n');

    % Matrices de regresores del segmento IV.
    vIV=v(iIV); uIV=u(iIV); rIV=r(iIV);
    k0=max(n,nb+nk-1)+1;
    kk=(k0:numel(vIV))';
    [Y,Phi]=regresores_arx(vIV,uIV,n,nb,nk);

    % Variante 1: p retardos de la referencia como instrumentos.
    % Zref(k,:)=[r(k-1),...,r(k-p)].
    % Se descartan muestras iniciales para disponer de todos los retardos.
    kref=max(k0,p+1);
    filas=(kref:numel(vIV))';
    [Yref,Phiref]=regresores_arx_filas(vIV,uIV,n,nb,nk,filas);
    Zref=zeros(numel(filas),p);
    for h=1:p
        Zref(:,h)=rIV(filas-h);
    end
    [thetaRef,condRef]=resolver_iv(Zref,Phiref,Yref);
    condInstrumental(j,1)=condRef;

    % Variante 2: instrumentos de salida simulada sin ruido del LS auxiliar.
    % La entrada experimental u se conserva; no se reconstruye el controlador
    % porque no se conoce con certeza su implementacion completa.
    Aaux=[1;-thetaAux(1:n)];
    Baux=[zeros(nk,1);thetaAux(n+1:end)];
    ySim=filter(Baux,Aaux,uIV);
    Zmod=zeros(numel(kk),p);
    for h=1:n
        Zmod(:,h)=ySim(kk-h);
    end
    for h=1:nb
        Zmod(:,n+h)=uIV(kk-nk-h+1);
    end
    [thetaMod,condMod]=resolver_iv(Zmod,Phi,Y);
    condInstrumental(j,2)=condMod;

    thetas={thetaRef,thetaMod};
    for h=1:2
        th=thetas{h};
        if any(~isfinite(th))
            warning('%s orden %d: matriz instrumental singular o mal condicionada.',metodos{h},n);
            continue;
        end
        A=[1;-th(1:n)]; B=[zeros(nk,1);th(n+1:end)];
        modelos{j,h}=idpoly(A.',B.',1,1,1,[],Ts);
        pp=pole(modelos{j,h});
        maxPolo(j,h)=max(abs(pp));
        estable(j,h)=isstable(modelos{j,h});
        fprintf('%s: cond(Z''Phi)=%.3g | max|p|=%.5f | estable=%d\n', ...
            metodos{h},condInstrumental(j,h),maxPolo(j,h),estable(j,h));
        if estable(j,h)
            fitSel(j,h)=ajuste_sim(zSel,modelos{j,h});
            fprintf('  Fit seleccion: %.2f %%\n',fitSel(j,h));
        end
    end
end

%% 5) TABLA Y SELECCION (SOLO CONJUNTO DE SELECCION)
resultado=table(ordenes(:),estable(:,1),estable(:,2), ...
    fitSel(:,1),fitSel(:,2),condInstrumental(:,1),condInstrumental(:,2), ...
    'VariableNames',{'Orden','Estable_IVref','Estable_IVmodelo', ...
    'FitSel_IVref','FitSel_IVmodelo','Cond_IVref','Cond_IVmodelo'});
disp(resultado);
fitValidos=fitSel; fitValidos(~isfinite(fitValidos))=-Inf;
[mejorFit,ind]=max(fitValidos(:));
if ~isfinite(mejorFit)
    error('Ninguna estimacion IV estable obtuvo un fit de seleccion valido.');
end
[jMejor,hMejor]=ind2sub(size(fitValidos),ind);
modeloElegido=modelos{jMejor,hMejor};
fitTest=ajuste_sim(zTest,modeloElegido);
fprintf('\nSELECCIONADO: %s, orden %d | seleccion %.2f %% | prueba %.2f %%\n', ...
    metodos{hMejor},ordenes(jMejor),mejorFit,fitTest);

%% 6) VALIDACION TEMPORAL, FRECUENCIAL Y RESIDUOS

%% DIAGNOSTICO DE AJUSTES IV

disp('========================================');
disp('AJUSTES DE SELECCION POR ORDEN');

tablaAjustes = table(ordenes(:), ...
    fitSel(:,1), fitSel(:,2), ...
    'VariableNames', ...
    {'Orden','IV_Referencia','IV_ModeloAuxiliar'});

disp(tablaAjustes);

figure('Name','IV | Ajuste de seleccion por orden e instrumentos');
plot(ordenes,fitSel(:,1),'o-',ordenes,fitSel(:,2),'s-','LineWidth',1.5);
grid on; xlabel('Orden n'); ylabel('Fit seleccion [%]');
legend('IV: retardos de referencia','IV: modelo auxiliar','Location','best');
title('Variables instrumentales: seleccion del orden y tipo de instrumento');

figure('Name','IV | Validacion temporal - prueba final');
compare(zTest,modeloElegido,Gder_d); grid on;
title(sprintf('%s orden %d: velocidad medida y simulada - prueba', ...
    metodos{hMejor},ordenes(jMejor)));

figure('Name','IV | Respuesta en frecuencia');
bode(Gder_d,modeloElegido); grid on;
legend('Modelo teorico de velocidad',sprintf('%s orden %d',metodos{hMejor},ordenes(jMejor)));

figure('Name','IV | Residuos - prueba final');
resid(zTest,modeloElegido);
sgtitle(sprintf('Residuos: %s orden %d',metodos{hMejor},ordenes(jMejor)));

%% 7) COMPARACION ADICIONAL DE ORDENES 3, 5 Y 6
% Comparacion diagnostica sobre prueba; NO se usa para seleccionar el modelo.
ordenesComparar=[3 5 6];
for h=1:2
    inds=find(ismember(ordenes,ordenesComparar) & estable(:,h).');
    if isempty(inds), continue; end
    modelosComp=modelos(inds,h);
    fprintf('\n%s - comparacion en prueba:\n',metodos{h});
    for a=1:numel(inds)
        fprintf('  Orden %d: %.2f %%\n',ordenes(inds(a)), ...
            ajuste_sim(zTest,modelosComp{a}));
    end
    figure('Name',['IV | Comparacion temporal - ' metodos{h}]);
    compare(zTest,modelosComp{:}); grid on;
    title(['Prueba final: ordenes 3, 5 y 6 estables - ' metodos{h}]);
    figure('Name',['IV | Comparacion Bode - ' metodos{h}]);
    bode(Gder_d,modelosComp{:}); grid on;
    nombres=[{'Modelo teorico'},arrayfun(@(x)sprintf('IV orden %d',x), ...
        ordenes(inds),'UniformOutput',false)];
    legend(nombres{:});
end

%% 8) SENALES EXPERIMENTALES DE PRUEBA
te=(0:numel(iTest)-1)'*Ts;
figure('Name','IV | Senales experimentales de prueba');
subplot(3,1,1); plot(te,u(iTest)); grid on;
ylabel('u [rad]'); title('Entrada: accion aplicada por el Wemos');
subplot(3,1,2); plot(te,v(iTest)); grid on;
ylabel('v [m/s]'); title('Salida: velocidad filtrada del carro');
subplot(3,1,3); plot(te,q2(iTest)); grid on;
ylabel('q2 [m]'); xlabel('Tiempo de prueba [s]');
title('Posicion medida (solo referencia)');


%% IDENTIFICACION BALL & BEAM - ARX DE ORDEN ALTO Y REDUCCION
% Objetivo: comparar ARX de orden 3, ARX de orden 6 y orden 6 reducido a 3.

%% 3) DATOS DE IDENTIFICACION, SELECCION Y PRUEBA
k1=1:floor(.60*N);
k2=(k1(end)+1):floor(.80*N);
k3=(k2(end)+1):N;
zId=iddata(v(k1),u(k1),Ts);
zSel=iddata(v(k2),u(k2),Ts);
zTest=iddata(v(k3),u(k3),Ts);
fprintf('Identificacion=%d | Seleccion=%d | Prueba=%d\n', ...
    numel(k1),numel(k2),numel(k3));

%% 4) IDENTIFICACION ARX DIRECTA Y DE ORDEN ALTO
% Misma familia estructural que en los scripts previos: [n n+1 1].
modeloDirecto=arx(zId,[ordenFisico ordenFisico+1 1]);
modeloAlto=arx(zId,[ordenAlto ordenAlto+1 1]);
disp('ARX DIRECTO DE ORDEN 3'); present(modeloDirecto);
disp('ARX DE ORDEN ALTO (6)'); present(modeloAlto);
assert(isstable(modeloAlto), ...
    'El ARX alto es inestable: no aplicar reduccion balanceada estable.');

%% 5) REALIZACION BALANCEADA Y REDUCCION A ORDEN 3
% Se reduce la funcion de transferencia deterministica B/A, no el modelo de ruido.
sysAlto=ss(tf(modeloAlto));
[Ab,Bb,Cb,Db]=ssdata(sysAlto);
[Abal,Bbal,Cbal,hsv]=dbalreal(Ab,Bb,Cb);
fprintf('\nValores singulares de Hankel (modelo alto):\n');
disp(hsv(:)');

% Equivalente moderno a eliminar estados con dmodred, manteniendo orden 3.
% balred realiza la reduccion balanceada sobre el modelo discreto.
sysReducido=balred(sysAlto,ordenFisico);
modeloReducido=idss(sysReducido);
modeloReducido.InputName={'u'};
modeloReducido.OutputName={'v'};

fprintf('\nOrden dinamico de la realizacion alta: %d\n',order(sysAlto));
fprintf('Orden dinamico reducido: %d\n',order(sysReducido));

%% 6) ESTABILIDAD Y SELECCION (SIN UTILIZAR LA PRUEBA)
modelos={modeloDirecto,modeloAlto,modeloReducido};
nombres={'ARX directo 3','ARX alto 6','ARX 6 reducido a 3'};
fitSel=nan(3,1); estable=false(3,1);
for j=1:3
    estable(j)=isstable(modelos{j});
    if estable(j)
        fitSel(j)=ajuste_sim(zSel,modelos{j});
    end
    fprintf('%s | estable=%d | fit seleccion=%.2f %%\n', ...
        nombres{j},estable(j),fitSel(j));
end
[mejorFit,idx]=max(fitSel);
assert(isfinite(mejorFit),'Ningun candidato estable con fit valido.');
fprintf('\nMEJOR EN SELECCION: %s (%.2f %%)\n',nombres{idx},mejorFit);

%% 7) VALIDACION TEMPORAL INDEPENDIENTE
% Se informan los tres ajustes de prueba SOLO como comparacion posterior.
fitTest=nan(3,1);
for j=1:3
    if estable(j)
        fitTest(j)=ajuste_sim(zTest,modelos{j});
    end
end
resumen=table(nombres(:),estable,fitSel,fitTest, ...
    'VariableNames',{'Modelo','Estable','FitSeleccion','FitPrueba'});
disp(resumen);

figure('Name','ARX | Comparacion temporal - prueba final');
compare(zTest,modeloDirecto,modeloAlto,modeloReducido,Inf);
grid on;
title('Velocidad medida vs ARX directo 3, ARX 6 y ARX 6 reducido a 3');

%% 8) COMPARACION FRECUENCIAL Y POLOS
figure('Name','ARX | Bode: teorico, directo, alto y reducido');
bode(Gder_d,tf(modeloDirecto),tf(modeloAlto),sysReducido); grid on;
legend('Lineal teorico','ARX directo 3','ARX alto 6','ARX 6 reducido a 3');

figure('Name','ARX | Polos y ceros');
pzmap(Gder_d,tf(modeloDirecto),tf(modeloAlto),sysReducido); grid on;
legend('Lineal teorico','ARX directo 3','ARX alto 6','ARX 6 reducido a 3');

figure('Name','ARX | Valores singulares de Hankel');
semilogy(1:numel(hsv),hsv,'o-','LineWidth',1.5); grid on;
xlabel('Indice del estado balanceado'); ylabel('Valor singular de Hankel');
title('ARX de orden alto: importancia relativa de los estados');

%% 9) ANALISIS DE RESIDUOS DEL MODELO ELEGIDO
% El analisis de residuos de modelos ARX se realiza sobre el modelo ARX
% seleccionado, cuando corresponda. Para el modelo reducido, la conversion
% a idss no preserva necesariamente el modelo de ruido ARX original.
if idx<=2
    figure('Name','ARX | Residuos del modelo seleccionado');
    resid(zTest,modelos{idx});
    sgtitle(['Residuos sobre prueba final: ' nombres{idx}]);
else
    fprintf(['\nEl reducido fue seleccionado: no se ejecuta resid automaticamente, ' ...
        'porque se redujo solo la dinamica B/A y no el modelo de ruido.\n']);
end


%% FUNCIONES LOCALES - NO AGREGAR SCRIPT DEBAJO

function f=ajuste_sim(datos,modelo)
    % Fit de simulacion libre sobre datos reservados.
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

function [Y,Phi]=regresores_arx(y,u,na,nb,nk)
    k0=max(na,nb+nk-1)+1;
    [Y,Phi]=regresores_arx_filas(y,u,na,nb,nk,(k0:numel(y))');
end

function [Y,Phi]=regresores_arx_filas(y,u,na,nb,nk,filas)
    Y=y(filas); Phi=zeros(numel(filas),na+nb);
    for a=1:na
        Phi(:,a)=y(filas-a);
    end
    for b=1:nb
        Phi(:,na+b)=u(filas-nk-b+1);
    end
end

function [theta,c]=resolver_iv(Z,Phi,Y)
    M=Z'*Phi; c=cond(M);
    if ~isfinite(c) || rcond(M)<1e-12
        theta=nan(size(Phi,2),1);
    else
        theta=M\(Z'*Y);
    end
end

