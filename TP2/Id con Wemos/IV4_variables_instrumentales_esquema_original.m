%% IDENTIFICACION PARAMETRICA BALL & BEAM - IV4 - DISTINTOS ORDENES
% Mismo esquema de los codigos originales de H2, con datos experimentales.
% Entrada: accion realmente aplicada u_wemos [rad].
% Salida: velocidad del carro [m/s], SIN integrador de posicion.
% Ordenes 2, 3, 4 y 5; el orden fisico esperado es 3.
% Un registro experimental dividido en identificacion, seleccion y prueba.
% Requiere Control System Toolbox y System Identification Toolbox.
% IV4 utiliza variables instrumentales y estructura ARX.
clearvars; close all; clc;

%% 1) PARAMETROS Y REFERENCIA TEORICA
m=0.033; b=0.15; g=9.8;
q0=33.1549; q1=9.736; k=11.4345;
ordenes=[2 3 4 5];  % orden fisico de referencia: 3
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

%% 4) IDENTIFICACION IV4: DISTINTOS ORDENES
% Estructura [na nb nk] = [n n+1 1], como el original [3 4 1].
modelos=cell(numel(ordenes),1);
fitSel=nan(numel(ordenes),1);
estable=false(numel(ordenes),1);
maxPolo=nan(numel(ordenes),1);
for j=1:numel(ordenes)
    n=ordenes(j);
    estructura=[n n+1 1];
    modelos{j}=iv4(zId,estructura);
    p=pole(modelos{j});
    maxPolo(j)=max(abs(p));
    estable(j)=isstable(modelos{j});
    fprintf('\n========================================\n');
    fprintf('IV4 orden %d | estructura [%d %d %d]\n',n,estructura);
    present(modelos{j});
    fprintf('max|p|=%.6f | estable=%d\n',maxPolo(j),estable(j));
    if estable(j)
        fitSel(j)=ajuste_sim(zSel,modelos{j});
        fprintf('Fit seleccion=%.2f %%\n',fitSel(j));
    else
        warning('IV4 orden %d inestable: no se considera para seleccion.',n);
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
fprintf('IV4 SELECCIONADO: orden %d | seleccion %.2f %% | prueba %.2f %%\n', ...
    ordenElegido,mejorFit,fitTest);

% Tabla: la prueba final solo se informa para el modelo elegido.
resultado=table(ordenes(:),estable,maxPolo,fitSel, ...
    'VariableNames',{'Orden','Estable','MaxAbsPolo','FitSeleccion'});
disp(resultado);

%% 6) GRAFICOS DE VALIDACION Y COMPARACION FRECUENCIAL
% Los titulos distinguen seleccion, prueba final y referencia teorica.
figure('Name','IV4 | Seleccion del orden del modelo');
plot(ordenes,fitSel,'o-','LineWidth',1.5); grid on;
xlabel('Orden del modelo n'); ylabel('Ajuste en seleccion (fit) [%]');
title('IV4 por variables instrumentales: ajuste segun el orden del modelo');

figure('Name','IV4 | Validacion temporal con datos experimentales no utilizados');
compare(zTest,modeloElegido,Gder_d); grid on;
title(sprintf('IV4 (orden %d): velocidad medida vs. modelos - prueba final',ordenElegido));

figure('Name','IV4 | Respuesta en frecuencia de la planta u a velocidad');
bode(Gder_d,modeloElegido); grid on;
legend('Modelo lineal teorico de orden 3', ...
       sprintf('IV4 experimental de orden %d',ordenElegido));

figure('Name','IV4 | Analisis de residuos en prueba final');
resid(zTest,modeloElegido);
sgtitle(sprintf('IV4 (orden %d): residuos sobre datos de prueba',ordenElegido));

%% 7) SENALES EXPERIMENTALES DE PRUEBA
te=(0:numel(i3)-1)'*Ts;
figure('Name','IV4 | Entrada, velocidad y posicion medidas - prueba final');
subplot(3,1,1); plot(te,u(i3)); grid on;
ylabel('u [rad]'); title('Entrada: accion de control aplicada por el Wemos');
subplot(3,1,2); plot(te,v(i3)); grid on;
ylabel('v [m/s]'); title('Salida identificada: velocidad filtrada del carro');
subplot(3,1,3); plot(te,q2(i3)); grid on;
ylabel('q2 [m]'); xlabel('Tiempo desde el inicio de la prueba [s]');
title('Posicion medida del carro (solo referencia; no se identifica)');

%% FUNCIONES LOCALES (SIEMPRE AL FINAL DEL SCRIPT)
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
