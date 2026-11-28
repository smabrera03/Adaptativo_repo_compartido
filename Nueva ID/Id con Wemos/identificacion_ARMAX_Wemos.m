%% IDENTIFICACION ARMAX DE BAJO ORDEN - WEMOS / BALL & BEAM
% Requiere System Identification Toolbox y Control System Toolbox.
% Estima Delta y = y(k)-y(k-1), preservando un integrador en posicion.
% El ensayo largo se divide cronologicamente 70/30; los escalones no se
% usan para estimar ni seleccionar el modelo.
clear; close all; clc;

%% 1. Cargar ensayos
base = fileparts(mfilename('fullpath'));
if isempty(base), base = pwd; end
f = fullfile(base,'datos_cart_beam(1).mat');
if ~isfile(f), f = fullfile(base,'datos_cart_beam.mat'); end
D = {leer(f), leer(fullfile(base,'datos_escalon.mat')), ...
    leer(fullfile(base,'datos_escalon_v2.mat'))};
nombresEnsayo = {'Ensayo largo','Escalon 1','Escalon 2'};
Ts = median(diff(D{1}.t));
Nid = floor(0.7*numel(D{1}.y));
fprintf('Ts = %.8f s | Nid = %d | Nval = %d\n',Ts,Nid,numel(D{1}.y)-Nid);
for j=1:numel(D)
    dt = diff(D{j}.t);
    fprintf('%s: N=%d, duracion=%.3f s, jitter max=%.4f ms\n', ...
        nombresEnsayo{j},numel(D{j}.y),D{j}.t(end)-D{j}.t(1),1000*max(abs(dt-Ts)));
    assert(all(dt>0) && max(abs(dt-Ts))<0.01*Ts, ...
        'Muestreo irregular: revisar/remuestrear antes de usar iddata.');
end

%% 2. Visualizar telemetria
figure('Name','Telemetria experimental');
subplot(3,1,1); plot(D{1}.t,[D{1}.r D{1}.y]); grid on;
ylabel('Posicion [m]'); legend('r','q2');
subplot(3,1,2); plot(D{1}.t,[D{1}.u D{1}.q1]); grid on;
ylabel('Angulo [rad]'); legend('u reportada','q1 medida');
subplot(3,1,3); plot(D{1}.t,D{1}.v); grid on;
ylabel('Velocidad [m/s]'); xlabel('Tiempo [s]');

%% 3. Modelos candidatos: orden [na nb nc nk] sobre Delta y
% A_delta(q^-1) Delta y = B(q^-1) entrada + C(q^-1) e
% A_pos = (1-q^-1) A_delta. B incluye nk ceros al principio.
% Conservar el mismo orden para comparar la entrada fisica y la reportada.
config = {'q1', [1 1 1 1]; 'q1', [1 1 1 2]; 'q1', [1 2 1 1]; ...
          'u',  [1 1 1 1]; 'u',  [1 1 1 2]; 'u',  [1 2 1 1]};
M = size(config,1);
modelos = struct([]); FIT = nan(M,3); FITpred = nan(M,1);
RMSE = nan(M,3); estable = false(M,1); minfase = false(M,1);
etiquetas = strings(M,1);
y = D{1}.y;
% La primera diferencia se toma cero por convencion de inicializacion.
dy = [0; diff(y)];
for i=1:M
    entrada = config{i,1}; orden = config{i,2};
    x = D{1}.(entrada);
    z_id = iddata(dy(1:Nid),x(1:Nid),Ts);
    z_id.InputName = {entrada}; z_id.OutputName = {'Delta posicion'};
    md = armax(z_id,orden);
    A = conv([1 -1],md.A); B = md.B; C = md.C;
    modelos(i).delta = md;
    modelos(i).A = A; modelos(i).B = B; modelos(i).C = C;
    modelos(i).entrada = entrada; modelos(i).orden = orden;
    modelos(i).G = tf(B,A,Ts,'Variable','z^-1');
    etiquetas(i) = sprintf('%s [%d %d %d %d]',entrada,orden);
    estable(i) = all(abs(roots(md.A))<1); % excluye integrador intencional
    ib = find(abs(B)>1e-12,1,'first');
    if ~isempty(ib), minfase(i) = all(abs(roots(B(ib:end)))<1); end
    fprintf('\n%s: A=%s | B=%s | C=%s\n',etiquetas(i), ...
        mat2str(A,7),mat2str(B,7),mat2str(C,7));
    % Prediccion a un paso (usa medidas previas, no es simulacion libre).
    e = filter(1,C,filter(A,1,y)-filter(B,1,x));
    yp = y-e;
    FITpred(i) = ajuste(y(Nid+1:end),yp(Nid+1:end));
    for j=1:3
        if j==1, k0=Nid+1; else, k0=4; end
        % Los 3 primeros puntos del ensayo independiente inicializan
        % la simulacion. No se corrige con medidas despues de k0.
        ysim = simular(A,B,D{j}.y,D{j}.(entrada),k0);
        modelos(i).ysim{j} = ysim;
        err = D{j}.y(k0:end)-ysim(k0:end);
        FIT(i,j) = ajuste(D{j}.y(k0:end),ysim(k0:end));
        RMSE(i,j) = sqrt(mean(err.^2));
    end
end

%% 4. Resultados: seleccionar por validacion del ensayo largo
resultados = table(etiquetas,FITpred,FIT(:,1),FIT(:,2),FIT(:,3), ...
    RMSE(:,1),RMSE(:,2),RMSE(:,3),estable,minfase, ...
    'VariableNames',{'Modelo','FIT_pred_1paso','FIT_val_libre', ...
    'FIT_escalon1','FIT_escalon2','RMSE_val_m','RMSE_esc1_m', ...
    'RMSE_esc2_m','Dinamica_estable','Minima_fase'});
disp(resultados);
% Los escalones son diagnosticos externos; no se usan para escoger orden.
% Se prioriza sencillez: si esta a <=5 puntos FIT del mejor, elegir
% un solo coeficiente B; preferir menor retardo en caso de empate.
seleccion = zeros(1,2); entradas = {'q1','u'};
for j=1:2
    ix = find(strcmp(config(:,1),entradas{j}) & estable & isfinite(FIT(:,1)));
    if isempty(ix), warning('No hay candidato estable para %s',entradas{j}); continue; end
    [~,ii] = max(FIT(ix,1)); mejor = ix(ii);
    simples = ix(cellfun(@(o)o(2)==1,config(ix,2)));
    simples = simples(FIT(simples,1)>=FIT(mejor,1)-5);
    if ~isempty(simples), mejor = simples(1); end
    seleccion(j)=mejor;
    fprintf('Candidato nominal %s: %s (FIT libre %.2f%%)\n', ...
        entradas{j},etiquetas(mejor),FIT(mejor,1));
end

%% 5. Validacion temporal (simulacion libre)
figure('Name','Validacion experimental de posicion');
for j=1:3
    subplot(3,1,j);
    if j==1, k0=Nid+1; else, k0=4; end
    k=k0:numel(D{j}.y);
    plot(D{j}.t(k),D{j}.y(k),'k','LineWidth',1.5); hold on;
    leg={'Posicion medida'};
    for i=seleccion(seleccion>0)
        plot(D{j}.t(k),modelos(i).ysim{j}(k),'LineWidth',1.3);
        leg{end+1}=sprintf('%s (FIT %.1f%%)',etiquetas(i),FIT(i,j)); %#ok<SAGROW>
    end
    title(nombresEnsayo{j}); ylabel('Posicion [m]'); grid on;
    legend(leg,'Location','best');
end
xlabel('Tiempo [s]');

%% 6. Residuos y Bode de los modelos nominales
for i=seleccion(seleccion>0)
    x=D{1}.(modelos(i).entrada);
    z_val=iddata(dy(Nid+1:end),x(Nid+1:end),Ts);
    figure('Name',char("Residuos Delta y - "+etiquetas(i)));
    resid(z_val,modelos(i).delta);
end
if all(seleccion>0)
    figure('Name','Bode - modelos de posicion');
    bode(modelos(seleccion(1)).G,modelos(seleccion(2)).G); grid on;
    legend('q1 -> q2','u -> q2');
end

%% 7. Guardar para informe / futura implementacion del STR
% G es la parte determinista. A, B, C se guardan en potencias de q^-1.
% No usar el candidato u en hardware sin resolver la mala validacion
% independiente y confirmar que u coincide con la consigna manipulada.
save(fullfile(base,'modelos_ARMAX_Wemos.mat'),'modelos','resultados','seleccion','Ts','Nid');
writetable(resultados,fullfile(base,'resultados_ARMAX_Wemos.csv'));

%% Funciones auxiliares
function d=leer(f)
    s=load(f,'datos'); T=s.datos;
    d.t=T.t(:); d.y=T.q2(:); d.q1=T.q1(:); d.u=T.u(:);
    d.v=T.q2_p(:); d.r=T.r(:);
    assert(all(isfinite([d.t d.y d.q1 d.u d.v d.r]),'all'),'Datos no finitos.');
end
function yh=simular(A,B,y,x,k0)
    na=numel(A)-1; nb=numel(B)-1;
    assert(k0>max(na,nb),'Historial inicial insuficiente.');
    yh=y;
    for k=k0:numel(y)
        yh(k)=-A(2:end)*yh(k-1:-1:k-na)+B*x(k:-1:k-nb);
    end
end
function f=ajuste(y,yh)
    den=norm(y-mean(y));
    if den>0, f=100*(1-norm(y-yh)/den); else, f=NaN; end
end
