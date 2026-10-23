%% BALL & BEAM - Diagnostico de ruido, orden y reconstruccion del integrador
% Comparacion reproducible: sin ruido / bajo / actual.
% LA: u -> diferencia de posicion (velocidad numerica).
% LC: r -> posicion medida. Planta u -> y en LC: exploratoria.
% Requiere Control System Toolbox y System Identification Toolbox.
clearvars; close all; clc;
rng(1);

%% 1. Modelo y configuracion
m=0.033; b=0.15; g=9.8;
q0=33.1549; q1=9.736; ks=11.4345;
fs=50; Ts=1/fs; Kp=2;
Nid=6000; Nval=4000; amp=0.01;
tid=(0:Nid-1)'*Ts; tval=(0:Nval-1)'*Ts;
Ac=[0 1 0 0;0 -b/m g 0;0 0 0 1;0 0 -q0 -q1];
Bc=[0;0;0;ks]; Cc=[1 0 0 0]; Dc=0;
Gd=c2d(ss(Ac,Bc,Cc,Dc),Ts,'zoh');
z=tf('z',Ts);
Gvd=minreal((1-z^-1)*tf(Gd)/Ts,1e-7);
Try=feedback(Kp*Gd,1);
S=feedback(1,Kp*Gd);
assert(isstable(Try),'El lazo cerrado teorico no es estable');

%% 2. Dos ensayos independientes, MISMAS senales para todos los ruidos
uid=amp*sign(randn(Nid,1));
uv=zeros(Nval,1); k=1;
while k<=Nval
    dwell=randi([round(2/Ts),round(5/Ts)]);
    kend=min(Nval,k+dwell-1);
    uv(k:kend)=amp*(2*rand-1);
    k=kend+1;
end
alpha=exp(-Ts/0.25);
uv=filter(1-alpha,[1 -alpha],uv);
% LA: posicion simulada. Usamos la MISMA realizacion de ruido base.
xid=lsim(Gd,uid,tid); xv=lsim(Gd,uv,tval);
base_id=randn(Nid,1); base_val=randn(Nval,1);
base_lc_id=randn(Nid,1); base_lc_val=randn(Nval,1);

%% 3. Escenarios y estructuras a contrastar
sigmas=[0,2e-6,2e-5];  % [m], valores supuestos
ordenes_LA={[3 3 1],[3 4 1],[4 4 1]};
ordenes_LC={[4 4 1],[4 5 1]};
resultados=struct([]);

for j=1:numel(sigmas)
    sig=sigmas(j);
    fprintf('\n================ RUIDO sigma_y = %.2g m ================\n',sig);
    yid=xid+sig*base_id; yv=xv+sig*base_val;
    % Derivada numerica: v(k)=(y(k)-y(k-1))/Ts.
    % Los pares v(k),u(k) se etiquetan para k=2..N.
    vid=diff(yid)/Ts; vv=diff(yv)/Ts;
    zid=iddata(vid,uid(2:end),Ts);
    zv=iddata(vv,uv(2:end),Ts);
    fprintf('Ruido teorico de velocidad: %.4g m/s\n',sqrt(2)*sig/Ts);

    % Comparacion de ordenes y estimadores en LA.
    modelos={}; nombres={};
    for o=1:numel(ordenes_LA)
        ord=ordenes_LA{o};
        for met=1:2
            try
                if met==1
                    modelo=arx(zid,ord); metodo='ARX';
                else
                    modelo=iv4(zid,ord); metodo='IV4';
                end
                nombre=sprintf('%s [%d %d %d]',metodo,ord);
                polos=pole(modelo); pmax=max(abs(polos));
                fprintf('LA %-18s | max|p| = %.5f | %s\n',nombre,pmax,etiqueta(modelo));
                modelos{end+1}=modelo; nombres{end+1}=nombre; %#ok<SAGROW>
            catch ME
                fprintf('LA: fallo %s [%d %d %d]: %s\n',metodo,ord,ME.message);
            end
        end
    end
    % Seleccion didactica: primero modelo estable, despues menor error
    % de simulacion sobre datos independientes. Nunca simular inestables.
    mejor=[]; mejorNombre=''; mejorError=Inf;
    for ii=1:numel(modelos)
        if ~isstable(modelos{ii}), continue; end
        try
            [yh,~]=compare(zv,modelos{ii});
            yh=yh.OutputData;
            err=norm(vv(:)-yh(:))/max(norm(vv(:)-mean(vv(:))),eps);
            if isfinite(err) && err<mejorError
                mejorError=err; mejor=modelos{ii}; mejorNombre=nombres{ii};
            end
        catch ME
            fprintf('No se pudo validar %s: %s\n',nombres{ii},ME.message);
        end
    end
    if isempty(mejor)
        fprintf('LA: ningun modelo estable y validable en este escenario.\n');
    else
        fprintf('LA elegido SOLO para diagnostico: %s; fit simulacion %.2f %%\n', ...
            mejorNombre,100*(1-mejorError));
        figure('Name',sprintf('LA validacion sigma=%.2g',sig));
        compare(zv,mejor,Gvd); grid on;
        title(sprintf('LA: validacion independiente, sigma_y=%.2g m',sig));
        figure('Name',sprintf('LA polos sigma=%.2g',sig));
        pzmap(Gvd,mejor); grid on; legend('Teorico','Estimado');
        % Reconstruccion estructural del integrador conocido.
        % Gv=(1-z^-1)/Ts * Gx, por lo tanto Gx=Ts/(1-z^-1)*Gv.
        Gx_recon= minreal(Ts*tf(mejor)/(1-z^-1),1e-7);
        figure('Name',sprintf('Planta reconstruida sigma=%.2g',sig));
        bode(Gd,Gx_recon); grid on;
        legend('Planta teorica','Reconstruida desde LA');
        title('Planta u -> x: integrador impuesto estructuralmente');
        fprintf('Reconstruccion: polo mas cercano a z=1: %.3g\n', ...
            min(abs(pole(Gx_recon)-1)));
    end

    %% LC: ruido dentro del lazo, r -> y
    nid=sig*base_lc_id; nv=sig*base_lc_val;
    ylcid=lsim(Try,uid,tid)+lsim(S,nid,tid);
    ylcv=lsim(Try,uv,tval)+lsim(S,nv,tval);
    ulcid=Kp*(uid-ylcid); ulcv=Kp*(uv-ylcv);
    zlcid=iddata(ylcid,uid,Ts);
    zlcv=iddata(ylcv,uv,Ts);
    fprintf('\nLC r -> y:\n');
    lcModels={}; lcNames={};
    for o=1:numel(ordenes_LC)
        ord=ordenes_LC{o};
        for met=1:2
            try
                if met==1, ml=arx(zlcid,ord); mn='ARX';
                else, ml=iv4(zlcid,ord); mn='IV4'; end
                fprintf('LC %-4s [%d %d %d] | max|p|=%.5f | %s\n', ...
                    mn,ord,max(abs(pole(ml))),etiqueta(ml));
                if isstable(ml)
                    lcModels{end+1}=ml; lcNames{end+1}=sprintf('%s [%d %d %d]',mn,ord); %#ok<SAGROW>
                end
            catch ME
                fprintf('LC: error: %s\n',ME.message);
            end
        end
    end
    if ~isempty(lcModels)
        figure('Name',sprintf('LC validacion sigma=%.2g',sig));
        compare(zlcv,lcModels{1},Try); grid on;
        title(sprintf('LC r -> y: primer modelo estable (%s)',lcNames{1}));
    end

    %% LC: u -> y, estimacion DIRECTA exploratoria
    zpid=iddata(ylcid,ulcid,Ts);
    zpv=iddata(ylcv,ulcv,Ts);
    fprintf('\nPlanta LC u -> y (entrada endogena, interpretar con cautela):\n');
    try
        mp=arx(zpid,[4 4 1]);
        fprintf('ARX planta: max|p|=%.5f | %s\n',max(abs(pole(mp))),etiqueta(mp));
        if isstable(mp)
            figure('Name',sprintf('Planta directa sigma=%.2g',sig));
            bode(Gd,mp); grid on;
            legend('Teorica','ARX directo en LC');
        end
    catch ME
        fprintf('Fallo planta directa: %s\n',ME.message);
    end
    resultados(j).sigma_y=sig; %#ok<SAGROW>
    resultados(j).mejor_LA=mejorNombre;
    resultados(j).fit_LA=100*(1-mejorError);
end

%% 4. Resumen
fprintf('\n========== RESUMEN LA ==========\n');
for j=1:numel(resultados)
    fprintf('sigma=%.2g m | %-20s | fit=%.2f %%\n', ...
        resultados(j).sigma_y,resultados(j).mejor_LA,resultados(j).fit_LA);
end
fprintf('\nNOTAS:\n');
fprintf('- Fit de validacion: simulacion sobre un ensayo independiente.\n');
fprintf('- La seleccion de orden por el conjunto de validacion lo convierte\n');
fprintf('  en conjunto de ajuste de hiperparametros; reservar un tercer ensayo\n');
fprintf('  para una evaluacion final imparcial.\n');
fprintf('- Ruido y amplitudes son supuestos de simulacion, no datos medidos.\n');
fprintf('- Reconstruir un integrador impone la fisica pero NO garantiza\n');
fprintf('  que la dinamica restante este correctamente identificada.\n');
fprintf('- La identificacion u->y en LC puede estar sesgada por feedback.\n');

function txt=etiqueta(modelo)
if isstable(modelo), txt='estable'; else, txt='INESTABLE'; end
end
