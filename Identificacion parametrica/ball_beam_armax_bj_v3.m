%% BALL & BEAM V3 - ARX / IV4 / ARMAX / BJ y ruido
% Ensayos simulados; NO son datos experimentales.
% LA: u -> velocidad por diferencia de posicion medida.
% LC: r -> posicion medida. Identificacion directa u->y en LC: sesgada potencialmente.
% Requiere Control System Toolbox y System Identification Toolbox.
clearvars; close all; clc; rng(1);

%% 1. Planta y condiciones
m=0.033; b=0.15; g=9.8; q0=33.1549; q1=9.736; ks=11.4345;
fs=50; Ts=1/fs; Kp=2; amp=0.01;
Nid=6000; Ntune=3000; Ntest=4000;
Ac=[0 1 0 0; 0 -b/m g 0; 0 0 0 1; 0 0 -q0 -q1];
Bc=[0;0;0;ks]; Cc=[1 0 0 0]; Dc=0;
Gd=c2d(ss(Ac,Bc,Cc,Dc),Ts,'zoh');
z=tf('z',Ts); Gvd=minreal((1-z^-1)*tf(Gd)/Ts,1e-7);
Try=feedback(Kp*Gd,1); S=feedback(1,Kp*Gd);
assert(isstable(Try),'El LC teorico no es estable');

%% 2. Tres ensayos independientes: estimacion, seleccion y prueba final
% Entrada de estimacion: binaria aleatoria de banda amplia.
uid=amp*sign(randn(Nid,1));
% Seleccion y prueba: niveles aleatorios con permanencias y suavizado.
utune=referencia_niveles(Ntune,Ts,amp);
utest=referencia_niveles(Ntest,Ts,amp);
tid=(0:Nid-1)'*Ts; ttune=(0:Ntune-1)'*Ts; ttest=(0:Ntest-1)'*Ts;
xid=lsim(Gd,uid,tid); xtune=lsim(Gd,utune,ttune); xtest=lsim(Gd,utest,ttest);
% Misma realizacion normalizada para comparacion justa entre sigmas.
nLAid=randn(Nid,1); nLAtune=randn(Ntune,1); nLAtest=randn(Ntest,1);
nLCid=randn(Nid,1); nLCtune=randn(Ntune,1); nLCtest=randn(Ntest,1);

%% 3. Modelos candidatos
% ARX/IV4: [na nb nk]; ARMAX: [na nb nc nk]; BJ: [nb nc nd nf nk].
% Ordenes exploratorios; no presuponen que un metodo sera superior.
specsLA={ ...
 'ARX [3 4 1]',   'arx',   [3 4 1]; ...
 'ARX [4 4 1]',   'arx',   [4 4 1]; ...
 'IV4 [3 4 1]',   'iv4',   [3 4 1]; ...
 'IV4 [4 4 1]',   'iv4',   [4 4 1]; ...
 'ARMAX [3 4 2 1]','armax',[3 4 2 1]; ...
 'ARMAX [4 4 2 1]','armax',[4 4 2 1]; ...
 'BJ [4 2 2 3 1]','bj',    [4 2 2 3 1]; ...
 'BJ [4 2 2 4 1]','bj',    [4 2 2 4 1]};
specsLC={ ...
 'ARX [4 5 1]',   'arx',   [4 5 1]; ...
 'IV4 [4 5 1]',   'iv4',   [4 5 1]; ...
 'ARMAX [4 5 2 1]','armax',[4 5 2 1]; ...
 'BJ [5 2 2 4 1]','bj',    [5 2 2 4 1]};
sigmas=[0 2e-6 2e-5];
resumen=cell(numel(sigmas),7);

for j=1:numel(sigmas)
    sig=sigmas(j);
    fprintf('\n=========== sigma_y = %.2g m ===========\n',sig);
    fprintf('sigma de ruido de velocidad = %.5g m/s\n',sqrt(2)*sig/Ts);

    %% 4. LA: derivar posicion ruidosa
    yid=xid+sig*nLAid; ytune=xtune+sig*nLAtune; ytest=xtest+sig*nLAtest;
    % v(k)=(y(k)-y(k-1))/Ts, para k=2...N.
    vid=diff(yid)/Ts; vtune=diff(ytune)/Ts; vtest=diff(ytest)/Ts;
    zid=iddata(vid,uid(2:end),Ts);
    ztune=iddata(vtune,utune(2:end),Ts);
    ztest=iddata(vtest,utest(2:end),Ts);
    [mLA,nLA,fitTuneLA]=elegir_modelo(zid,ztune,specsLA,'LA');
    fitTestLA=NaN;
    if ~isempty(mLA)
        fitTestLA=fit_sim(ztest,mLA);
        fprintf('LA seleccionado: %s | ajuste seleccion=%.2f%% | prueba final=%.2f%%\n',nLA,fitTuneLA,fitTestLA);
        figure('Name',sprintf('LA prueba final sigma=%.2g',sig));
        compare(ztest,mLA,Gvd); grid on;
        title(sprintf('LA: prueba final independiente, sigma_y=%.2g',sig));
        figure('Name',sprintf('LA Bode sigma=%.2g',sig));
        bode(Gvd,mLA); grid on; legend('Teorico','Estimado');
        % Integrador impuesto estructuralmente; no implica modelo preciso.
        Gxrec=minreal(Ts*tf(mLA)/(1-z^-1),1e-7);
        figure('Name',sprintf('Planta reconstruida sigma=%.2g',sig));
        bode(Gd,Gxrec); grid on; legend('Planta teorica','Reconstruida desde LA');
        % Residuos: diagnostico adicional, no unico criterio de seleccion.
        figure('Name',sprintf('LA residuos sigma=%.2g',sig));
        resid(ztest,mLA);
    end

    %% 5. LC: el ruido entra en la realimentacion
    ylcid=lsim(Try,uid,tid)+lsim(S,sig*nLCid,tid);
    ylctune=lsim(Try,utune,ttune)+lsim(S,sig*nLCtune,ttune);
    ylctest=lsim(Try,utest,ttest)+lsim(S,sig*nLCtest,ttest);
    ulcid=Kp*(uid-ylcid); ulctest=Kp*(utest-ylctest);
    zlcid=iddata(ylcid,uid,Ts);
    zlctune=iddata(ylctune,utune,Ts);
    zlctest=iddata(ylctest,utest,Ts);
    [mLC,nLC,fitTuneLC]=elegir_modelo(zlcid,zlctune,specsLC,'LC r->y');
    fitTestLC=NaN;
    if ~isempty(mLC)
        fitTestLC=fit_sim(zlctest,mLC);
        fprintf('LC seleccionado: %s | ajuste seleccion=%.2f%% | prueba final=%.2f%%\n',nLC,fitTuneLC,fitTestLC);
        figure('Name',sprintf('LC prueba final sigma=%.2g',sig));
        compare(zlctest,mLC,Try); grid on;
        title(sprintf('LC: prueba final independiente, sigma_y=%.2g',sig));
        figure('Name',sprintf('LC residuos sigma=%.2g',sig));
        resid(zlctest,mLC);
    end

    %% 6. LC u->y: solo exploratorio, la entrada es endogena
    zplanta=iddata(ylcid,ulcid,Ts);
    fprintf('LC u->y: estimacion directa EXPLORATORIA (feedback + ruido).\n');
    try
        mDirecto=arx(zplanta,[4 5 1]);
        fprintf('ARX planta: max|p|=%.5f; estable=%d\n', ...
            max(abs(pole(mDirecto))),isstable(mDirecto));
        if isstable(mDirecto)
            figure('Name',sprintf('Planta directa LC sigma=%.2g',sig));
            bode(Gd,mDirecto); grid on;
            legend('Planta teorica','ARX directo (cautela)');
        end
    catch ME
        fprintf('Planta directa: %s\n',ME.message);
    end

    resumen(j,:)={sig,nLA,fitTuneLA,fitTestLA,nLC,fitTuneLC,fitTestLC};
end

%% 7. Resumen: seleccion y prueba son conjuntos DISTINTOS
fprintf('\n=========== RESUMEN FINAL ===========\n');
for j=1:size(resumen,1)
    fprintf('sigma=%.2g | LA: %-22s tune=%7.2f%% test=%7.2f%% | LC: %-22s tune=%7.2f%% test=%7.2f%%\n', ...
        resumen{j,1},resumen{j,2},resumen{j,3},resumen{j,4}, ...
        resumen{j,5},resumen{j,6},resumen{j,7});
end
fprintf('\nIMPORTANTE: ruido supuesto; no medido en hardware.\n');
fprintf('BJ/ARMAX modelan ruido, pero no garantizan mejor ajuste.\n');
fprintf('Los modelos inestables se descartan antes de simular.\n');
fprintf('La identificacion directa u->y en LC puede estar sesgada por feedback.\n');

%% Funciones locales (MATLAB R2016b o posterior)
function r=referencia_niveles(N,Ts,amp)
    r=zeros(N,1); k=1;
    while k<=N
        dwell=randi([round(2/Ts),round(5/Ts)]);
        fin=min(N,k+dwell-1);
        r(k:fin)=amp*(2*rand-1);
        k=fin+1;
    end
    alpha=exp(-Ts/0.25);
    r=filter(1-alpha,[1 -alpha],r);
end

function [mejor,nombre,mejorFit]=elegir_modelo(zid,zsel,specs,etiqueta)
    mejor=[]; nombre='NINGUNO'; mejorFit=-Inf;
    for ii=1:size(specs,1)
        nm=specs{ii,1}; metodo=specs{ii,2}; ord=specs{ii,3};
        try
            switch metodo
                case 'arx', modelo=arx(zid,ord);
                case 'iv4', modelo=iv4(zid,ord);
                case 'armax', modelo=armax(zid,ord);
                case 'bj', modelo=bj(zid,ord);
            end
            pmax=max(abs(pole(modelo)));
            if ~isstable(modelo)
                fprintf('%-9s %-22s max|p|=%.5f INESTABLE (no simular)\n',etiqueta,nm,pmax);
                continue;
            end
            f=fit_sim(zsel,modelo);
            fprintf('%-9s %-22s max|p|=%.5f fit seleccion=%8.2f%%\n',etiqueta,nm,pmax,f);
            if isfinite(f) && f>mejorFit
                mejor=modelo; nombre=nm; mejorFit=f;
            end
        catch ME
            fprintf('%-9s %-22s ERROR: %s\n',etiqueta,nm,ME.message);
        end
    end
    if isempty(mejor)
        mejorFit=NaN;
        fprintf('%s: ningun modelo estable y validable.\n',etiqueta);
    end
end

function f=fit_sim(z,modelo)
    % compare devuelve respuesta simulada para este conjunto de datos.
    yh=compare(z,modelo);
    real=z.OutputData(:); pred=yh.OutputData(:);
    if numel(real)~=numel(pred) || any(~isfinite(pred))
        f=NaN; return;
    end
    denom=norm(real-mean(real));
    if denom<=eps, f=NaN; else, f=100*(1-norm(real-pred)/denom); end
end
