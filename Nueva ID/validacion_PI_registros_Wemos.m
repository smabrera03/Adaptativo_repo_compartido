%% BALL & BEAM - Comparacion con registros experimentales existentes
% NO requiere Wemos conectado. No es una validacion en hardware de IMC/IFT.
% El registro real se compara con simulaciones bajo la MISMA referencia r.
% Solo controladores de la familia PI (Ki=0 permitido); sin derivada.
clear; close all; clc;
base=fileparts(mfilename('fullpath')); if isempty(base), base=pwd; end
out=fullfile(base,'graficos_validacion_PI'); if ~isfolder(out), mkdir(out); end
Ts=.05; p=.790869; b=.01983333; % planta q1 -> y con ganancia negativa
% v=-q1: Delta y(k)=p*Delta y(k-1)+b*v(k-1)
% PI en forma paralela: v=Kp*(e+Ki*Ts*sum(e)), Ki=1/Ti.

%% 1. Ganancias de la familia PI
% Recuperar ajuste IFT existente, sin repetir ni inventar experimentos.
f=fullfile(base,'resultados_IMC_IFT.mat');
if ~isfile(f), f=fullfile(base,'graficos_IMC_IFT','resultados_IMC_IFT.mat'); end
S=load(f,'par','par0','hist','Jhist','Ts','beta','alpha');
assert(abs(S.Ts-Ts)<1e-8,'Periodo de muestreo incompatible');
par0=S.par0(:); parIFT=S.par(:);
% IMC anterior era PD exacto: NO se usa en esta comparacion PI.
% Alternativa SIMC-PI (regla de ajuste basada en IMC) sobre aproximacion
% integradora con retardo efectivo de baja frecuencia Gv(s)~kv*exp(-theta*s)/s.
kv=b/(Ts*(1-p));
theta=Ts/2+Ts*p/(1-p); % expansion de Gv(exp(s*Ts)) a baja frecuencia
beta=double(S.beta); % misma constante de tiempo elegida previamente
KpSIMC=1/(kv*(beta+theta)); TiSIMC=4*(beta+theta);
parIMC=[KpSIMC;1/TiSIMC];
P=[par0,parIMC,parIFT];
names=["Registro Wemos";"PI inicial";"PI-SIMC";"PI-IFT"];
for j=1:3
    Kp=P(1,j); Ki=P(2,j); A=conv([1 -1],[1 -p]);
    if Ki==0, den=A+[0 b*Kp 0];
    else, den=conv(A,[1 -1])+[0 b*Kp*(1+Ts*Ki) -b*Kp 0]; end
    assert(all(abs(roots(den))<1),'PI no estable en modelo nominal');
end
Ganancias=table(names(2:end),P(1,:)',P(2,:)',(P(1,:).*P(2,:))', ...
    'VariableNames',{'Controlador','Kp_rad_m','Ki_1_s','Kipar_rad_m_s'});
fprintf('Aproximacion SIMC: kv=%.6f m/(rad*s), theta=%.6f s, beta=%.3f s\n',kv,theta,beta);
disp(Ganancias);

%% 2. Registros disponibles y simulaciones con la referencia medida
f=fullfile(base,'datos_cart_beam(1).mat');
if ~isfile(f), f=fullfile(base,'datos_cart_beam.mat'); end
files={f,fullfile(base,'datos_escalon.mat'),fullfile(base,'datos_escalon_v2.mat')};
ensayos=["Ensayo largo";"Escalon 1";"Escalon 2"];
res=table(); pasos=table();
for n=1:numel(files)
    Z=load(files{n},'datos'); D=Z.datos;
    t=D.t(:); t=t-t(1); r=D.r(:); yreal=D.q2(:);
    qreal=D.q1(:); ureport=D.u(:); vel=D.q2_p(:);
    assert(all(isfinite([t r yreal qreal ureport vel]),'all'),'Datos no finitos');
    assert(all(diff(t)>0)&&max(abs(diff(t)-Ts))<.01*Ts,'Muestreo irregular');
    Y=zeros(numel(t),4); Q=Y; Y(:,1)=yreal; Q(:,1)=qreal;
    % Misma posicion inicial y aproximacion de velocidad inicial medida.
    % El PI simulado parte de memoria integral nula. No se conoce su
    % prehistoria real. No hay ruido ni saturacion en esta comparacion.
    for j=1:3
        [Y(:,j+1),Q(:,j+1)]=simPI(r,p,b,Ts,P(:,j),yreal(1),Ts*vel(1));
    end
    % Respuesta deseada de primer orden, inicializada en posicion registrada.
    ym=zeros(size(r)); ym(1)=yreal(1);
    for k=2:numel(r), ym(k)=S.alpha*ym(k-1)+(1-S.alpha)*r(k-1); end
    for j=1:4
        y=Y(:,j); q=Q(:,j);
        % Esfuerzo angular: q1 MEDIDA en el registro; q1 IDEAL en simulacion.
        % u reportada se informa por separado; no se presume consigna.
        met=stepMetrics(t,r,y);
        if isempty(met)
            mp=NaN; tsmed=NaN; nset=0; nstep=0;
        else
            mp=mean(met.Sobreimpulso_pct,'omitnan');
            tsmed=mean(met.Ts2_s,'omitnan'); nset=sum(isfinite(met.Ts2_s)); nstep=height(met);
            met=addvars(met,repmat(ensayos(n),nstep,1),repmat(names(j),nstep,1), ...
                'Before',1,'NewVariableNames',{'Ensayo','Controlador'});
            if isempty(pasos), pasos=met; else, pasos=[pasos;met]; end %#ok<AGROW>
        end
        z=table(ensayos(n),names(j),1000*sqrt(mean((y-r).^2)), ...
            1000*sqrt(mean((y-ym).^2)),max(abs(q)),sqrt(mean(q.^2)), ...
            trapz(t,q.^2),mp,tsmed,nset,nstep, ...
            'VariableNames',{'Ensayo','Controlador','RMSE_ref_mm','RMSE_deseada_mm', ...
            'Pico_q1_rad','RMS_q1_rad','Integral_q1_2_rad2_s','Sobreimpulso_medio_pct', ...
            'Ts2_medio_s','Escalones_establecidos','Escalones_observados'});
        if isempty(res), res=z; else, res=[res;z]; end %#ok<AGROW>
    end
    fprintf('\n%s: pico u reportada=%.6f, RMS u reportada=%.6f (unidad segun adquisicion)\n', ...
        ensayos(n),max(abs(ureport)),sqrt(mean(ureport.^2)));
    f=figure('Name',char(ensayos(n)),'Color','w');
    subplot(3,1,1); plot(t,r,'k:',t,ym,'k--','LineWidth',1.3); hold on;
    plot(t,Y,'LineWidth',1.1); grid on; ylabel('Posicion [m]');
    legend(["Referencia";"Deseada";names],'Location','best'); title(ensayos(n));
    subplot(3,1,2); plot(t,1000*(r-Y),'LineWidth',1.1); grid on;
    ylabel('Error [mm]'); legend(names,'Location','best');
    subplot(3,1,3); plot(t,Q,'LineWidth',1.1); hold on; plot(t,ureport,'k--'); grid on;
    ylabel('Angulo [rad]'); xlabel('Tiempo [s]');
    legend([names;"u reportada"],'Location','best');
    print(f,fullfile(out,sprintf('comparacion_registro_%d.png',n)),'-dpng','-r180');
end
fprintf('\nMETRICAS (registro real frente a simulaciones, no ensayos nuevos)\n'); disp(res);
fprintf('\nESCALONES OBSERVADOS (NaN: no se establecio dentro del tramo)\n'); disp(pasos);
writetable(Ganancias,fullfile(out,'ganancias_PI.csv'));
writetable(res,fullfile(out,'metricas_validacion.csv'));
if ~isempty(pasos), writetable(pasos,fullfile(out,'metricas_por_escalon.csv')); end

%% 3. Convergencia del IFT anterior (en simulacion)
it=(0:numel(S.Jhist)-1)';
f=figure('Name','Convergencia IFT','Color','w');
subplot(3,1,1); semilogy(it,S.Jhist,'o-'); grid on; ylabel('J');
subplot(3,1,2); plot(it,S.hist(:,1),'o-'); grid on; ylabel('Kp');
subplot(3,1,3); plot(it,S.hist(:,2),'o-'); grid on; ylabel('Ki [1/s]'); xlabel('Iteracion');
print(f,fullfile(out,'convergencia_IFT_simulada.png'),'-dpng','-r180');
fprintf('Reduccion del costo IFT simulado: %.4f %%\n',100*(1-S.Jhist(end)/S.Jhist(1)));
save(fullfile(out,'resultados_validacion_PI.mat'),'res','pasos','Ganancias','P','kv','theta','beta');

%% Funciones locales
function [y,q]=simPI(r,p,b,Ts,par,y0,dy0)
    y=zeros(size(r)); v=y; y(1)=y0; dy=dy0; I=0;
    for k=1:numel(r)
        if k>1, dy=p*dy+b*v(k-1); y(k)=y(k-1)+dy; end
        e=r(k)-y(k); I=I+Ts*e;
        v(k)=par(1)*(e+par(2)*I);
    end
    q=-v;
end
function T=stepMetrics(t,r,y)
    % Solo saltos visibles DENTRO del registro; no se presupone r anterior.
    % Sobreimpulso en direccion del salto; normalizado por amplitud de r.
    % Establecimiento al 2%: permanecer hasta siguiente salto o fin del
    % registro, con >=0.25 s de observacion restante. No sustituye una
    % prueba con un tramo suficientemente largo.
    idx=find(abs(diff(r))>1e-6)+1;
    m=numel(idx); ini=zeros(m,1); amp=ini; mp=nan(m,1); ts=mp; dur=ini;
    for i=1:m
        k=idx(i); if i<m, kf=idx(i+1)-1; else, kf=numel(r); end
        dr=r(k)-r(k-1); yy=y(k:kf); tt=t(k:kf);
        ini(i)=t(k); amp(i)=dr; dur(i)=tt(end)-tt(1);
        if dur(i)<.25, continue; end % tramo insuficiente
        mp(i)=100*max(0,max(sign(dr)*(yy-r(k))))/abs(dr);
        fuera=find(abs(yy-r(k))>.02*abs(dr),1,'last');
        if isempty(fuera), ks=1; else, ks=fuera+1; end
        if ks<=numel(tt) && tt(end)-tt(ks)>=.25, ts(i)=tt(ks)-tt(1); end
    end
    T=table(ini,amp,dur,mp,ts,'VariableNames',{'Inicio_s','Salto_m','Duracion_s','Sobreimpulso_pct','Ts2_s'});
end
