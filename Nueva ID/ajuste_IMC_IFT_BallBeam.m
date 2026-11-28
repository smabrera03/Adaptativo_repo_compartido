%% BALL & BEAM - Ajuste nominal IMC e IFT
% Simulacion offline, NO se comunica con Wemos. No requiere toolboxes.
% Planta mecanica: q1 (rad) -> q2 (m). Se supone seguimiento angular ideal.
% v = -q1 permite usar ganancias positivas con error r-y.
clear; close all; clc;
Ts = 0.05; p = 0.790869; b = 0.01983333;
A = conv([1 -1],[1 -p]); B = [0 -b]; C = [1 -0.9687558];
fprintf('A = %s | B = %s | C = %s\n',mat2str(A),mat2str(B),mat2str(C));
% C se conserva como informacion de identificacion. No se inyecta ruido
% en esta primera comparacion determinista (ni se conoce su varianza aqui).

%% 1. Modelo de referencia y ensayo de ajuste
beta = 1;                          % s, constante de tiempo deseada
alpha = exp(-Ts/beta);
t = (0:Ts:40-Ts)'; r = zeros(size(t));
r(t>=2 & t<10) = .08; r(t>=18 & t<26) = -.08;
ym = filter([0 1-alpha],[1 -alpha],r);
% Pesos de costo normalizado (hipotesis de diseno, no limites de hardware)
yScale=.08; uScale=.20; lambda=.003; uGuard=.22; yGuard=.16;
Jfun = @(y,v,yd) .5*mean(((y-yd)/yScale).^2+lambda*(v/uScale).^2);

%% 2. IMC: Q=F/G, R_IMC=Q/(1-F)
% Para Gv=b*q^-1/((1-q^-1)*(1-p*q^-1)) y
% F=(1-alpha)*q^-1/(1-alpha*q^-1):
% R_IMC=(1-alpha)/b*(1-p*q^-1), expresado en v=-q1.
kimc=(1-alpha)/b;
[yimc,vimc]=simCL(r,p,b,Ts,'IMC',[kimc p]);
fprintf('\nIMC en q1: q1(k) = %.8f*e(k) + %.8f*e(k-1)\n',-kimc,kimc*p);
fprintf('Error maximo nominal respecto de F: %.3g m\n',max(abs(yimc-ym)));

%% 3. IFT: PI del notebook, con ki = Ts*Ki (Ki en 1/s)
% Rv = Kp*(1+Ts*Ki-q^-1)/(1-q^-1).
% Experimento 1: r. Experimento 2: r-y1, mismos parametros.
% Delta_Kp=1/Kp; Delta_Ki=Ts/(1+Ts*Ki-q^-1).
% Gradiente IFT, SIN derivar el modelo de planta.
% La busqueda de paso usa simulaciones adicionales; no es IFT experimental.
par=[.8;.10]; par0=par; gamma=2; nIter=60;
lo=[.05;0]; hi=[3;2];
[y0,v0]=simCL(r,p,b,Ts,'PI',par0);
Jhist=Jfun(y0,v0,ym); hist=par';
for it=1:nIter
    [y1,v1]=simCL(r,p,b,Ts,'PI',par);
    [y2,v2]=simCL(r-y1,p,b,Ts,'PI',par);
    den=[1+Ts*par(2),-1];
    dY=[y2/par(1),filter(Ts,den,y2)];
    dV=[v2/par(1),filter(Ts,den,v2)];
    grad=(dY'*(y1-ym)/yScale^2 + lambda*dV'*v1/uScale^2)/numel(r);
    Jold=Jfun(y1,v1,ym); accepted=false;
    for ls=0:19
        cand=min(hi,max(lo,par-gamma*2^(-ls)*grad));
        if ~stablePI(p,b,Ts,cand), continue; end
        [yc,vc]=simCL(r,p,b,Ts,'PI',cand);
        Jnew=Jfun(yc,vc,ym);
        if isfinite(Jnew) && Jnew<Jold && max(abs(vc))<=uGuard && max(abs(yc))<=yGuard
            accepted=true; break;
        end
    end
    if ~accepted, break; end
    par=cand; hist(end+1,:)=par'; Jhist(end+1,1)=Jnew; %#ok<SAGROW>
    fprintf('IFT %02d | J=%.8g | Kp=%.6f | Ki=%.6f 1/s\n',it,Jnew,par(1),par(2));
end
[yift,vift]=simCL(r,p,b,Ts,'PI',par);
fprintf('\nIFT final (v=-q1): Kp=%.8f, Ki=%.8f 1/s, ki=%.8f\n',par(1),par(2),Ts*par(2));
fprintf('J inicial %.8g -> J final %.8g\n',Jhist(1),Jhist(end));

%% 4. Comparacion nominal y validacion con referencia distinta
% No son ensayos experimentales nuevos; son simulaciones nominales.
tv=(0:Ts:30-Ts)'; rv=zeros(size(tv));
rv(tv>=2 & tv<8)=.06; rv(tv>=12 & tv<19)=-.06;
rv(tv>=23)=.03;
rm=filter([0 1-alpha],[1 -alpha],rv);
[yv0,vv0]=simCL(rv,p,b,Ts,'PI',par0);
[yvm,vvm]=simCL(rv,p,b,Ts,'IMC',[kimc p]);
[yvf,vvf]=simCL(rv,p,b,Ts,'PI',par);
% Prueba de sensibilidad: dinamica incremental y ganancia perturbadas.
% Se conserva el integrador; el controlador no se vuelve a ajustar.
pp=.85; bp=.85*b;
[ypm,vpm]=simCL(rv,pp,bp,Ts,'IMC',[kimc p]);
[ypf,vpf]=simCL(rv,pp,bp,Ts,'PI',par);
Modelo={'PI inicial';'IMC';'IFT'};
resA=metrics({y0,yimc,yift},{v0,vimc,vift},ym,r,Jfun,Modelo);
resV=metrics({yv0,yvm,yvf},{vv0,vvm,vvf},rm,rv,Jfun,Modelo);
resP=metrics({ypm,ypf},{vpm,vpf},rm,rv,Jfun,{'IMC';'IFT'});
disp('AJUSTE NOMINAL'); disp(resA); disp('VALIDACION NOMINAL'); disp(resV);
disp('SENSIBILIDAD (p=.85, b=85% nominal)'); disp(resP);

%% 5. Graficos
out='graficos_IMC_IFT'; if ~exist(out,'dir'), mkdir(out); end
f=figure('Name','Comparacion nominal');
subplot(2,1,1); plot(t,r,'k:',t,ym,'k--',t,y0,t,yimc,t,yift,'LineWidth',1.3); grid on;
ylabel('Posicion [m]'); legend('r','Deseada','PI inicial','IMC','IFT','Location','best');
subplot(2,1,2); stairs(t,[-v0,-vimc,-vift],'LineWidth',1.2); grid on; ylabel('q1 [rad]'); xlabel('Tiempo [s]'); legend('PI inicial','IMC','IFT');
print(f,fullfile(out,'comparacion_nominal.png'),'-dpng','-r180');
f=figure('Name','Convergencia IFT');
subplot(3,1,1); plot(0:numel(Jhist)-1,Jhist,'o-'); grid on; ylabel('J');
subplot(3,1,2); plot(0:size(hist,1)-1,hist(:,1),'o-'); grid on; ylabel('Kp');
subplot(3,1,3); plot(0:size(hist,1)-1,hist(:,2),'o-'); grid on; ylabel('Ki [1/s]'); xlabel('Iteracion');
print(f,fullfile(out,'convergencia_IFT.png'),'-dpng','-r180');
f=figure('Name','Validacion nominal');
subplot(2,1,1); plot(tv,rv,'k:',tv,rm,'k--',tv,yv0,tv,yvm,tv,yvf,'LineWidth',1.3); grid on; ylabel('Posicion [m]'); legend('r','Deseada','PI inicial','IMC','IFT','Location','best');
subplot(2,1,2); stairs(tv,[-vv0,-vvm,-vvf],'LineWidth',1.2); grid on; ylabel('q1 [rad]'); xlabel('Tiempo [s]'); legend('PI inicial','IMC','IFT');
print(f,fullfile(out,'validacion_nominal.png'),'-dpng','-r180');
f=figure('Name','Sensibilidad paramétrica');
subplot(2,1,1); plot(tv,rv,'k:',tv,rm,'k--',tv,ypm,tv,ypf,'LineWidth',1.3); grid on; ylabel('Posicion [m]'); legend('r','Deseada','IMC','IFT','Location','best');
subplot(2,1,2); stairs(tv,[-vpm,-vpf],'LineWidth',1.2); grid on; ylabel('q1 [rad]'); xlabel('Tiempo [s]'); legend('IMC','IFT');
print(f,fullfile(out,'sensibilidad.png'),'-dpng','-r180');
save(fullfile(out,'resultados_IMC_IFT.mat'),'A','B','C','Ts','beta','alpha','kimc','par','par0','hist','Jhist','resA','resV','resP');

%% Funciones locales
function [y,v]=simCL(r,p,b,Ts,kind,par)
N=numel(r); y=zeros(N,1); v=y; dy=0; ep=0; vp=0;
for k=1:N
    if k>1, dy=p*dy+b*v(k-1); y(k)=y(k-1)+dy; end
    e=r(k)-y(k);
    if strcmp(kind,'PI')
        v(k)=vp+par(1)*((1+Ts*par(2))*e-ep);
    else
        v(k)=par(1)*(e-par(2)*ep);
    end
    ep=e; vp=v(k);
end
end
function ok=stablePI(p,b,Ts,par)
A=conv([1 -1],[1 -p]);
if par(2)==0
    acl=A+[0 b*par(1) 0]; % PI reducido a P
else
    acl=conv(A,[1 -1])+[0 b*par(1)*(1+Ts*par(2)) -b*par(1) 0];
end
ok=all(abs(roots(acl))<1);
end
function T=metrics(Y,V,ym,r,Jfun,names)
n=numel(Y); J=zeros(n,1); RMSE_deseada_mm=J; RMSE_ref_mm=J; pico_rad=J; energia_rad2_muestra=J;
for i=1:n
    J(i)=Jfun(Y{i},V{i},ym); RMSE_deseada_mm(i)=1000*sqrt(mean((Y{i}-ym).^2));
    RMSE_ref_mm(i)=1000*sqrt(mean((Y{i}-r).^2)); pico_rad(i)=max(abs(V{i}));
    energia_rad2_muestra(i)=mean(V{i}.^2); % media cuadratica, no energia fisica
end
T=table(names,J,RMSE_deseada_mm,RMSE_ref_mm,pico_rad,energia_rad2_muestra,'VariableNames',{'Modelo','J','RMSE_deseada_mm','RMSE_ref_mm','Pico_rad','Media_u2_rad2'});
end
