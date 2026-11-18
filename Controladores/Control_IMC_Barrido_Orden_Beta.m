%% CONTROL IMC BALL & BEAM - autonomo, sin archivos MAT
clearvars; close all; clc; rng(1);
m=.033; b=.15; g=9.8; q0=33.1549; q1=9.736; ks=11.4345;
Ts=.02; Kp=2; sigma_v=2e-5; N=[6000 3000 4000];
beta=.5; s=tf('s'); z=tf('z',Ts); tol=1e-7;
Gv=g*ks/((s+b/m)*(s^2+q1*s+q0));
Gd=c2d(Gv/s,Ts,'zoh'); Id=Ts/(z-1);
%% Datos comunes: primero todas las entradas, luego simulacion y ruido
for j=1:2
 for k=1:3
  entrada{j,k}=.01*sign(randn(N(k),1));
 end
end
for j=1:2
 for k=1:3
  [u{j,k},v{j,k}]=ensayo(entrada{j,k},Ts,m,b,g,q0,q1,ks,Kp,j==2);
 end
end
for j=1:2
 for k=1:3
  v{j,k}=v{j,k}+sigma_v*randn(N(k),1);
  datos{j,k}=iddata(v{j,k},u{j,k},Ts);
 end
end
%% Identificacion y seleccion por fit de simulacion (velocidad)
metodos={'ARX','IV4','ARMAX','BJ'}; lazos={'LA','LC'};
fitSel=nan(8,1); fitTest=nan(8,1); nombres=cell(8,1); Gpos=cell(8,1);
maxCero=nan(8,1); estable=false(8,1);
for h=1:4
 for j=1:2
  i=2*(h-1)+j; nombres{i}=[metodos{h} ' ' lazos{j}];
  try
   d=datos{j,1};
   switch h
    case 1, M=arx(d,[3 4 1]);
    case 2, M=iv4(d,[3 4 1]);
    case 3
     op=armaxOptions; op.SearchOptions.MaxIterations=100;
     M=armax(d,[3 4 2 1],op);
    case 4, M=bj(d,[4 2 2 3 1]);
   end
   V=minreal(tf(M),tol); Gpos{i}=minreal(V*Id,tol);
   estable(i)=isstable(V); maxCero(i)=max([0;abs(zero(Gpos{i}))]);
   if estable(i)
    fitSel(i)=fitvel(V,datos{j,2},Ts);
   end
  catch ME
   warning('%s: %s',nombres{i},ME.message);
  end
 end
end
validos=find(estable & isfinite(fitSel));
if isempty(validos), error('No hay modelos estables de velocidad validos.'); end
[~,k]=max(fitSel(validos)); elegido=validos(k); j=mod(elegido-1,2)+1;
Gm=Gpos{elegido}; fitTest(elegido)=fitvel(minreal(Gm/Id,tol),datos{j,3},Ts);
disp(table(nombres,estable,fitSel,fitTest,maxCero));
fprintf('Modelo seleccionado: %s\n',nombres{elegido});
disp('Modelo de posicion:'); disp(Gm); disp('Ceros:'); disp(zero(Gm));
%% IMC por inversion parcial
% Gplus conserva ceros no invertibles y retardo relativo.
% Se normaliza Gplus(1)=1. Filtro discreto de ganancia DC unitaria.
[num,den]=tfdata(Gm,'v'); num=num(find(num~=0,1):end);
r=length(den)-length(num);
if r<0, error('Planta no causal.'); end
zz=zero(Gm); malos=zz(abs(zz)>=1-1e-6);
if any(abs(malos-1)<1e-6), error('Cero en DC: diseno no aplicable.'); end
B=1;
for k=1:numel(malos), B=B*(z-malos(k))/(1-malos(k)); end
Gp=minreal(B/z^(r+numel(malos)),tol);
% Gminus se obtiene algebraicamente sin invertir ceros no minimos.
% Gm = ganancia * prod(z-ceros)/prod(z-polos).
[zc,pc,kc]=zpkdata(Gm,'v'); buenos=zc(abs(zc)<1-1e-6);
km=kc*prod(1-malos);
Gminus=zpk([buenos(:);zeros(r+numel(malos),1)],pc,real(km),Ts);
Gminus=minreal(Gminus,tol);
%% Barrido conjunto de orden y beta
[BB,NN]=ndgrid([.25 .5 .75 1 1.5 2 3 4 6 8],[2 3 4 5]);
betas=BB(:); ordenes=NN(:);
t=(0:Ts:60)'; ref=.1*(t>=.5); du=deg2rad(1)*(t>=5 & t<6);
plantas={Gm,Gd}; etiquetas={'Identificado','Nominal ZOH'};
nb=numel(betas); rho=nan(nb,2); estableLC=false(nb,2);
UM=nan(nb,2); URMS=UM; Mp=UM; TE=UM; RMSE=UM; Pert=UM; UconPert=UM;
Y=nan(numel(t),nb,2); U=Y; D=Y; Objetivos=nan(numel(t),nb); controladores=cell(nb,1);
for i=1:nb
 beta=betas(i); a=exp(-Ts/beta); n=ordenes(i); F=zpk(zeros(n,1),a*ones(n,1),(1-a)^n,Ts);
 try
 Q=zpk(pc,[buenos(:);zeros(r+numel(malos),1)],1/real(km),Ts)*F; Tobj=ss(Gp)*ss(F);
 if ~isproper(Q)||~isstable(Q), warning('Q invalido para beta=%g',beta); continue; end
 % C = A_m*N_f / (km*B_s*(z^L*D_f-N_+*N_f)).
 % Cancel only the known simple integrator at z=1 analytically.
 Lret=r+numel(malos); Nf=(1-a)^n*[1 zeros(1,n)];
 Df=poly(a*ones(n,1)); Nplus=real(poly(malos))/prod(1-malos);
 d1=conv([1 zeros(1,Lret)],Df); d2=conv(Nplus,Nf);
 dd=d1-[zeros(1,numel(d1)-numel(d2)) d2];
 [dq,resto]=deconv(dd,[1 -1]);
 [~,ii]=min(abs(pc-1));
 if abs(pc(ii)-1)>1e-5 || norm(resto)>1e-8*norm(dd), error('Cancelacion del integrador no verificada.'); end
 pp=pc; pp(ii)=[];
 Cd=zpk([pp(:);zeros(n,1)],[buenos(:);roots(dq)],(1-a)^n/real(km),Ts);
 if ~isproper(Cd), warning('C no causal para beta=%g',beta); continue; end
 controladores{i}=Cd; Objetivos(:,i)=lsim(Tobj,ref,t);
 for j=1:2
  P=ss(plantas{j}); C=ss(Cd); L=feedback(P*C,1);
  rho(i,j)=max(abs(pole(L))); estableLC(i,j)=isstable(L);
  if ~estableLC(i,j), continue; end
  UR=feedback(C,P); GS=feedback(P,C);
  y=lsim(L,ref,t); u=lsim(UR,ref,t); d=lsim(GS,du,t);
  ud=lsim(-C*GS,du,t);
  Y(:,i,j)=y; U(:,i,j)=u; D(:,i,j)=d;
  UM(i,j)=max(abs(rad2deg(u))); UconPert(i,j)=max(abs(rad2deg(u+ud)));
  URMS(i,j)=sqrt(mean(rad2deg(u).^2));
  Mp(i,j)=100*max(0,max(y)-.1)/.1;
  idx=find(t>=.5,1); fuera=find(abs(y(idx:end)-.1)>.002,1,'last');
  if isempty(fuera), TE(i,j)=0; elseif fuera<numel(t)-idx+1, TE(i,j)=t(idx+fuera)-t(idx); end
  RMSE(i,j)=sqrt(mean((y-Objetivos(:,i)).^2)); Pert(i,j)=1000*max(abs(d));
 end
 catch ME
  warning('n=%d beta=%g: %s',n,beta,ME.message);
  estableLC(i,:)=false;
 end
end
for j=1:2
 fprintf('\nRESULTADOS: %s\n',etiquetas{j});
 disp(table(ordenes,betas(:),rho(:,j),estableLC(:,j),UM(:,j),UconPert(:,j),URMS(:,j),Mp(:,j),TE(:,j),RMSE(:,j),Pert(:,j),...
 'VariableNames',{'Orden','Beta_s','RadioLC','Estable','Umax_deg','UmaxPert_deg','Urms_deg','Mp_pct','Te_s','RMSE_m','Pert_mm'}));
end
%% Seleccion comparativa y graficos
uReferencia=5.6975;
validos=find(all(estableLC,2)&all(isfinite(TE),2)&all(UM<=uReferencia,2)&all(UconPert<=uReferencia,2));
if isempty(validos)
 disp('Ningun candidato cumple todos los criterios.');
else
 [~,k]=min(max(TE(validos,:),[],2)); mejor=validos(k);
 fprintf('Candidato: n=%d beta=%g s\n',ordenes(mejor),betas(mejor));
 disp(table(UM(mejor,2),UconPert(mejor,2),TE(mejor,2),Pert(mejor,2),'VariableNames',{'Umax_deg','UmaxPert_deg','Te_s','Pert_mm'}));
 figure; plot(t,ref,'k--',t,Y(:,mejor,2),t,Y(:,mejor,2)+D(:,mejor,2)); grid on; legend('Referencia','Sin perturbacion','Con perturbacion'); xlabel('Tiempo [s]'); ylabel('Posicion [m]');
 figure; plot(t,rad2deg(U(:,mejor,2))); grid on; xlabel('Tiempo [s]'); ylabel('Control [grados]');
end
metricas={UM(:,2),TE(:,2),Pert(:,2)}; etiquetasY={'Control maximo [grados]','Establecimiento [s]','Desviacion [mm]'};
for k=1:3
 figure; hold on;
 for n=unique(ordenes)'
  ii=ordenes==n & estableLC(:,2); plot(betas(ii),metricas{k}(ii),'-o','DisplayName',sprintf('n=%d',n));
 end
 if k==1, set(gca,'YScale','log'); yline(uReferencia,'k--','Referencia IFT','HandleVisibility','off'); end
 grid on; xlabel('Beta [s]'); ylabel(etiquetasY{k}); legend('Location','best');
end
figure; hold on;
for n=unique(ordenes)'
 ii=ordenes==n & all(estableLC,2) & all(isfinite(TE),2); plot(TE(ii,2),UM(ii,2),'o-','DisplayName',sprintf('n=%d',n));
end
set(gca,'YScale','log'); yline(uReferencia,'k--','Referencia IFT','HandleVisibility','off'); grid on; xlabel('Establecimiento [s]'); ylabel('Control maximo [grados]'); legend('Location','best');
%% Funciones locales
function [u,v]=ensayo(e,Ts,m,b,g,q0,q1,ks,Kp,LC)
 t=(0:numel(e)-1)'*Ts;
 if LC
  r=@(tt)interp1(t,e,tt,'linear','extrap');
  accion=@(tt,x)Kp*(r(tt)-x(1));
 else
  r=@(tt)interp1(t,e,tt,'previous','extrap');
  accion=@(tt,x)r(tt);
 end
 f=@(tt,x)[x(2);g*sin(x(3))-b/m*x(2);x(4);-q0*x(3)-q1*x(4)+ks*accion(tt,x)];
 [~,x]=ode45(f,t,zeros(4,1),odeset('MaxStep',Ts/5));
 v=x(:,2); if LC, u=Kp*(e-x(:,1)); else, u=e; end
end
function f=fitvel(V,d,Ts)
 y=d.OutputData; u=d.InputData; t=(0:numel(y)-1)'*Ts;
 yh=lsim(V,u,t); f=100*(1-norm(y-yh)/norm(y-mean(y)));
end
