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
%% Barrido de beta: mismo modelo, factorizacion y filtro de orden 2
betas=[.5 1 1.5 2 3 4 6 8];
t=(0:Ts:60)'; ref=.1*(t>=.5); du=deg2rad(1)*(t>=5 & t<6);
plantas={Gm,Gd}; etiquetas={'Identificado','Nominal ZOH'};
nb=numel(betas); rho=nan(nb,2); estableLC=false(nb,2);
UM=nan(nb,2); URMS=UM; Mp=UM; TE=UM; RMSE=UM; Pert=UM; UconPert=UM;
Y=nan(numel(t),nb,2); U=Y; D=Y; Objetivos=nan(numel(t),nb); controladores=cell(nb,1);
for i=1:nb
 beta=betas(i); a=exp(-Ts/beta); F=((1-a)*z/(z-a))^2;
 Q=minreal(F/Gminus,tol); Tobj=minreal(Gp*F,tol);
 if ~isproper(Q)||~isstable(Q), warning('Q invalido para beta=%g',beta); continue; end
 Cd=minreal(Q/(1-Tobj),1e-5);
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
end
for j=1:2
 fprintf('\nRESULTADOS: %s\n',etiquetas{j});
 disp(table(betas(:),rho(:,j),estableLC(:,j),UM(:,j),UconPert(:,j),URMS(:,j),Mp(:,j),TE(:,j),RMSE(:,j),Pert(:,j),...
 'VariableNames',{'Beta_s','RadioLC','Estable','Umax_deg','UmaxPert_deg','Urms_deg','Mp_pct','Te_s','RMSE_m','Pert_mm'}));
end
% Referencia comparativa, no limite fisico certificado.
uReferencia=5.6975;
figure; plot(betas,UM,'-o','LineWidth',1.4); hold on; yline(uReferencia,'k--','Referencia IFT'); set(gca,'YScale','log'); grid on; xlabel('Beta [s]'); ylabel('Control maximo [grados]'); legend(etiquetas{:},'Location','best');
figure; plot(betas,TE,'-o','LineWidth',1.4); grid on; xlabel('Beta [s]'); ylabel('Tiempo de establecimiento [s]'); legend(etiquetas{:});
figure; plot(betas,Pert,'-o','LineWidth',1.4); grid on; xlabel('Beta [s]'); ylabel('Desviacion maxima [mm]'); legend(etiquetas{:});
leyendas=arrayfun(@(b)sprintf('Beta = %g s',b),betas,'UniformOutput',false);
figure; plot(t,ref,'k--',t,Y(:,:,2),'LineWidth',1.2); grid on; xlabel('Tiempo [s]'); ylabel('Posicion [m]'); legend([{'Referencia'},leyendas],'Location','best'); title('Seguimiento nominal ZOH');
figure; plot(t,rad2deg(U(:,:,2)),'LineWidth',1.2); grid on; xlabel('Tiempo [s]'); ylabel('Control [grados]'); legend(leyendas,'Location','best'); xlim([0 10]); title('Accion de control nominal');
figure; plot(t,D(:,:,2)*1000,'LineWidth',1.2); grid on; xlabel('Tiempo [s]'); ylabel('Desviacion [mm]'); legend(leyendas,'Location','best'); title('Rechazo de perturbacion nominal');
validos=find(all(estableLC,2)&all(isfinite(TE),2)&all(UM<=uReferencia,2)&all(UconPert<=uReferencia,2));
if isempty(validos)
 disp('Ninguna configuracion cumple la referencia comparativa de control y establecimiento.');
else
 [~,k]=min(max(TE(validos,:),[],2)); fprintf('\nCandidato mas rapido bajo referencia comparativa: beta=%g s\n',betas(validos(k)));
end
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
