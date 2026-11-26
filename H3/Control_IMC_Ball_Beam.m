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
a=exp(-Ts/beta); F=((1-a)*z/(z-a))^2;
Q=minreal(F/Gminus,tol); Tobj=minreal(Gp*F,tol);
if ~isproper(Q)||~isstable(Q), error('Q no es estable y causal.'); end
% C equivalente: cancelar solo factores comunes al construir C,
% nunca reducir el lazo cerrado antes de verificar sus estados.
Cd=minreal(Q/(1-Tobj),1e-5);
if ~isproper(Cd), error('Controlador equivalente no causal.'); end
disp('Controlador equivalente:'); disp(Cd);
%% Validacion sobre modelo identificado y planta nominal ZOH
plantas={Gm,Gd}; etiquetas={'Identificado','Nominal ZOH'};
t=(0:Ts:30)'; ref=.1*(t>=.5); du=deg2rad(1)*(t>=5 & t<6);
yobj=lsim(Tobj,ref,t); Y=nan(length(t),2); U=Y; D=Y; Ytotal=Y;
for j=1:2
 P=ss(plantas{j}); C=ss(Cd); L=feedback(P*C,1);
 % Realizacion sin minreal del lazo: incluye estados internos.
 pp=pole(L); fprintf('\n%s: max|pLC|=%.8f, estable=%d\n',etiquetas{j},max(abs(pp)),isstable(L));
 disp(pp);
 if ~isstable(L), warning('Lazo %s inestable; se omite simulacion.',etiquetas{j}); continue; end
 UR=feedback(C,P); GS=feedback(P,C);
 Y(:,j)=lsim(L,ref,t); U(:,j)=lsim(UR,ref,t); D(:,j)=lsim(GS,double(du),t);
 Ytotal(:,j)=Y(:,j)+D(:,j);
 fprintf('RMSE objetivo: %.6g m | max control: %.6g grados | max perturbacion: %.6g m\n',sqrt(mean((Y(:,j)-yobj).^2)),max(abs(rad2deg(U(:,j)))),max(abs(D(:,j))));
end
%% Mapa de polos y ceros de los modelos de posicion

% Obtener polos y ceros
p_BJ = pole(Gm);
z_BJ = zero(Gm);

p_ZOH = pole(Gd);
z_ZOH = zero(Gd);

% Colores
c_BJ = [0 0.4470 0.7410];
c_ZOH = [0.8500 0.3250 0.0980];

fig_pz = figure('Color','w');
hold on;
grid on;
box on;

% Circunferencia unitaria
ang = linspace(0,2*pi,500);

plot(cos(ang),sin(ang),'k--', ...
    'LineWidth',1.2, ...
    'HandleVisibility','off');

% Ejes real e imaginario
xline(0,'k:','HandleVisibility','off');
yline(0,'k:','HandleVisibility','off');

% Polos BJ LC
h1 = plot(real(p_BJ),imag(p_BJ),'x', ...
    'Color',c_BJ, ...
    'MarkerSize',11, ...
    'LineWidth',2);

% Ceros BJ LC
h2 = plot(real(z_BJ),imag(z_BJ),'o', ...
    'Color',c_BJ, ...
    'MarkerSize',10, ...
    'LineWidth',2);

% Polos nominal ZOH
h3 = plot(real(p_ZOH),imag(p_ZOH),'+', ...
    'Color',c_ZOH, ...
    'MarkerSize',9, ...
    'LineWidth',2);

% Ceros nominal ZOH
h4 = plot(real(z_ZOH),imag(z_ZOH),'o', ...
    'Color',c_ZOH, ...
    'MarkerSize',7, ...
    'LineWidth',2);

axis equal;

xlabel('Parte real');
ylabel('Parte imaginaria');

title('Polos y ceros de los modelos de posición');

legend([h1 h2 h3 h4], ...
    {'Polos BJ LC', ...
     'Ceros BJ LC', ...
     'Polos nominal ZOH', ...
     'Ceros nominal ZOH'}, ...
    'Location','best');

set(gca,'FontSize',12);

hold off;

exportgraphics(fig_pz, ...
    'polos_ceros_imc.png', ...
    'Resolution',300);
figure; plot(t,ref,'k--',t,yobj,':',t,Y,'LineWidth',1.4); grid on; xlabel('Tiempo [s]'); ylabel('Posicion [m]'); legend('Referencia','Objetivo IMC',etiquetas{:}); title('Seguimiento IMC');
figure; plot(t,rad2deg(U),'LineWidth',1.4); grid on; xlabel('Tiempo [s]'); ylabel('Control [grados]'); legend(etiquetas); title('Accion de control');
figure; stairs(t,rad2deg(du)); grid on; xlabel('Tiempo [s]'); ylabel('Perturbacion [grados]');
figure; plot(t,D,'LineWidth',1.4); grid on; xlabel('Tiempo [s]'); ylabel('Desviacion [m]'); legend(etiquetas); title('Respuesta a perturbacion de entrada');
figure; plot(t,ref,'k--',t,Ytotal,'LineWidth',1.4); grid on; xlabel('Tiempo [s]'); ylabel('Posicion [m]'); legend('Referencia',etiquetas{:}); title('Seguimiento con perturbacion');

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
