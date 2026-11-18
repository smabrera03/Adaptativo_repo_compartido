%% VALIDACION DEL CONTROLADOR PID
% Ball & Beam - Hito 3
%
% Evaluacion de los resultados experimentales
% obtenidos mediante IFT.

clear; close all; clc;

load('resultados_simulaciones\Ajuste PID.mat');

lw = 1.5;

%% 1) PARAMETROS INICIALES Y FINALES

theta0 = th(:, 1);
thetaf = th(:, end);

kp0 = theta0(1);
ki0 = theta0(2);
kd0 = theta0(3);

kpf = thetaf(1);
kif = thetaf(2);
kdf = thetaf(3);

fprintf('\nCONTROLADOR INICIAL\n');
fprintf('kp = %.8f\n', kp0);
fprintf('ki = %.8f\n', ki0);
fprintf('kd = %.8f\n', kd0);

fprintf('\nCONTROLADOR FINAL\n');
fprintf('kp = %.8f\n', kpf);
fprintf('ki = %.8f\n', kif);
fprintf('kd = %.8f\n', kdf);

%% 2) PREPARACION DE LAS SEÑALES

t = t(:);
ref = ref(:);
ym = ym(:);

y0 = y0(:);
u0 = u0(:);

y_final = y_final(:);
u_final = u_final(:);

N = length(t);

Ts = mean(diff(t));

% Instante de aplicacion del escalon

idx = find(abs(ref-ref(1)) > 1e-9, 1);

t_escalon = t(idx);

% Amplitud del escalon

rf = ref(end);
dr = rf-ref(1);

% Banda de tolerancia del 2 %

tol = 0.02*abs(dr);

% Cantidad de muestras del tramo final
% Se considera el ultimo segundo.

Nf = min(round(1/Ts), N);

%% 3) COMPARACION DE RESPUESTAS

figure;

plot(t, ref, 'k--', 'LineWidth', lw);
hold on;

plot(t, y0, 'LineWidth', lw);
plot(t, y_final, 'LineWidth', lw);
plot(t, ym, 'LineWidth', lw);

yline(rf+tol, 'k:', ...
    'HandleVisibility', 'off');

yline(rf-tol, 'k:', ...
    'HandleVisibility', 'off');

grid on;

xlabel('Tiempo [s]');
ylabel('Posición [m]');

title('Comparación de respuestas experimentales');

legend('Referencia', ...
       'Controlador inicial', ...
       'Controlador final', ...
       'Modelo objetivo', ...
       'Location', 'best');

xlim([t(1) t(end)]);

%% 4) COMPARACION DE ACCIONES DE CONTROL

figure;

plot(t, rad2deg(u0), 'LineWidth', lw);
hold on;

plot(t, rad2deg(u_final), 'LineWidth', lw);

grid on;

xlabel('Tiempo [s]');
ylabel('Acción de control [°]');

title('Comparación de las acciones de control');

legend('Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

xlim([t(1) t(end)]);

%% 5) ERROR DE SEGUIMIENTO

e0 = ref-y0;
efinal = ref-y_final;

figure;

plot(t, e0, 'LineWidth', lw);
hold on;

plot(t, efinal, 'LineWidth', lw);

yline(0, 'k--', 'HandleVisibility', 'off');

grid on;

xlabel('Tiempo [s]');
ylabel('Error de seguimiento [m]');

title('Comparación del error de seguimiento');

legend('Controlador inicial', ...
       'Controlador final', ...
       'Location', 'best');

%% 6) INDICADORES DE DESEMPEÑO

% Cada columna corresponde a un controlador.

Y = [y0, y_final];
U = [u0, u_final];

Mp   = zeros(1, 2);
ts   = NaN(1, 2);
ess  = zeros(1, 2);
ef   = zeros(1, 2);
RMSE = zeros(1, 2);
umax = zeros(1, 2);
urms = zeros(1, 2);
J    = zeros(1, 2);

for k = 1:2

    y = Y(:, k);
    u = U(:, k);

    %% Sobreimpulso

    ymax = max(y(idx:end));

    Mp(k) = 100*max(0, ymax-rf)/abs(dr);

    %% Tiempo de establecimiento

    error = abs(y(idx:end)-rf);

    fuera = find(error > tol, 1, 'last');

    if isempty(fuera)

        ts(k) = 0;

    elseif fuera < length(error)

        idx_est = idx+fuera;

        ts(k) = t(idx_est)-t_escalon;

    end

    %% Error medio durante el ultimo segundo

    ess(k) = mean(ref(end-Nf+1:end) ...
                  -y(end-Nf+1:end));

    %% Error al finalizar el ensayo

    ef(k) = ref(end)-y(end);

    %% RMSE respecto del modelo objetivo

    RMSE(k) = sqrt(mean((y-ym).^2));

    %% Esfuerzo maximo de control

    umax(k) = max(abs(u));

    %% Valor eficaz del control

    urms(k) = sqrt(mean(u.^2));

    %% Funcion de costo

    J(k) = (sum((y-ym).^2) ...
            +lambda*sum(u.^2))/(2*N);

end

%% 7) TABLA COMPARATIVA

indicadores = {
    'Sobreimpulso [%]';
    'Tiempo de establecimiento [s]';
    'Error medio final [m]';
    'Error ultima muestra [m]';
    'RMSE modelo objetivo [m]';
    'Control maximo [rad]';
    'Control RMS [rad]';
    'Funcion de costo'
};

Inicial = [
    Mp(1);
    ts(1);
    ess(1);
    ef(1);
    RMSE(1);
    umax(1);
    urms(1);
    J(1)
];

Final = [
    Mp(2);
    ts(2);
    ess(2);
    ef(2);
    RMSE(2);
    umax(2);
    urms(2);
    J(2)
];

Resultados = table(indicadores, Inicial, Final);

fprintf('\n--- INDICADORES DE DESEMPEÑO ---\n\n');

disp(Resultados);

%% 8) REDUCCION DE LA FUNCION DE COSTO

reduccion_J = 100*(J(1)-J(2))/J(1);

fprintf('\n--- COMPARACION EXPERIMENTAL ---\n');

fprintf('Costo inicial: %.8f\n', J(1));
fprintf('Costo final:   %.8f\n', J(2));

fprintf('Reduccion del costo: %.2f %%\n', ...
    reduccion_J);

%% 9) VERIFICACION DEL ESTABLECIMIENTO

nombres = {'Controlador inicial', ...
           'Controlador final'};

for k = 1:2

    if isnan(ts(k))

        fprintf(['\n%s: no alcanza el ' ...
                 'establecimiento durante el ensayo.\n'], ...
                 nombres{k});

    end

end