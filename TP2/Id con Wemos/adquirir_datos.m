delete(instrfindall);
clear; clc; close all;

%% Configuracion del ensayo

Texp = 60;           % Duracion [s]
Ts_prbs = 1.5;      % Actualizacion PRBS: 20 Hz
A = 0.1;            % Amplitud de la entrada [rad]

% Controlador PD (valores iniciales para ajustar)
Kp = 1.0;
Kd = 0.0;
% Filtro de velocidad
alpha = 0.7;

%% Generar PRBS
% Requiere System Identification Toolbox
N = ceil(Texp/Ts_prbs) + 1;
prbs = A * idinput(N, 'prbs', [0 1], [-1 1]);
prbs = prbs(:);

%prbs = ones(N,1);

%% Abrir puerto serie
s = serial('COM11', ...
    'BaudRate', 115200, ...
    'Terminator', 'LF', ...
    'Timeout', 2);

fopen(s);
pause(5);
flushinput(s);

fprintf(s, '0\n');


% El puerto puede reiniciar el ESP8266 al abrirse
pause(2);
flushinput(s);
fprintf(s, '0\n');

fprintf("Esperando telemetria del Wemos...\n");

%% Esperar una trama valida
while true
    linea = fgetl(s); %extrae la línea de caracteres
    v = sscanf(linea, '%f'); %convierte los caracteres a un vector de flotantes

    if numel(v) == 4 && all(isfinite(v))
        break;
    end
end

fprintf("Adquisicion iniciada.\n");

%% Adquisicion en lazo cerrado

% Columnas:
% timestamp_us, q1, q2, u_wemos, referencia, tau_cmd
T = zeros(0,7);

t0 = tic;
t_ref_next = 0;
k = 1;

% Estado inicial del estimador de velocidad
q1_prev = v(2);
q2_prev = v(3);
ts_prev = v(1);
q1_p_f = 0;
q2_p_f = 0;

% Referencia inicial
r = prbs(k);
k = k + 1;

while toc(t0) < Texp

    % Leer telemetria: timestamp_us, q1, q2, u
    linea = fgetl(s); %extrae una línea de caracteres
    vn = sscanf(linea, '%f'); %extrae de la línea un vector de 4 flotantes

    if numel(vn) ~= 4 || any(~isfinite(vn))
        continue;
    end

    timestamp_us = vn(1); %Tiempo según el wemos [us]
    q1 = vn(2); %ángulo de la barra [rad]
    q2 = vn(3); %posición del carro [m]
    u_wemos = vn(4); %acción de control aplicada por el wemos

    % Intervalo real entre muestras segun micros() del Wemos
    dt = mod(timestamp_us - ts_prev, 2^32)*1e-6;

    if dt <= 0 || dt > 0.5
        ts_prev = timestamp_us;
        q2_prev = q2;
        q1_prev = q1;
        continue;
    end

    % Estimacion filtrada de la velocidad del carro
    q2_p_raw = (q2 - q2_prev)/dt;
    q2_p_f = alpha*q2_p_f + (1-alpha)*q2_p_raw;

    % Actualizar referencia PRBS
    t = toc(t0);

    if t >= t_ref_next
        if k <= numel(prbs)
            r = prbs(k);
            k = k + 1;
        else
            r = 0;
        end

        t_ref_next = t_ref_next + Ts_prbs;
    end

    % Control de posicion del carro
    % Realimentacion de estados
    q1_ref = Kp*(-r+q2); % - Kd*q2_p_f; 

    % Enviar q1_ref al Wemos
    fprintf(s, '%.6f\n', q1_ref);

    % Guardar telemetria y variables del controlador
    T(end+1,:) = [timestamp_us, q1, q2, q2_p_f ...
                  u_wemos, r, q1_ref];

    % Actualizar muestras anteriores
    q2_prev = q2;
    ts_prev = timestamp_us;
end

% Llevar el torque a cero al terminar
fprintf(s, '0\n');

fclose(s);
delete(s);
clear s;

%% Procesamiento de los datos
%{
Columnas de T:
  T(:,1) = timestamp_us -> Timestamp del Wemos [us] (micros(), 32 bits)
  T(:,2) = q1           -> Angulo de la barra medido [rad]
  T(:,3) = q2           -> Posicion del carro medida [m]
   T(:, 4) = q2_p_f   -> Velocidad del carro luego de pasar por el filtro
  T(:,5) = u_wemos      -> Accion de control aplicada, reportada por el Wemos [rad]
  T(:,6) = r            -> Referencia PRBS en ese instante [rad]
  T(:,7) = q1_ref       -> Consigna enviada por MATLAB al Wemos [rad]
%}
timestamp_us = T(:,1);
q1 = T(:,2); %ángulo de la barra [rad]
q2 = T(:,3); %posición del carro [m]
q2_p = T(:, 4);
u = T(:,5); %acción de control [rad]
r = T(:, 6);

% Tiempo relativo usando el reloj del Wemos.
% Contempla el desbordamiento de micros() a los ~71.6 min.
dt_us = mod(diff(timestamp_us), 2^32);
t = [0; cumsum(dt_us)] * 1e-6;

datos = table(t, timestamp_us, q1, q2, q2_p, u, r);

%% Graficos

figure;
subplot(2, 1, 1);
plot(t, q2, 'LineWidth', 1.5);
grid on;
title('Posición [m]');

subplot(2, 1, 2);
plot(t, r, 'LineWidth', 1.5);
grid on;
title('Referencia[m]');
ylim( [-1.1*A, 1.1*A]);


figure;

subplot(2,1,1);
plot(t, rad2deg(u));
grid on;
ylabel('u [°]');
title('Acción de control');

subplot(2, 1, 2);
plot(t, q2_p);
grid on;
ylabel('q2\_p');
title('Velocidad');

%% Guardar
%writetable(datos, 'datos_pendulo_prbs.csv');
%save('datos_pendulo_prbs.mat', 'datos', 'prbs', 'Ts_adq', 'Ts_prbs', 'A');

%save('datos_cart_beam.mat', 'datos');

fprintf('\nMuestras adquiridas: %d\n', height(datos));
fprintf('Duracion registrada: %.3f s\n', t(end));
fprintf('Intervalo medio: %.3f ms\n', 1000*mean(diff(t)));
fprintf('Datos guardados.\n');