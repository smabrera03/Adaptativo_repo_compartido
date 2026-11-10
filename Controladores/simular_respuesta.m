function [y, u, datos] = simular_respuesta(t, ref, kp, ki, kd)
% SIMULAR_RESPUESTA  Ensayo a lazo cerrado del carro-barra en el WeMos.
%   [y, u] = simular_respuesta(t, ref, kp, ki, kd)
%   [y, u, datos] = ...   (datos: telemetría cruda, sin remuestrear)
%
%   t    : eje de tiempos [s] (vector)
%   ref  : referencia de posición [m], mismo largo que t
%   kp, ki, kd : constantes del regulador
%       R(q) = kp*(1 + ki/(1 - q^-1) + kd*(1 - q^-1))
%       Con kd = 0 es el PI de la filmina 17.
%
%   y : posición del carro q2 [m], remuestreada sobre t
%   u : acción de control enviada al WeMos (q1_ref) [rad], sobre t

    %% Configuración de la comunicación
    PUERTO  = 'COM11';
    BAUD    = 115200;
    TIMEOUT = 2;       % [s] timeout de lectura
    T_ESPERA_TRAMA = 10;  % [s] máximo para recibir la primera trama
    SIGNO   = -1;      % convención del código original: q1_ref = Kp*(q2 - r)

    %% Validación de entradas
    t = t(:);  ref = ref(:);
    if numel(t) ~= numel(ref)
        error('t y ref deben tener el mismo largo.');
    end
    tau  = t - t(1);          % tiempo relativo
    Texp = tau(end);

    %% Apertura del puerto
    delete(instrfindall);
    s = serial(PUERTO, 'BaudRate', BAUD, 'Terminator', 'LF', 'Timeout', TIMEOUT);
    fopen(s);
    limpieza = onCleanup(@() cerrar_puerto(s)); % cierra aunque haya error o Ctrl+C

    pause(5);  flushinput(s);  fprintf(s, '0\n');
    pause(2);  flushinput(s);  fprintf(s, '0\n');   % el ESP8266 puede reiniciarse al abrir

    %% Esperar una trama válida
    t_espera = tic;
    while true
        v = sscanf(fgetl(s), '%f');
        if numel(v) == 4 && all(isfinite(v)), break; end
        if toc(t_espera) > T_ESPERA_TRAMA
            error('No se recibió telemetría válida del WeMos.');
        end
    end

    %% Lazo cerrado
    % Columnas: t_wemos, timestamp_us, q1, q2, u_wemos, r, u_cmd
    buf = zeros(20000, 7);  n = 0;

    ts_prev = v(1);
    t_w     = 0;        % tiempo según reloj del WeMos [s]
    idx     = 1;        % índice de referencia vigente (retención de orden cero)
    e1 = 0;  e2 = 0;    % e[k-1], e[k-2]
    v_prev = 0;         % salida del regulador v[k-1]

    while t_w < Texp
        vn = sscanf(fgetl(s), '%f');
        if numel(vn) ~= 4 || any(~isfinite(vn)), continue; end

        ts = vn(1);  q1 = vn(2);  q2 = vn(3);  u_wemos = vn(4);

        dt = mod(ts - ts_prev, 2^32) * 1e-6;
        ts_prev = ts;
        if dt <= 0 || dt > 0.5, continue; end
        t_w = t_w + dt;

        % Referencia vigente en t_w
        while idx < numel(tau) && tau(idx+1) <= t_w
            idx = idx + 1;
        end
        r = ref(idx);

        % PID en forma INCREMENTAL:
        % v[k] = v[k-1] + kp*[(1+ki+kd) e[k] - (1+2kd) e[k-1] + kd e[k-2]]
        e  = r - q2;
        vk = v_prev + kp*((1 + ki + kd)*e - (1 + 2*kd)*e1 + kd*e2);
        u_cmd = SIGNO * vk;

        fprintf(s, '%.6f\n', u_cmd);

        n = n + 1;
        if n > size(buf, 1), buf = [buf; zeros(20000, 7)]; end %#ok<AGROW>
        buf(n, :) = [t_w, ts, q1, q2, u_wemos, r, u_cmd];

        e2 = e1;  e1 = e;  v_prev = vk;
    end
    % Al salir, onCleanup manda '0' y cierra el puerto.

    %% Salidas remuestreadas sobre t
    buf = buf(1:n, :);
    tw  = buf(:, 1);
    
    %El WeMos no necesariamente usa el mismo eje de tiempos que matlab.
    %Hace falta interpolar
    y = fillmissing(interp1(tw, buf(:,4), tau, 'linear'),   'nearest');
    u = fillmissing(interp1(tw, buf(:,7), tau, 'previous'), 'nearest');

    if nargout > 2
        datos = array2table(buf, 'VariableNames', ...
            {'t', 'timestamp_us', 'q1', 'q2', 'u_wemos', 'r', 'u_cmd'});
    end
end

%% Función local: cierre seguro del puerto
function cerrar_puerto(s)
    try, fprintf(s, '0\n'); catch, end   % torque/consigna a cero
    try
        if strcmp(s.Status, 'open'), fclose(s); end
        delete(s);
    catch
    end
end