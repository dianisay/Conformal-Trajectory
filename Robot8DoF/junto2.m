clear all
clc
%% ===================== CONFIGURACIÓN INICIAL ============================
%% Cámara
res = "1280x720";
nameContains = ""; 
cam = pick_webcam(nameContains, res);

% Intrínsecas
load('cameraParams.mat','cameraParams');
K  = cameraParams.IntrinsicMatrix';
fx = K(1,1); fy = K(2,2);
cx = K(1,3); cy = K(2,3);

% Red
load('trainedNet.mat','net','psX','psT');

% Profundidad del plano (mm)
Zc = 650;

% === Parámetro de calibración de la red ===
yaw0 = 0;  

% === Límites reales del workspace del mycobot ===
Xmin = -280;  Xmax = 280;
Ymin = -280;  Ymax = 280;

%% ===================== CARTESIANO (GRBL/Marlin/Klipper) =================
port = 'COM5'; baudRate = 115200;
s = serialport(port, baudRate);
configureTerminator(s, "LF"); flush(s);
disp('Conexión serial abierta correctamente');
leerDatos(s, 39);

writeline(s,'G28');                        pause(1.0);  leerDatos(s,39);

% Reinicio ordenado del objeto serialport (sin fclose)
try, clear s; catch, end
s = serialport(port, baudRate);
configureTerminator(s,"LF"); flush(s);
disp('Conexión serial abierta correctamente (reiniciada)');
leerDatos(s, 39);

writeline(s,'G1 X180 Y180 F3000');         pause(1.5);  leerDatos(s, 3);
writeline(s,'G90');                         pause(0.1);  leerDatos(s, 5);   
writeline(s,'G92 X0 Y0');                   pause(0.2);  leerDatos(s, 5);  

% Tras G92, el origen de trabajo es (0,0). Fija estimado coherente:
XR_est = 0; 
YR_est = 0;

%% ===================== CONTROL PD EN XY =========================
Kp = 0.8;             % proporcional [mm/mm]
Kd = 0.1;             % derivativo   [mm/(mm/s)]
tol_mm = 5.0;         % radio en mm
maxStepXY = 15.0;     % mm por ciclo (por eje)
writeline(s,'G90'); pause(0.05); leerDatos(s,1);

lastE = [0;0];
lastT = tic;

% Fallback y blending
lastFeedbackT = tic;
fallbackAggroT = 5.0;
alpha_sync     = 0.6;

% Yaw actual usado en fase cart. (si no se conoce aún, igual a yaw0)
yaw_cur = yaw0;

%% ===================== MYCOBOT 280 (PYTHON) =========================
pyExe = "C:\Users\MonDi\AppData\Local\Programs\Python\Python311\python.exe";
pe = pyenv;
if pe.Status == "Loaded" && pe.ExecutionMode == "OutOfProcess"
    terminate(pyenv);
elseif pe.Status == "Loaded" && pe.ExecutionMode == "InProcess"
    error("Python está InProcess. Reinicia MATLAB.");
end
pyenv('Version', pyExe, 'ExecutionMode', 'OutOfProcess');

mc = py.importlib.import_module('mc_bridge');
py.importlib.reload(mc);
disp("✅ mc_bridge importado.");

q_home = [0, -80, 120, 60, 30, 0];
disp("→ HOME...");
disp(mc.home(py.list(num2cell(q_home))));

%% ===================== FIGURAS =========================
bufN = 300;
Xt  = nan(1,bufN);   Yt  = nan(1,bufN);
Xp  = nan(1,bufN);   Yp  = nan(1,bufN);
tt  = nan(1,bufN);
cxHist = []; cyHist = [];

hFig = figure('Name','Seguimiento de centroide (Q para salir)', ...
              'NumberTitle','off', 'Color','w', ...
              'KeyPressFcn', @(f,k) setappdata(f,'key',k.Key));

subplot(2,2,[1 3]);
hImg  = imshow(zeros(720,1280,3,'uint8')); 
title('Frame + trayectoria del centroide'); hold on;
hPath = plot(nan,nan,'r.-','LineWidth',1.2,'DisplayName','Trayectoria');
hPt   = plot(nan,nan,'b*','MarkerSize',10,'LineWidth',1.5);
hTxt  = text(20,40,'','Color','y','FontSize',12,'FontWeight','bold');
legend('Location','southoutside'); grid on;

subplot(2,2,2); hold on; grid on;
hIn  = plot(nan,nan,'-o','DisplayName','Xc'); 
hIn2 = plot(nan,nan,'-o','DisplayName','Yc');
xlabel('t [s]'); ylabel('Entrada cámara [mm]'); title('Entradas (Xc,Yc)'); legend;

subplot(2,2,4); hold on; grid on;
hOut = plot(nan,nan,'-o','DisplayName','X_{pred}'); 
hOut2= plot(nan,nan,'-o','DisplayName','Y_{pred}');
xlabel('t [s]'); ylabel('Salida predicha [mm]'); title('Salidas (X,Y) - Red'); legend;

idx = 0; t0 = tic; fps_t = tic; frames = 0;

%% ===================== LOOP EN TIEMPO REAL =========================
Xpred2 = NaN; Ypred2 = NaN;
D = inf;                              % distancia al objetivo
try
    while ishandle(hFig)
        % Salir con Q
        k = getappdata(hFig,'key');
        if ~isempty(k) && (k=='q' || k=='Q'), disp('Salida manual.'); break; end

        if (D > 4)
            % 1) Captura
            img_raw = snapshot(cam);

            % 2) Undistorsión
            img = undistortImage(img_raw, cameraParams, 'OutputView','same');

            % 3) Segmentación simple (Otsu + morfología)
            grayImage = rgb2gray(img);
            grayImage = imadjust(grayImage, stretchlim(grayImage), []);
            bw = imbinarize(grayImage);
            bw = imopen(bw, strel('disk', 2));
            bw = imclose(bw, strel('disk', 3));
            bw = imfill(bw, 'holes');

            % 4) Centroide
            stats = regionprops(bw, 'Area','Centroid');
            if isempty(stats)
                set(hImg,'CData',img);
                set(hPt,'XData',nan,'YData',nan);
                set(hTxt,'String','No se encontró objeto');
                drawnow limitrate; 
                continue;
            end
            [~,imax] = max([stats.Area]);
            centroid = stats(imax).Centroid;

            % 5) Back-projection -> mm
            u = centroid(1); v = centroid(2);
            x_norm = (u - cx)/fx;  y_norm = (v - cy)/fy;
            Xc = x_norm * Zc; 
            Yc = y_norm * Zc;

            % 6) Red
            nuevo_X  = [Xc; Yc];
            nuevo_Xn = mapminmax('apply', nuevo_X, psX);
            nuevo_Yn = net(nuevo_Xn);
            nuevo_Y  = mapminmax('reverse', nuevo_Yn, psT);
            % ---->> COMPENSACIÓN POR YAW (rota salida al marco actual) <<----
            Xpred = nuevo_Y(1);  
            Ypred = nuevo_Y(2);
            Xpred2 = max(0, min(350, Xpred)) * -1;
            Ypred2 = max(0, min(350, Ypred)) * -1;

            % ===== Lectura XY con fallback =====
            [XR_meas, YR_meas, ok, src] = getXY(s);
            if ok && ~any(isnan([XR_meas YR_meas]))
                XR = alpha_sync*XR_meas + (1-alpha_sync)*XR_est;
                YR = alpha_sync*YR_meas + (1-alpha_sync)*YR_est;
                XR_est = XR; YR_est = YR;
                lastFeedbackT = tic;
            else
                XR = XR_est; YR = YR_est; src = "EST";
            end

            % Distancia al objetivo
            D = hypot(Xpred2 - XR, Ypred2 - YR);

            % Agresividad según feedback
            timeNoFb = toc(lastFeedbackT);
            if strcmp(src,"EST") && timeNoFb > fallbackAggroT
                maxStepXY_eff = min(maxStepXY, 1.0);
                Kp_eff = 0.4;  Kd_eff = Kd;
            else
                maxStepXY_eff = maxStepXY;
                Kp_eff = Kp;   Kd_eff = Kd;
            end

            % ==== PD con zona muerta ====
            e = [Xpred2 - XR; Ypred2 - YR];
            if norm(e) <= tol_mm
                set(hTxt,'String',sprintf('[%s] En área (|e|=%.2f mm) -> HOLD', string(src), norm(e)));
            else
                dt = max(1e-3, toc(lastT));
                de = (e - lastE) / dt;
                uPD = Kp_eff*e + Kd_eff*de;
                stepXY = max(-maxStepXY_eff, min(maxStepXY_eff, uPD));

                % Movimiento relativo
                writeline(s, 'G91');                           pause(0.005);  leerDatos(s,1);
                writeline(s, sprintf('G1 X%.3f Y%.3f F3000', stepXY(1), stepXY(2)));
                pause(0.02);                                   leerDatos(s,1);
                writeline(s, 'G90');                           pause(0.005);  leerDatos(s,1);

                XR_est = XR + stepXY(1);
                YR_est = YR + stepXY(2);
                lastE = e; lastT = tic;
            end

            % === UI ===
            idx = idx + 1; ii = 1 + mod(idx-1, bufN);
            tt(ii) = toc(t0);  
            Xt(ii) = Xc;  Yt(ii) = Yc;  
            Xp(ii) = Xpred2;  Yp(ii) = Ypred2;

            cxHist(end+1) = centroid(1);
            cyHist(end+1) = centroid(2);
            set(hImg,'CData',img);
            set(hPt,'XData',centroid(1),'YData',centroid(2));
            set(hPath,'XData',cxHist,'YData',cyHist);
            set(hTxt,'String', sprintf('[%s] |e|=%.2f mm  Dest=(%.1f,%.1f)', ...
                string(src), norm(e), Xpred2, Ypred2));
            set(hIn, 'XData', tt, 'YData', Xt);
            set(hIn2,'XData', tt, 'YData', Yt);
            set(hOut,'XData', tt, 'YData', Xp);
            set(hOut2,'XData', tt, 'YData', Yp);
            drawnow limitrate;

            frames = frames + 1;
            if toc(fps_t) > 1
                frames = 0; fps_t = tic;
            end
        end % if (D>4)

        % =================== BLOQUE FINAL: D <= 4 ====================
        if (D <= 4)
            disp('Objetivo alcanzado: deteniendo cartesiano...');
            writeline(s,'M0');    pause(0.5);  leerDatos(s,1);
            writeline(s,'M18');   pause(0.5);  leerDatos(s,1);

            % Asegura que tenemos una predicción válida
            if isnan(Xpred2) || isnan(Ypred2)
                warning('Xpred2/Ypred2 no válidos; no se ejecuta etapa MyCobot.');
                break;
            end

            % ===== Recalcula una sola vez el objetivo con la cámara =====
            img_raw = snapshot(cam);
            img = undistortImage(img_raw, cameraParams, 'OutputView','same');
            grayImage = rgb2gray(img);
            grayImage = imadjust(grayImage, stretchlim(grayImage), []);
            bw = imbinarize(grayImage);
            bw = imopen(bw, strel('disk', 2));
            bw = imclose(bw, strel('disk', 3));
            bw = imfill(bw, 'holes');
            stats = regionprops(bw, 'Area','Centroid');
            if isempty(stats), warning('Sin objeto en etapa final.'); 
                break; 
            end
            [~,imax] = max([stats.Area]);
            centroid = stats(imax).Centroid;
            u = centroid(1); v = centroid(2);
            x_norm = (u - cx)/fx;  y_norm = (v - cy)/fy;
            Xc = x_norm * Zc;  Yc = y_norm * Zc;

            % === Red + Compensación por yaw (ahora con yaw_cur REAL) ===
            poseA=mc.get_pose(); cA=pylist_to_double(poseA{'coords'}); if any(isnan(cA(1:3))), error('Lectura coords NaN'); end
            yaw_cur = cA(6);    % yaw real
            y_hat=mapminmax('reverse', net(mapminmax('apply',[Xc;Yc],psX)), psT);
            dpsi=deg2rad(wrapTo180(yaw_cur-yaw0)); Rz=[cos(dpsi) -sin(dpsi); sin(dpsi) cos(dpsi)];
            pRot=Rz*y_hat(1:2);
            Xpred2=min(max(Xmin,pRot(1)),Xmax);
            Ypred2=min(max(Ymin,pRot(2)),Ymax);

            dpsi = deg2rad(wrapTo180(yaw_cur - yaw0));
            Rz   = [cos(dpsi) -sin(dpsi); sin(dpsi) cos(dpsi)];
            pRot = Rz * [nuevo_Y(1); nuevo_Y(2)];
            Xpred2 = min(max(Xmin, pRot(1)), Xmax);
            Ypred2 = min(max(Ymin, pRot(2)), Ymax);
            Rmax = 280;   % 
            Rsafe = 260;  % mm, 
            
            if hypot(Xpred2, Ypred2) > Rsafe
                theta = atan2(Ypred2, Xpred2);
                Xpred2 = Rsafe * cos(theta);
                Ypred2 = Rsafe * sin(theta);
            end

            % ====== TRAYECTORIA CON MYCOBOT ======
            rx_fix = cA(4);    % conserva roll
            ry_fix = cA(5);    % conserva pitch

            % Yaw target mirando al objetivo en XY (opcional; aquí conservamos rz actual hacia target suave)
            p_now = cA(1:3);
            p_tar = [Xpred2, Ypred2, 320];
            v_xy  = [p_tar(1)-p_now(1); p_tar(2)-p_now(2)];
            if norm(v_xy) < 1e-6
                yaw_des = cA(6);
            else
                yaw_des = atan2d(v_xy(2), v_xy(1));
            end

            % Interpolación SUAVE de yaw (sigmoide 0→1)
            T  = 7.0;  dt = 0.03;  t  = 0:dt:T;
            cYaw = 1.0; sYaw = 1 ./ (1 + exp(-cYaw*(t - T/2)));
            dYaw = wrapTo180(yaw_des - yaw_cur);
            yaw_seq = yaw_cur + sYaw * dYaw;

            rf = [p_tar(1), p_tar(2), p_tar(3), rx_fix, ry_fix, yaw_des];

            % Trayectoria sigmoidal en posición
            c1 = 0.5; c2 = 1.0; c3 = 1.0;
            ri = cA;
            Sx = ri(1) + (rf(1)-ri(1))./(1 + exp(-c1*(t - T/2)));
            Sy = ri(2) + (rf(2)-ri(2))./(1 + exp(-c2*(t - T/2)));
            Sz = ri(3) + (rf(3)-ri(3))./(1 + exp(-c3*(t - T/2)));
            z_min = 80;  Sz = max(Sz, z_min);

            % ---- Modo de movimiento (ajusta 0/1 según tu mc_bridge) ----
            mc.set_move_mode(int32(0));    % prueba 0 si con 1 no obedece
            spd_waypt = int32(80);

            tau = 0.10; idx_tau  = @(k) max(1, k - round(tau/dt));
            dt_cmd = 0.03; accum = 0;

            % Buffers de telemetría
            N  = numel(t);
            r_log   = nan(N,3);
            ref_log = [Sx(:), Sy(:), Sz(:)];
            e_log   = nan(N,3);
            t_log   = (0:N-1)'.*dt;
            E_to_goal = zeros(1,N);
            last_valid = cA(1:3);
            tic;

            for i = 1:N
                xi = Sx(i); yi = Sy(i); zi = Sz(i);

                % Orientación por waypoint: rx,ry fijos; rz = yaw_i
                yaw_i = yaw_seq(i);
                rx_i = rx_fix; ry_i = ry_fix; rz_i = yaw_i;

                % ---- thinning de comandos ----
                accum = accum + dt;
                if accum >= dt_cmd
                    % fprintf('[CMD] xyz=(%.1f,%.1f,%.1f)  rxyz=(%.1f,%.1f,%.1f)\n', xi, yi, zi, rx_i, ry_i, rz_i);
                    mc.move_cartesian(xi, yi, zi, rx_i, ry_i, rz_i, spd_waypt, false, 0);
                    accum = 0;
                end

                % ---- telemetría tolerante ----
                pose_i = mc.get_pose();
                coords = pylist_to_double(pose_i{'coords'});
                if any(isnan(coords(1:3)))
                    r = last_valid(:);
                else
                    r = coords(1:3).';
                    last_valid = coords(1:3);
                end

                % ---- error al waypoint con retardo compensado ----
                kdel  = idx_tau(i);
                xref  = Sx(kdel); yref = Sy(kdel); zref = Sz(kdel);
                e_wp  = [xref; yref; zref] - r(:);

                % Guardar trazas
                r_log(i,:) = r(:).';
                e_log(i,:) = e_wp(:).';
                E_to_goal(i) = norm([rf(1); rf(2); rf(3)] - r(:));

                pause(dt);
            end
            T_exec = toc;
            fprintf("Tiempo de ejecución: %.2f s (%d waypoints)\n", T_exec, N);

            poseF = mc.get_pose();
            cF    = pylist_to_double(poseF{'coords'});
            fprintf("Pose FINAL: [%.2f %.2f %.2f | %.2f %.2f %.2f]\n", cF);
            err_vec = [rf(1)-cF(1), rf(2)-cF(2), rf(3)-cF(3)];
            fprintf("Error a RF: [%.2f %.2f %.2f] mm\n", err_vec);

            % ===== Métricas y gráficas =====
            valid = all(~isnan(r_log),2);
            t_plot   = t_log(valid);
            r_plot   = r_log(valid,:);
            ref_plot = ref_log(valid,:);
            e_plot   = e_log(valid,:);

            rmse_x  = sqrt(mean((e_plot(:,1)).^2));
            rmse_y  = sqrt(mean((e_plot(:,2)).^2));
            rmse_z  = sqrt(mean((e_plot(:,3)).^2));
            rmse_3d = sqrt(mean(sum(e_plot.^2,2)));
            fprintf('\nRMSE  X=%.2f  Y=%.2f  Z=%.2f  |e|=%.2f  [mm]\n', rmse_x, rmse_y, rmse_z, rmse_3d);

            figure('Name','Trayectoria 3D: deseada vs. real','Color','w');
            plot3(ref_plot(:,1), ref_plot(:,2), ref_plot(:,3), '.', 'MarkerSize',8); hold on;
            plot3(r_plot(:,1),   r_plot(:,2),   r_plot(:,3),   '-', 'LineWidth',1.6);
            grid on; axis equal;
            xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
            title(sprintf('Trayectoria 3D (RMSE_{3D}=%.2f mm)', rmse_3d));
            legend('Deseada','Real','Location','best');

            figure('Name','Error por eje','Color','w');
            plot(t_plot, e_plot(:,1),'LineWidth',1.6); hold on;
            plot(t_plot, e_plot(:,2),'LineWidth',1.6);
            plot(t_plot, e_plot(:,3),'LineWidth',1.6);
            grid on; xlabel('t [s]'); ylabel('error [mm]');
            legend('e_x','e_y','e_z','Location','best');
            title(sprintf('Error por eje (RMSE: X=%.2f, Y=%.2f, Z=%.2f mm)', rmse_x, rmse_y, rmse_z));

            e_norm = sqrt(sum(e_plot.^2,2));
            figure('Name','Norma del error','Color','w');
            plot(t_plot, e_norm, 'LineWidth',1.8); grid on;
            xlabel('t [s]'); ylabel('|e| [mm]');
            title(sprintf('Norma del error |e| (RMSE_{3D}=%.2f mm)', rmse_3d));

            if norm(err_vec) <= 10
                disp('MyCobot: error <= 10 mm. Secuencia finalizada.');
            end

            break;  % salir del while principal tras la etapa final
        end % if (D<=4)

    end % while

catch ME
    warning('Error en loop: %s', ME.message);
end

% Limpieza
try, clear cam; catch, end

%% ===================== HELPERS =====================
function [XR, YR, ok, src] = getXY(s)
% Intenta leer XY desde:
% 1) Marlin/RepRap:  M114 -> "X:... Y:..."
% 2) GRBL:           '?'  -> "<...|WPos:x,y,z|...>" o "<...|MPos:x,y,z|...>"
% 3) Klipper:        'GET_POSITION' -> "position: x:... y:..."
% Devuelve:
%   XR, YR  [mm]
%   ok      true si pudo parsear
%   src     'M114' | 'GRBL:WPos' | 'GRBL:MPos' | 'KLIPPER' | 'NONE'

    XR = NaN; YR = NaN; ok = false; src = 'NONE';

    % ---- utilidades de lectura
    function txt = readBurst(timeout_s)
        if nargin<1, timeout_s = 0.25; end
        raw = strings(0);
        t0 = tic;
        while toc(t0) < timeout_s
            if s.NumBytesAvailable > 0
                try
                    raw(end+1) = readline(s); %#ok<AGROW>
                catch
                    break;
                end
            else
                pause(0.01);
            end
        end
        txt = strjoin(raw, ' ');
    end

    % ===== 1) M114 (Marlin/RepRap) =====
    try
        flush(s);
        writeline(s,'M114');               
        pause(0.03);
        txt = readBurst(0.25);
        rx = regexp(txt, 'X\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        ry = regexp(txt, 'Y\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rx) && ~isempty(ry)
            XR = str2double(rx{1}); 
            YR = str2double(ry{1}); 
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'M114'; return; end
        end
    catch
    end

    % ===== 2) GRBL ('?') =====
    try
        flush(s);
        write(s, '?', 'char');             
        pause(0.05);
        txt = readBurst(0.25);
        % WPos (coords de trabajo)
        rW = regexp(txt, 'WPos\s*:\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rW)
            XR = str2double(rW{1}); 
            YR = str2double(rW{2});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'GRBL:WPos'; return; end
        end
        % MPos (coords máquina)
        rM = regexp(txt, 'MPos\s*:\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rM)
            XR = str2double(rM{1}); 
            YR = str2double(rM{2});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'GRBL:MPos'; return; end
        end
    catch
    end

    % ===== 3) Klipper ('GET_POSITION') =====
    try
        flush(s);
        writeline(s, 'GET_POSITION');
        pause(0.05);
        txt = readBurst(0.25);
        rx = regexp(txt, 'x\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        ry = regexp(txt, 'y\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rx) && ~isempty(ry)
            XR = str2double(rx{1}); 
            YR = str2double(ry{1});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'KLIPPER'; return; end
        end
    catch
    end
end

function pyl = to_py_list_vec(v)
% Convierte vector numérico (fila o columna) a py.list asegurando 1xN
    pyl = py.list(num2cell(v(:)'));   % fuerza fila 1xN
end

function v = pylist_to_double(pyobj)
% Convierte py.list de escalares o listas (vector/matriz pequeña) a double
% Evita: Non-scalar in Uniform output...
    C = cell(pyobj);
    if isempty(C)
        v = [];
        return;
    end
    if all(cellfun(@(x) ~iscell(x), C))      % vector de escalares
        v = cellfun(@double, C, 'UniformOutput', true);
    else                                      % lista de listas -> matriz
        n = numel(C); m = numel(cell(C{1}));
        v = zeros(n,m);
        for i = 1:n
            v(i,:) = cellfun(@double, cell(C{i}), 'UniformOutput', true);
        end
    end
end

function J = pylist2mat(J_py)
% Convierte un Jacobiano py.list[6] de py.list[6] a double[6x6]
    rows = cell(J_py);                  % 1x6 celdas (cada una py.list de 6)
    n = numel(rows);
    m = numel(cell(rows{1}));
    J = zeros(n,m);
    for i = 1:n
        J(i,:) = cellfun(@double, cell(rows{i}), 'UniformOutput', true);
    end
end

%% ===== Helpers de orientación (matriz -> rx,ry,rz en ZYX) =====
function [rx, ry, rz] = R_to_rxyz_ZYX(R)
% Convierte R(3x3) a (rx,ry,rz) en grados usando convención ZYX (yaw-pitch-roll)
    R = R(1:3,1:3);
    ry = asind(-R(3,1));
    if abs(cosd(ry)) > 1e-6
        rx = atan2d(R(3,2), R(3,3));
        rz = atan2d(R(2,1), R(1,1));
    else
        % gimbal lock
        rx = 0;
        rz = atan2d(-R(1,2), R(2,2));
    end
    rx = wrapTo180(rx); ry = wrapTo180(ry); rz = wrapTo180(rz);
end

function a = wrapTo180(a)
    a = mod(a + 180, 360) - 180;
end
