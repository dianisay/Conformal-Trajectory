% ar4_base_test.m
% =========================================================================
% Codigo base: genera patrones XYZ simples con orientacion fija
% para que Monse pruebe la comunicacion con el robot AR4.
%
% Salida:  archivos CSV  ->  X, Y, Z, Orientacion
%          Unidades: mm, grados
%          Orientacion = 90° (boquilla perpendicular / apuntando hacia abajo)
%
% USO:
%   1. Ajustar CX, CY segun donde este el scaffold en el workspace del AR4
%   2. Ejecutar en MATLAB
%   3. Se generan 3 CSV + se imprimen en consola
%   4. Monse toma el CSV y lo mete a su codigo del robot
% =========================================================================

close all; clear; clc;

%% ===================== PARAMETROS AR4 =====================
Z_MAX    = 450;   % mm — limite superior de Z del AR4
Z_MIN    = 270;   % mm — limite inferior de Z del AR4
Z_WORK   = 290;   % mm — altura de deposicion (cerca del scaffold)
Z_TRAVEL = 380;   % mm — altura de desplazamiento (boquilla arriba)
ORI      = 90;    % grados — orientacion fija (perpendicular)

% Centro del area de trabajo en XY — AJUSTAR segun posicion del scaffold
CX = 0;     % mm
CY = 300;   % mm (al frente del robot, dentro de su alcance)

%% ===================== PATRON 1: CUADRADO 20x20 mm =====================
lado = 20; % mm
hl = lado / 2;

pts_cuadrado = [
    CX-hl, CY-hl, Z_TRAVEL, ORI    % ir a esquina (arriba)
    CX-hl, CY-hl, Z_WORK,   ORI    % bajar a superficie
    CX+hl, CY-hl, Z_WORK,   ORI    % trazar cuadrado
    CX+hl, CY+hl, Z_WORK,   ORI
    CX-hl, CY+hl, Z_WORK,   ORI
    CX-hl, CY-hl, Z_WORK,   ORI    % cerrar
    CX-hl, CY-hl, Z_TRAVEL, ORI    % subir
];

%% ===================== PATRON 2: HEXAGONO (1 celda honeycomb) =====================
hex_r = 10; % mm — radio del hexagono
ang_deg = 0:60:360;
ang_rad = ang_deg * pi / 180;
hx = CX + hex_r * cos(ang_rad);
hy = CY + hex_r * sin(ang_rad);
n_v = numel(ang_deg);

pts_hexagono = zeros(n_v + 3, 4);
pts_hexagono(1,:) = [hx(1), hy(1), Z_TRAVEL, ORI];  % arriba del vertice 1
pts_hexagono(2,:) = [hx(1), hy(1), Z_WORK,   ORI];  % bajar
for k = 1:n_v
    pts_hexagono(k+2,:) = [hx(k), hy(k), Z_WORK, ORI];
end
pts_hexagono(end,:) = [hx(1), hy(1), Z_TRAVEL, ORI]; % subir

%% ===================== PATRON 3: 3 HEXAGONOS CONTIGUOS =====================
% Simula 3 celdas vecinas del honeycomb para verificar desplazamiento
hex_spacing = hex_r * 1.5;  % separacion entre centros
centers = [
    CX - hex_spacing, CY,   0
    CX,               CY,   0
    CX + hex_spacing, CY,   0
];

pts_3hex = [];
for c = 1:3
    ccx = centers(c,1);
    ccy = centers(c,2);
    vx = ccx + hex_r * cos(ang_rad);
    vy = ccy + hex_r * sin(ang_rad);

    pts_3hex = [pts_3hex;
        vx(1), vy(1), Z_TRAVEL, ORI];  % ir arriba del vertice 1
    pts_3hex = [pts_3hex;
        vx(1), vy(1), Z_WORK,   ORI];  % bajar

    for k = 1:n_v
        pts_3hex = [pts_3hex; vx(k), vy(k), Z_WORK, ORI]; %#ok
    end

    pts_3hex = [pts_3hex;
        vx(1), vy(1), Z_TRAVEL, ORI];  % subir
end

%% ===================== PATRON 4: PRUEBA RANGO Z =====================
pts_z = [
    CX, CY, Z_MAX,  ORI    % ir al Z maximo
    CX, CY, Z_WORK, ORI    % bajar a trabajo
    CX, CY, Z_MIN,  ORI    % ir al Z minimo
    CX, CY, Z_MAX,  ORI    % regresar arriba
];

%% ===================== EXPORTAR CSV =====================
writematrix(pts_cuadrado, 'ar4_test_cuadrado.csv');
writematrix(pts_hexagono, 'ar4_test_hexagono.csv');
writematrix(pts_3hex,     'ar4_test_3hexagonos.csv');
writematrix(pts_z,        'ar4_test_rango_z.csv');

fprintf('============================================\n');
fprintf(' AR4 — Patrones de prueba generados\n');
fprintf('============================================\n');
fprintf(' Formato: X, Y, Z, Orientacion  (mm, mm, mm, deg)\n');
fprintf(' Rango Z: [%d, %d] mm\n', Z_MIN, Z_MAX);
fprintf(' Orientacion: %d° (fija, boquilla perpendicular)\n\n', ORI);

patterns = {'CUADRADO 20x20', 'HEXAGONO r=10', '3 HEXAGONOS', 'RANGO Z'};
files    = {'ar4_test_cuadrado.csv', 'ar4_test_hexagono.csv', ...
            'ar4_test_3hexagonos.csv', 'ar4_test_rango_z.csv'};
data     = {pts_cuadrado, pts_hexagono, pts_3hex, pts_z};

for p = 1:numel(patterns)
    d = data{p};
    fprintf('--- %s (%s, %d puntos) ---\n', patterns{p}, files{p}, size(d,1));
    fprintf('  X,       Y,       Z,     Ori\n');
    for r = 1:size(d,1)
        fprintf('  %7.2f, %7.2f, %7.2f, %3.0f\n', d(r,:));
    end
    fprintf('\n');
end

%% ===================== VISUALIZACION =====================
figure('Name','AR4 — Patrones de Prueba','Position',[100 100 1600 500]);

subplot(1,4,1);
plot3(pts_cuadrado(:,1), pts_cuadrado(:,2), pts_cuadrado(:,3), ...
    'b-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', 'b');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Cuadrado 20x20 mm'); grid on; axis equal; view(135, 25);
zlim([Z_MIN-20, Z_MAX+20]);

subplot(1,4,2);
plot3(pts_hexagono(:,1), pts_hexagono(:,2), pts_hexagono(:,3), ...
    'r-o', 'LineWidth', 1.5, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Hexagono r=10 mm'); grid on; axis equal; view(135, 25);
zlim([Z_MIN-20, Z_MAX+20]);

subplot(1,4,3);
plot3(pts_3hex(:,1), pts_3hex(:,2), pts_3hex(:,3), ...
    'm-o', 'LineWidth', 1.2, 'MarkerSize', 5, 'MarkerFaceColor', 'm');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('3 Hexagonos contiguos'); grid on; axis equal; view(135, 25);
zlim([Z_MIN-20, Z_MAX+20]);

subplot(1,4,4);
plot3(pts_z(:,1), pts_z(:,2), pts_z(:,3), ...
    'g-o', 'LineWidth', 2, 'MarkerSize', 8, 'MarkerFaceColor', 'g');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Prueba rango Z'); grid on; view(135, 25);
zlim([Z_MIN-20, Z_MAX+20]);

sgtitle('Patrones de Prueba para Robot AR4', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('Listo. Monse puede probar con estos CSV.\n');
fprintf('Si funciona, ejecutar export_honeycomb_for_ar4.m para la trayectoria real.\n');
