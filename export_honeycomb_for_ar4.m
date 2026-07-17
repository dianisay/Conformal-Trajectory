% export_honeycomb_for_ar4.m
% =========================================================================
% Genera la trayectoria conformal del honeycomb y la exporta como CSV
% para el robot AR4 6-DOF.
%
% Pipeline:
%   1. Carga el STL del scaffold y detecta el void
%   2. Genera la malla hexagonal en espacio UV
%   3. Optimiza el orden de visita (TSP) para minimizar viaje muerto
%   4. Genera la trayectoria en UV (outline + deposito)
%   5. Mapeo conformal UV -> XYZ (sobre la superficie del cilindro)
%   6. Traslada al workspace del AR4
%   7. Exporta CSV: X, Y, Z, Orientacion, Tipo
%
% SALIDA:
%   ar4_honeycomb_paredes.csv   — trayectoria de paredes TPU (perimetros)
%   ar4_honeycomb_relleno.csv   — trayectoria de relleno (deposito vertical)
%   ar4_honeycomb_completo.csv  — todo junto (paredes + relleno)
%
% Formato CSV:  X, Y, Z, Orientacion, Tipo
%   - X, Y, Z en mm
%   - Orientacion en grados (90° = perpendicular)
%   - Tipo: 0 = desplazamiento (boquilla arriba), 1 = deposicion
%
% REQUISITOS:
%   - scaffold_curved_void.stl en el mismo directorio
%   - MATLAB con Optimization Toolbox (para intlinprog / TSP)
%
% MODO MANUAL:
%   Si no tienes el STL, pon USE_STL = false y ajusta los parametros
%   del void manualmente en la seccion "MODO MANUAL".
% =========================================================================

close all; clear; clc;

%% =================== CONFIGURACION ===================

% --- Modo de entrada ---
USE_STL = true;   % true = cargar scaffold_curved_void.stl
                   % false = especificar geometria del void manualmente

% --- Parametros del workspace AR4 ---
AR4_Z_MAX     = 450;   % mm — Z maximo seguro
AR4_Z_MIN     = 270;   % mm — Z minimo seguro
AR4_Z_SURFACE = 300;   % mm — Z de la superficie del scaffold en el AR4
AR4_XY_CENTER = [0, 300]; % mm — centro XY del scaffold en el AR4
ORIENTACION   = 90;    % grados — orientacion fija (perpendicular)

% --- Parametros de deposicion ---
LAYER_HEIGHT = 0.4;    % mm — altura de cada capa
RISE_HEIGHT  = 20;     % mm — altura de desplazamiento sobre la superficie
NUM_PTS_EDGE = 20;     % puntos por arista del hexagono

% --- MODO MANUAL (solo si USE_STL = false) ---
MANUAL_CYL_R  = 100;   % mm — radio del cilindro
MANUAL_CYL_CY = 0;     % mm — centro Y del cilindro
MANUAL_CYL_CZ = 0;     % mm — centro Z del cilindro
MANUAL_VOID_THETA = [-25, 25]; % grados — rango angular del void
MANUAL_VOID_X     = [-20, 20]; % mm — rango axial del void
MANUAL_SHELL_THICK = 4; % mm — espesor de la pared

%% =================== 1. GEOMETRIA DEL SCAFFOLD ===================
if USE_STL
    fprintf('=== Cargando scaffold desde STL ===\n');
    scaffold_stl = 'scaffold_curved_void.stl';
    if ~isfile(scaffold_stl)
        error(['No se encontro %s en el directorio actual.\n' ...
               'Opciones:\n  1. Copiar el STL aqui\n  ' ...
               '2. Poner USE_STL = false y usar modo manual'], scaffold_stl);
    end

    TR = stlread(scaffold_stl);
    scaffold_pts  = TR.Points;
    scaffold_conn = TR.ConnectivityList;

    % Rotacion Rx90 (alinear eje del cilindro con X)
    Rx90 = [1 0 0; 0 0 -1; 0 1 0];
    scaffold_pts = (Rx90 * scaffold_pts')';

    % Ajuste de circulo (Kasa fit) en plano YZ
    A_fit = [scaffold_pts(:,2), scaffold_pts(:,3), ones(size(scaffold_pts,1),1)];
    b_fit = scaffold_pts(:,2).^2 + scaffold_pts(:,3).^2;
    x_fit = A_fit \ b_fit;
    cyl_cy = x_fit(1)/2;
    cyl_cz = x_fit(2)/2;
    cyl_R  = sqrt(x_fit(3) + cyl_cy^2 + cyl_cz^2);

    fprintf('  Cilindro: R=%.2f mm, centro YZ=[%.2f, %.2f]\n', cyl_R, cyl_cy, cyl_cz);

    % --- Deteccion del void por aristas agudas ---
    [theta_min, theta_max, x_void_min, x_void_max, shell_thickness] = ...
        detectVoid(scaffold_pts, scaffold_conn, cyl_cy, cyl_cz, cyl_R);
else
    fprintf('=== Modo manual (sin STL) ===\n');
    cyl_R  = MANUAL_CYL_R;
    cyl_cy = MANUAL_CYL_CY;
    cyl_cz = MANUAL_CYL_CZ;
    theta_min = deg2rad(MANUAL_VOID_THETA(1));
    theta_max = deg2rad(MANUAL_VOID_THETA(2));
    x_void_min = MANUAL_VOID_X(1);
    x_void_max = MANUAL_VOID_X(2);
    shell_thickness = MANUAL_SHELL_THICK;
end

void_u_range = [theta_min * cyl_R, theta_max * cyl_R];
void_v_range = [x_void_min, x_void_max];
void_width  = diff(void_u_range);
void_length = diff(void_v_range);

fprintf('  Void: %.1f mm (arco) x %.1f mm (axial), espesor=%.1f mm\n', ...
    void_width, void_length, shell_thickness);

%% =================== 2. MALLA HEXAGONAL EN UV ===================
hex_side = min(void_width, void_length) / 6;
Nx = max(2, floor(void_width  / (hex_side * 1.5)));
Ny = max(2, floor(void_length / (hex_side * sqrt(3))));
num_layers = ceil(shell_thickness / LAYER_HEIGHT);

fprintf('  Honeycomb: %dx%d celdas, lado=%.1f mm, %d capas\n', ...
    Nx, Ny, hex_side, num_layers);

G_uv = createGrid(Nx, Ny, hex_side);

% Centrar la malla sobre el void
grid_u_ext = max(G_uv(:,:,1),[],'all') - min(G_uv(:,:,1),[],'all');
grid_v_ext = max(G_uv(:,:,2),[],'all') - min(G_uv(:,:,2),[],'all');
u_offset = mean(void_u_range) - grid_u_ext / 2;
v_offset = mean(void_v_range) - grid_v_ext / 2;

% Indice de todas las celdas
cell_idx = [];
for iy = 1:Ny
    for ix = 1:Nx
        cell_idx = [cell_idx; ix iy]; %#ok
    end
end

%% =================== 3. OPTIMIZACION TSP ===================
n_cells = size(cell_idx, 1);
fprintf('  Optimizando orden de visita (%d celdas, TSP)...\n', n_cells);

cell_centroids = zeros(n_cells, 2);
for i = 1:n_cells
    cell_centroids(i,:) = squeeze(G_uv(cell_idx(i,2), cell_idx(i,1), :))';
end

z_penalty = 2 * RISE_HEIGHT;
D_tsp = zeros(n_cells);
for i = 1:n_cells
    for j = 1:n_cells
        if i ~= j
            D_tsp(i,j) = norm(cell_centroids(i,:) - cell_centroids(j,:)) + z_penalty;
        end
    end
end

tsp_order = solveTSP_MTZ(D_tsp, n_cells);
cell_idx = cell_idx(tsp_order, :);

seq_cost = 0; opt_cost = 0;
for i = 1:n_cells-1
    seq_cost = seq_cost + D_tsp(i, i+1);
    opt_cost = opt_cost + D_tsp(tsp_order(i), tsp_order(i+1));
end
fprintf('  TSP: secuencial=%.0f mm, optimo=%.0f mm (ahorro %.1f%%)\n', ...
    seq_cost, opt_cost, 100*(seq_cost - opt_cost)/seq_cost);

%% =================== 4. TRAYECTORIA UV ===================
fprintf('  Generando trayectorias UV...\n');

% --- 4a. Paredes (perimetros de hexagonos, capa por capa) ---
wall_traj_uv = [];
wall_type    = [];  % 0 = travel, 1 = deposition
pos = [0; 0; RISE_HEIGHT];

for i = 1:n_cells
    gx = cell_idx(i,1);
    gy = cell_idx(i,2);
    center = squeeze(G_uv(gy, gx, :))';
    pts = hexagonPerimeter(center, hex_side, NUM_PTS_EDGE);

    % Desplazamiento al primer vertice (arriba)
    target = [pts(1,:), RISE_HEIGHT]';
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    wall_traj_uv = [wall_traj_uv, seg]; %#ok
    wall_type = [wall_type, zeros(1, size(seg,2))]; %#ok
    pos = target;

    % Bajar a la superficie
    target(3) = 0;
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    wall_traj_uv = [wall_traj_uv, seg]; %#ok
    wall_type = [wall_type, zeros(1, size(seg,2))]; %#ok
    pos = target;

    % Trazar perimetro capa por capa (desde la superficie hacia adentro)
    for layer = 1:num_layers
        h_layer = -((layer - 1) * LAYER_HEIGHT);
        hex_3d = [pts, repmat(h_layer, size(pts,1), 1)]';
        wall_traj_uv = [wall_traj_uv, hex_3d]; %#ok
        wall_type = [wall_type, ones(1, size(hex_3d,2))]; %#ok
        pos = hex_3d(:,end);
    end

    % Subir
    target = [pts(1,:), RISE_HEIGHT]';
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    wall_traj_uv = [wall_traj_uv, seg]; %#ok
    wall_type = [wall_type, zeros(1, size(seg,2))]; %#ok
    pos = target;
end

% --- 4b. Relleno (deposito vertical en el centro de cada celda) ---
fill_traj_uv = [];
fill_type    = [];

for i = 1:n_cells
    gx = cell_idx(i,1);
    gy = cell_idx(i,2);
    center = squeeze(G_uv(gy, gx, :))';

    target = [center, RISE_HEIGHT]';
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    fill_traj_uv = [fill_traj_uv, seg]; %#ok
    fill_type = [fill_type, zeros(1, size(seg,2))]; %#ok
    pos = target;

    % Bajar al nivel de la superficie
    target = [center, 0]';
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    fill_traj_uv = [fill_traj_uv, seg]; %#ok
    fill_type = [fill_type, zeros(1, size(seg,2))]; %#ok
    pos = target;

    % Inyectar hacia abajo (toda la profundidad del shell)
    target = [center, -shell_thickness]';
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    fill_traj_uv = [fill_traj_uv, seg]; %#ok
    fill_type = [fill_type, ones(1, size(seg,2))]; %#ok
    pos = target;

    % Subir
    target = [center, RISE_HEIGHT]';
    seg = linePoints(pos, target, NUM_PTS_EDGE);
    fill_traj_uv = [fill_traj_uv, seg]; %#ok
    fill_type = [fill_type, zeros(1, size(seg,2))]; %#ok
    pos = target;
end

fprintf('  Paredes: %d puntos, Relleno: %d puntos\n', ...
    size(wall_traj_uv,2), size(fill_traj_uv,2));

%% =================== 5. MAPEO CONFORMAL UV -> XYZ ===================
fprintf('  Mapeo conformal UV -> XYZ...\n');

wall_xyz = uv2xyz_batch(wall_traj_uv, u_offset, v_offset, cyl_R, cyl_cy, cyl_cz);
fill_xyz = uv2xyz_batch(fill_traj_uv, u_offset, v_offset, cyl_R, cyl_cy, cyl_cz);

%% =================== 6. TRANSFORMAR AL WORKSPACE AR4 ===================
% La trayectoria esta en coordenadas del scaffold (mm).
% Trasladamos para que el centro de la superficie quede en AR4_XY_CENTER
% y la superficie del scaffold quede a Z = AR4_Z_SURFACE.

fprintf('  Trasladando al workspace AR4...\n');

% Centro de la trayectoria de paredes en el frame del scaffold
traj_all = [wall_xyz, fill_xyz];
centroid_xy = [mean(traj_all(1,:)), mean(traj_all(2,:))];

% La superficie del scaffold esta en Z ≈ cyl_cz + cyl_R (tope del cilindro)
surface_z = cyl_cz + cyl_R;

% Offsets
dx = AR4_XY_CENTER(1) - centroid_xy(1);
dy = AR4_XY_CENTER(2) - centroid_xy(2);
dz = AR4_Z_SURFACE - surface_z;

wall_ar4 = wall_xyz;
wall_ar4(1,:) = wall_ar4(1,:) + dx;
wall_ar4(2,:) = wall_ar4(2,:) + dy;
wall_ar4(3,:) = wall_ar4(3,:) + dz;

fill_ar4 = fill_xyz;
fill_ar4(1,:) = fill_ar4(1,:) + dx;
fill_ar4(2,:) = fill_ar4(2,:) + dy;
fill_ar4(3,:) = fill_ar4(3,:) + dz;

% Verificar que todo esta dentro del rango Z
all_z = [wall_ar4(3,:), fill_ar4(3,:)];
fprintf('  Rango Z resultante: [%.1f, %.1f] mm\n', min(all_z), max(all_z));

if min(all_z) < AR4_Z_MIN
    warning('Z minimo (%.1f) esta por debajo del limite AR4 (%d mm). Ajustar AR4_Z_SURFACE.', ...
        min(all_z), AR4_Z_MIN);
end
if max(all_z) > AR4_Z_MAX
    warning('Z maximo (%.1f) esta por encima del limite AR4 (%d mm). Ajustar AR4_Z_SURFACE.', ...
        max(all_z), AR4_Z_MAX);
end

%% =================== 7. EXPORTAR CSV ===================
fprintf('  Exportando CSV...\n');

% Formato: X, Y, Z, Orientacion, Tipo
export_wall = [wall_ar4', repmat(ORIENTACION, size(wall_ar4,2), 1), wall_type'];
export_fill = [fill_ar4', repmat(ORIENTACION, size(fill_ar4,2), 1), fill_type'];
export_all  = [export_wall; export_fill];

writematrix(export_wall, 'ar4_honeycomb_paredes.csv');
writematrix(export_fill, 'ar4_honeycomb_relleno.csv');
writematrix(export_all,  'ar4_honeycomb_completo.csv');

fprintf('\n============================================\n');
fprintf(' ARCHIVOS GENERADOS\n');
fprintf('============================================\n');
fprintf(' ar4_honeycomb_paredes.csv   — %d puntos (paredes TPU)\n', size(export_wall,1));
fprintf(' ar4_honeycomb_relleno.csv   — %d puntos (relleno hydrogel)\n', size(export_fill,1));
fprintf(' ar4_honeycomb_completo.csv  — %d puntos (todo junto)\n', size(export_all,1));
fprintf('\n Formato: X, Y, Z, Orientacion, Tipo\n');
fprintf('   Orientacion = %d° (fija)\n', ORIENTACION);
fprintf('   Tipo: 0 = desplazamiento, 1 = deposicion\n');
fprintf('   Unidades: mm, grados\n');
fprintf('\n Workspace AR4:\n');
fprintf('   Centro XY: [%.0f, %.0f] mm\n', AR4_XY_CENTER);
fprintf('   Rango Z: [%.1f, %.1f] mm (limites: [%d, %d])\n', ...
    min(all_z), max(all_z), AR4_Z_MIN, AR4_Z_MAX);
fprintf('============================================\n');

% Primeros 20 puntos en consola (preview)
fprintf('\nPreview (primeros 20 puntos de paredes):\n');
fprintf('  X,        Y,        Z,      Ori,  Tipo\n');
for r = 1:min(20, size(export_wall,1))
    fprintf('  %8.2f, %8.2f, %8.2f, %4.0f, %4.0f\n', export_wall(r,:));
end
fprintf('  ... (%d puntos mas)\n', max(0, size(export_wall,1) - 20));

%% =================== 8. VISUALIZACION ===================
figure('Name','Trayectoria Honeycomb para AR4','Position',[50 50 1400 600]);

% Panel 1: Vista 3D
subplot(1,3,1);
dep_mask_w = wall_type == 1;
trv_mask_w = wall_type == 0;
plot3(wall_ar4(1,trv_mask_w), wall_ar4(2,trv_mask_w), wall_ar4(3,trv_mask_w), ...
    '.', 'Color', [0.7 0.7 0.7], 'MarkerSize', 2);
hold on;
plot3(wall_ar4(1,dep_mask_w), wall_ar4(2,dep_mask_w), wall_ar4(3,dep_mask_w), ...
    'b.', 'MarkerSize', 3);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Paredes TPU (azul = deposicion)');
grid on; axis equal; view(135, 25);
zlim([AR4_Z_MIN-10, AR4_Z_MAX+10]);

% Panel 2: Vista superior (XY)
subplot(1,3,2);
plot(wall_ar4(1,trv_mask_w), wall_ar4(2,trv_mask_w), '.', 'Color', [0.8 0.8 0.8], 'MarkerSize', 2);
hold on;
plot(wall_ar4(1,dep_mask_w), wall_ar4(2,dep_mask_w), 'b.', 'MarkerSize', 3);
plot(fill_ar4(1,:), fill_ar4(2,:), 'g.', 'MarkerSize', 4);
xlabel('X (mm)'); ylabel('Y (mm)');
title('Vista superior (azul=paredes, verde=relleno)');
grid on; axis equal;

% Panel 3: Perfil lateral (Y-Z) — curvatura visible
subplot(1,3,3);
plot(wall_ar4(2,dep_mask_w), wall_ar4(3,dep_mask_w), 'b.', 'MarkerSize', 3);
hold on;
dep_mask_f = fill_type == 1;
plot(fill_ar4(2,dep_mask_f), fill_ar4(3,dep_mask_f), 'g.', 'MarkerSize', 4);
yline(AR4_Z_MIN, 'r--', 'Z_{min}', 'LineWidth', 1.5);
yline(AR4_Z_MAX, 'r--', 'Z_{max}', 'LineWidth', 1.5);
xlabel('Y (mm)'); ylabel('Z (mm)');
title('Perfil lateral — curvatura conformal');
grid on; axis equal;

sgtitle(sprintf('Trayectoria Honeycomb Conformal para AR4 (%d celdas, %d puntos totales)', ...
    n_cells, size(export_all,1)), 'FontSize', 13, 'FontWeight', 'bold');


%% =================== FUNCIONES AUXILIARES ===================

function G = createGrid(Nx, Ny, hex_side)
    xSpacing = 1.5 * hex_side;
    ySpacing = hex_side * sqrt(3);
    [X, Y] = meshgrid(0:xSpacing:(Nx-1)*xSpacing, 0:ySpacing:(Ny-1)*ySpacing);
    Y(:, 1:2:end) = Y(:, 1:2:end) + ySpacing/2;
    G = zeros(Ny, Nx, 2);
    G(:,:,1) = X;
    G(:,:,2) = Y;
end

function pts = hexagonPerimeter(center, hex_side, n)
    if nargin < 3, n = 20; end
    cx = center(1); cy = center(2);
    R = hex_side;
    angles = [0 60 120 180 240 300] * pi/180;
    V = [cx + R*cos(angles)', cy + R*sin(angles)'];
    V = [V; V(1,:)];
    pts = [];
    for i = 1:6
        x = linspace(V(i,1), V(i+1,1), n);
        y = linspace(V(i,2), V(i+1,2), n);
        pts = [pts; x(1:end-1)', y(1:end-1)']; %#ok
    end
end

function seg = linePoints(start_pos, end_pos, n)
    seg = [linspace(start_pos(1), end_pos(1), n); ...
           linspace(start_pos(2), end_pos(2), n); ...
           linspace(start_pos(3), end_pos(3), n)];
end

function xyz = uv2xyz_batch(traj_uv, u_off, v_off, R, cy, cz)
    Npts = size(traj_uv, 2);
    xyz = zeros(3, Npts);
    for k = 1:Npts
        u = traj_uv(1,k) + u_off;
        v = traj_uv(2,k) + v_off;
        h = traj_uv(3,k);

        theta_k = u / R;
        Sx = v;
        Sy = cy + R * sin(theta_k);
        Sz = cz + R * cos(theta_k);

        nx = 0; ny = sin(theta_k); nz = cos(theta_k);
        xyz(:,k) = [Sx; Sy; Sz] + h * [nx; ny; nz];
    end
end

function [theta_min, theta_max, x_void_min, x_void_max, shell_thick] = ...
        detectVoid(scaffold_pts, scaffold_conn, cyl_cy, cyl_cz, cyl_R)
    % Detect void by finding sharp edges (dihedral angle > threshold)
    % and selecting the best internal component.

    nf = size(scaffold_conn, 1);
    face_normals = zeros(nf, 3);
    for f = 1:nf
        v1 = scaffold_pts(scaffold_conn(f,1),:);
        v2 = scaffold_pts(scaffold_conn(f,2),:);
        v3 = scaffold_pts(scaffold_conn(f,3),:);
        fn = cross(v2 - v1, v3 - v1);
        face_normals(f,:) = fn / (norm(fn) + eps);
    end

    edges_all = [scaffold_conn(:,[1 2]); scaffold_conn(:,[2 3]); scaffold_conn(:,[3 1])];
    faces_all = [(1:nf)'; (1:nf)'; (1:nf)'];
    edges_sorted = sort(edges_all, 2);
    [uniq_edges, ~, ic] = unique(edges_sorted, 'rows');

    angle_thr = 35;
    sharp_mask = false(size(uniq_edges,1),1);
    for e = 1:size(uniq_edges,1)
        fids = faces_all(ic == e);
        if numel(fids) == 2
            n1 = face_normals(fids(1),:);
            n2 = face_normals(fids(2),:);
            ang = acosd(max(-1, min(1, dot(n1,n2))));
            sharp_mask(e) = ang > angle_thr;
        end
    end
    sharp_edges = uniq_edges(sharp_mask,:);

    % Connected components via BFS
    all_v = unique(sharp_edges(:));
    vmap = containers.Map(num2cell(all_v), num2cell(1:numel(all_v)));
    adj = cell(numel(all_v),1);
    for k = 1:size(sharp_edges,1)
        a = vmap(sharp_edges(k,1));
        b = vmap(sharp_edges(k,2));
        adj{a}(end+1) = b;
        adj{b}(end+1) = a;
    end

    visited = false(numel(all_v),1);
    components = {};
    for s = 1:numel(all_v)
        if visited(s), continue; end
        queue = s; visited(s) = true; comp = s;
        while ~isempty(queue)
            cur = queue(1); queue(1) = [];
            for nb = adj{cur}
                if ~visited(nb)
                    visited(nb) = true;
                    queue(end+1) = nb; %#ok
                    comp(end+1) = nb;  %#ok
                end
            end
        end
        components{end+1} = all_v(comp); %#ok
    end

    % Score components to find the internal void
    R_c = cyl_R;
    xmin_all = min(scaffold_pts(:,1));
    xmax_all = max(scaffold_pts(:,1));
    edge_margin = 3;
    best_idx = -1; best_score = -inf;
    for c = 1:numel(components)
        vid = components{c};
        p = scaffold_pts(vid,:);
        axr = [min(p(:,1)), max(p(:,1))];
        nverts = size(p,1);
        touches_end = (axr(1) <= xmin_all + edge_margin) || (axr(2) >= xmax_all - edge_margin);
        mean_rad_err = mean(abs(sqrt((p(:,2)-cyl_cy).^2 + (p(:,3)-cyl_cz).^2) - R_c));
        theta_r = atan2(p(:,2)-cyl_cy, p(:,3)-cyl_cz);
        span_area = (max(theta_r)-min(theta_r))*R_c * (axr(2)-axr(1));
        score = 0;
        if ~touches_end, score = score + 100; end
        score = score + min(nverts, 80);
        score = score + max(0, 20 - mean_rad_err*4);
        score = score + min(span_area/40, 40);
        if score > best_score, best_score = score; best_idx = c; end
    end
    if best_idx < 0, error('No se pudo detectar el void en el STL.'); end

    void_vid = components{best_idx};
    void_pts = scaffold_pts(void_vid,:);
    theta_void = atan2(void_pts(:,2) - cyl_cy, void_pts(:,3) - cyl_cz);
    theta_min = min(theta_void);
    theta_max = max(theta_void);
    x_void_min = min(void_pts(:,1));
    x_void_max = max(void_pts(:,1));

    % Shell thickness from radial gap
    side_band = (theta_max - theta_min) * 0.1;
    left = abs(theta_void - theta_min) < side_band;
    right = abs(theta_void - theta_max) < side_band;
    if sum(left) >= 4
        side_r = sqrt((void_pts(left,2)-cyl_cy).^2 + (void_pts(left,3)-cyl_cz).^2);
    elseif sum(right) >= 4
        side_r = sqrt((void_pts(right,2)-cyl_cy).^2 + (void_pts(right,3)-cyl_cz).^2);
    else
        side_r = sqrt((void_pts(:,2)-cyl_cy).^2 + (void_pts(:,3)-cyl_cz).^2);
    end
    shell_thick = max(side_r) - min(side_r);

    fprintf('  Void detectado: theta=[%.1f, %.1f] deg, X=[%.1f, %.1f] mm, shell=%.1f mm\n', ...
        rad2deg(theta_min), rad2deg(theta_max), x_void_min, x_void_max, shell_thick);
end

function tour = solveTSP_MTZ(D, n)
    % Open-path TSP via MTZ formulation (intlinprog).
    % Adds a dummy node with zero-cost arcs to convert to closed tour.
    N = n + 1;
    D_ext = zeros(N);
    D_ext(1:n, 1:n) = D;

    n_x = N * N;
    n_u = N;
    n_vars = n_x + n_u;
    xidx = @(i,j) (i-1)*N + j;
    uidx = @(i) n_x + i;

    f = zeros(n_vars, 1);
    for i = 1:N
        for j = 1:N
            if i ~= j, f(xidx(i,j)) = D_ext(i,j); end
        end
    end

    intcon = 1:n_x;
    lb = zeros(n_vars, 1);
    ub = [ones(n_x, 1); (N-1)*ones(n_u, 1)];
    for i = 1:N, ub(xidx(i,i)) = 0; end
    lb(uidx(N)) = 0; ub(uidx(N)) = 0;

    Aeq = sparse(2*N, n_vars);
    beq = ones(2*N, 1);
    for i = 1:N
        for j = 1:N
            if i ~= j
                Aeq(i, xidx(i,j)) = 1;
                Aeq(N+i, xidx(j,i)) = 1;
            end
        end
    end

    mtz_pairs = [];
    for i = 1:n
        for j = 1:n
            if i ~= j, mtz_pairs = [mtz_pairs; i j]; end %#ok
        end
    end
    n_mtz = size(mtz_pairs, 1);
    A_ineq = sparse(n_mtz, n_vars);
    b_ineq = (N-1) * ones(n_mtz, 1);
    for k = 1:n_mtz
        i = mtz_pairs(k,1); j = mtz_pairs(k,2);
        A_ineq(k, uidx(i)) = 1;
        A_ineq(k, uidx(j)) = -1;
        A_ineq(k, xidx(i,j)) = N;
    end

    opts = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', 60);
    [x_sol, ~, exitflag] = intlinprog(f, intcon, A_ineq, b_ineq, Aeq, beq, lb, ub, opts);

    if exitflag <= 0
        warning('TSP no encontro solucion optima, usando orden secuencial.');
        tour = 1:n;
        return;
    end

    X = reshape(round(x_sol(1:n_x)), [N, N]);
    tour_full = zeros(1, N);
    tour_full(1) = N;
    for step = 2:N
        curr = tour_full(step-1);
        nxt = find(X(curr,:) > 0.5, 1);
        tour_full(step) = nxt;
    end
    tour = tour_full(tour_full <= n);
end
