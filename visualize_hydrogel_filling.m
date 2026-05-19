%% visualize_hydrogel_filling.m
% Two-stage visualization of conformal honeycomb repair:
%   Stage 1: TPU honeycomb wall formation (hexagon perimeters deposited layer by layer)
%   Stage 2: Hydrogel injection (cells fill from center outward)
%
% This script reuses the scaffold geometry and honeycomb grid from
% MuffinFresa_ConformalMapping.m but generates independent figures
% suitable for the article.

close all; clc;

%% =============== PARAMETERS ===============
% Scaffold (semi-cylinder)
cyl_R  = 100;       % mm — cylinder radius
cyl_cy = 0;         % mm — cylinder center Y
cyl_cz = 0;         % mm — cylinder center Z

% Void region (UV bounds on cylinder)
theta_min = deg2rad(-25);
theta_max = deg2rad(25);
x_void_min = -20;   % mm (axial)
x_void_max = 20;    % mm (axial)

void_u_range = [theta_min * cyl_R, theta_max * cyl_R];
void_v_range = [x_void_min, x_void_max];
void_width  = diff(void_u_range);
void_length = diff(void_v_range);

% Honeycomb
hex_side = min(void_width, void_length) / 6;
Nx = max(2, floor(void_width / (hex_side * 1.5)));
Ny = max(2, floor(void_length / (hex_side * sqrt(3))));
shell_thickness = 4;   % mm
layer_height = 0.4;    % mm
num_layers = ceil(shell_thickness / layer_height);

fprintf('Grid: %dx%d cells, hex_side=%.1f mm, %d layers\n', Nx, Ny, hex_side, num_layers);

%% =============== GENERATE HONEYCOMB IN UV SPACE ===============
G_uv = createGrid_local(Nx, Ny, hex_side);

% Center on void
grid_u_extent = max(G_uv(:,:,1),[],'all') - min(G_uv(:,:,1),[],'all');
grid_v_extent = max(G_uv(:,:,2),[],'all') - min(G_uv(:,:,2),[],'all');
u_offset = mean(void_u_range) - grid_u_extent / 2;
v_offset = mean(void_v_range) - grid_v_extent / 2;

%% =============== MAP HEXAGONS TO 3D (CYLINDER SURFACE) ===============
% For each cell, compute the 3D perimeter points on the cylinder surface
% at multiple layers (from outer surface inward).

n_cells = Nx * Ny;
cell_data = struct('perimeter_3d', {}, 'center_3d', {}, 'fill_pts', {});

cell_idx = 0;
for iy = 1:Ny
    for ix = 1:Nx
        cell_idx = cell_idx + 1;
        center_uv = squeeze(G_uv(iy, ix, :))';
        pts_uv = hexagonPerimeter_local(center_uv, hex_side, 30);

        % Map each layer to 3D
        all_layers_3d = [];
        for layer = 1:num_layers
            h_layer = -((layer - 1) * layer_height);
            pts_3d = uv2xyz(pts_uv, h_layer, cyl_R, cyl_cy, cyl_cz, u_offset, v_offset);
            all_layers_3d = [all_layers_3d; pts_3d];
        end
        cell_data(cell_idx).perimeter_3d = all_layers_3d;

        % Cell center on surface (for injection point)
        center_3d = uv2xyz(center_uv, 0, cyl_R, cyl_cy, cyl_cz, u_offset, v_offset);
        cell_data(cell_idx).center_3d = center_3d;

        % Fill points: dense grid inside hexagon at all layers (hydrogel volume)
        fill_uv = hexFillPoints(center_uv, hex_side, 0.85);
        fill_all = [];
        for layer = 1:num_layers
            h_layer = -((layer - 1) * layer_height);
            fill_3d = uv2xyz(fill_uv, h_layer, cyl_R, cyl_cy, cyl_cz, u_offset, v_offset);
            fill_all = [fill_all; fill_3d];
        end
        cell_data(cell_idx).fill_pts = fill_all;
    end
end

%% =============== SCAFFOLD SURFACE (for background) ===============
theta_surf = linspace(theta_min - deg2rad(10), theta_max + deg2rad(10), 60);
x_surf = linspace(x_void_min - 10, x_void_max + 10, 40);
[TH_s, X_s] = meshgrid(theta_surf, x_surf);
Y_s = cyl_cy + cyl_R * sin(TH_s);
Z_s = cyl_cz + cyl_R * cos(TH_s);

%% =============== FIGURE 1: STAGE 1 — TPU HONEYCOMB FORMATION ===============
fig1 = figure('Name','Stage 1: TPU Honeycomb Formation','Position',[50 50 1400 550]);

% --- Panel A: Partial deposition (first few cells) ---
subplot(1,3,1);
surf(X_s, Y_s, Z_s, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'FaceColor', [0.8 0.75 0.6]);
hold on;
n_partial = ceil(n_cells * 0.4);  % show ~40% deposited
for c = 1:n_partial
    pts = cell_data(c).perimeter_3d;
    plot3(pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 1.0);
end
% Show nozzle at current deposition point
last_pt = cell_data(n_partial).perimeter_3d(end,:);
plot3(last_pt(1), last_pt(2), last_pt(3), 'rv', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
axis equal; grid on; view(135, 25);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('(a) Deposition in progress (~40%)');

% --- Panel B: Complete honeycomb structure ---
subplot(1,3,2);
surf(X_s, Y_s, Z_s, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'FaceColor', [0.8 0.75 0.6]);
hold on;
for c = 1:n_cells
    pts = cell_data(c).perimeter_3d;
    plot3(pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 1.0);
end
axis equal; grid on; view(135, 25);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('(b) Complete TPU honeycomb');

% --- Panel C: Top view of complete honeycomb ---
subplot(1,3,3);
surf(X_s, Y_s, Z_s, 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'FaceColor', [0.8 0.75 0.6]);
hold on;
for c = 1:n_cells
    pts = cell_data(c).perimeter_3d;
    plot3(pts(:,1), pts(:,2), pts(:,3), 'b-', 'LineWidth', 1.2);
end
axis equal; grid on; view(0, 90);
xlabel('X (mm)'); ylabel('Y (mm)');
title('(c) Top view — cell regularity');

sgtitle('Stage 1: Transparent TPU Honeycomb Wall Deposition', 'FontSize', 13, 'FontWeight', 'bold');

%% =============== FIGURE 2: STAGE 2 — HYDROGEL INJECTION ===============
fig2 = figure('Name','Stage 2: Hydrogel Filling','Position',[50 100 1400 550]);

% --- Panel A: Partial fill (first few cells injected) ---
subplot(1,3,1);
surf(X_s, Y_s, Z_s, 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'FaceColor', [0.8 0.75 0.6]);
hold on;
% Draw all honeycomb walls (transparent blue = TPU)
for c = 1:n_cells
    pts = cell_data(c).perimeter_3d;
    plot3(pts(:,1), pts(:,2), pts(:,3), '-', 'Color', [0.3 0.6 1 0.5], 'LineWidth', 0.8);
end
% Fill first ~30% of cells with hydrogel
n_filled = ceil(n_cells * 0.3);
for c = 1:n_filled
    fp = cell_data(c).fill_pts;
    scatter3(fp(:,1), fp(:,2), fp(:,3), 4, 'g', 'filled', 'MarkerFaceAlpha', 0.6);
end
% Injection needle at current cell
cur_center = cell_data(n_filled + 1).center_3d;
quiver3(cur_center(1), cur_center(2), cur_center(3) + 15, 0, 0, -12, 0, ...
    'Color', [0.8 0 0], 'LineWidth', 2.5, 'MaxHeadSize', 0.8);
plot3(cur_center(1), cur_center(2), cur_center(3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
axis equal; grid on; view(135, 25);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('(a) Hydrogel injection (~30%)');

% --- Panel B: All cells filled ---
subplot(1,3,2);
surf(X_s, Y_s, Z_s, 'FaceAlpha', 0.10, 'EdgeColor', 'none', 'FaceColor', [0.8 0.75 0.6]);
hold on;
for c = 1:n_cells
    pts = cell_data(c).perimeter_3d;
    plot3(pts(:,1), pts(:,2), pts(:,3), '-', 'Color', [0.3 0.6 1 0.5], 'LineWidth', 0.8);
end
for c = 1:n_cells
    fp = cell_data(c).fill_pts;
    scatter3(fp(:,1), fp(:,2), fp(:,3), 4, 'g', 'filled', 'MarkerFaceAlpha', 0.6);
end
axis equal; grid on; view(135, 25);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('(b) Fully filled — volumetric repair');

% --- Panel C: Cross-section view (YZ plane) showing layers ---
subplot(1,3,3);
hold on; grid on;
section_x = mean([x_void_min, x_void_max]);
section_tol = 1.5;  % mm

% Draw scaffold arc
th_ref = linspace(theta_min - deg2rad(5), theta_max + deg2rad(5), 200);
plot(cyl_cy + cyl_R*sin(th_ref), cyl_cz + cyl_R*cos(th_ref), '-', ...
    'Color', [0.6 0.5 0.3], 'LineWidth', 2.5);

% Draw honeycomb walls in cross-section
for c = 1:n_cells
    pts = cell_data(c).perimeter_3d;
    in_section = abs(pts(:,1) - section_x) < section_tol;
    if any(in_section)
        plot(pts(in_section,2), pts(in_section,3), 'b.', 'MarkerSize', 6);
    end
end
% Draw hydrogel fill in cross-section
for c = 1:n_cells
    fp = cell_data(c).fill_pts;
    in_section = abs(fp(:,1) - section_x) < section_tol;
    if any(in_section)
        plot(fp(in_section,2), fp(in_section,3), '.', 'Color', [0.2 0.8 0.2], 'MarkerSize', 5);
    end
end
xlabel('Y (mm)'); ylabel('Z (mm)');
title('(c) YZ cross-section at X=0');
legend('Scaffold surface', 'TPU walls', 'Hydrogel fill', 'Location', 'best');
axis equal;

sgtitle('Stage 2: Hydrogel (Gelatin) Cell Filling', 'FontSize', 13, 'FontWeight', 'bold');

%% =============== FIGURE 3: COMBINED RESULT — BOTH STAGES ===============
fig3 = figure('Name','Multi-Material Result','Position',[80 80 900 700]);

surf(X_s, Y_s, Z_s, 'FaceAlpha', 0.12, 'EdgeColor', 'none', 'FaceColor', [0.8 0.75 0.6]);
hold on;

% Honeycomb walls (TPU — blue, semi-transparent)
for c = 1:n_cells
    pts = cell_data(c).perimeter_3d;
    plot3(pts(:,1), pts(:,2), pts(:,3), '-', 'Color', [0.1 0.4 0.9 0.7], 'LineWidth', 1.2);
end

% Hydrogel fill (green, semi-transparent)
for c = 1:n_cells
    fp = cell_data(c).fill_pts;
    scatter3(fp(:,1), fp(:,2), fp(:,3), 3, [0.2 0.8 0.3], 'filled', 'MarkerFaceAlpha', 0.4);
end

axis equal; grid on; view(135, 25);
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title({'Multi-Material Conformal Void Repair'; ...
       'Transparent TPU honeycomb (blue) + Hydrogel fill (green)'});
legend('Scaffold surface', 'TPU walls', 'Hydrogel (gelatin)', 'Location', 'best');

fprintf('\nDone. Three figures generated.\n');
fprintf('  Fig 1: Stage 1 — TPU honeycomb formation\n');
fprintf('  Fig 2: Stage 2 — Hydrogel injection and filling\n');
fprintf('  Fig 3: Combined multi-material result\n');

%% =============== LOCAL HELPER FUNCTIONS ===============

function G = createGrid_local(Nx, Ny, hex_side)
    xSpacing = 1.5 * hex_side;
    ySpacing = hex_side * sqrt(3);
    [X, Y] = meshgrid(0:xSpacing:(Nx-1)*xSpacing, 0:ySpacing:(Ny-1)*ySpacing);
    Y(:, 1:2:end) = Y(:, 1:2:end) + ySpacing/2;
    G = zeros(Ny, Nx, 2);
    G(:,:,1) = X;
    G(:,:,2) = Y;
end

function pts = hexagonPerimeter_local(center, hex_side, n)
    if nargin < 3, n = 20; end
    cx = center(1);
    cy = center(2);
    R = hex_side;
    angles = (0:60:300) * pi/180;
    V = [cx + R*cos(angles)', cy + R*sin(angles)'];
    V = [V; V(1,:)];
    pts = [];
    for i = 1:6
        x = linspace(V(i,1), V(i+1,1), n);
        y = linspace(V(i,2), V(i+1,2), n);
        pts = [pts; x(1:end-1)', y(1:end-1)'];
    end
end

function pts_3d = uv2xyz(pts_uv, h, R, cy, cz, u_off, v_off)
    % Maps UV points (Nx2) + height to 3D cylinder surface
    n = size(pts_uv, 1);
    pts_3d = zeros(n, 3);
    for k = 1:n
        u = pts_uv(k,1) + u_off;
        v = pts_uv(k,2) + v_off;
        theta_k = u / R;
        Sx = v;
        Sy = cy + R * sin(theta_k);
        Sz = cz + R * cos(theta_k);
        nx = 0; ny = sin(theta_k); nz = cos(theta_k);
        pts_3d(k,:) = [Sx, Sy, Sz] + h * [nx, ny, nz];
    end
end

function fill_pts = hexFillPoints(center, hex_side, shrink)
    % Generate dense interior points for a hexagonal cell
    if nargin < 3, shrink = 0.85; end
    cx = center(1);
    cy = center(2);
    r = hex_side * shrink;

    % Grid of candidate points
    n_grid = 12;
    xx = linspace(cx - r, cx + r, n_grid);
    yy = linspace(cy - r * sqrt(3)/2, cy + r * sqrt(3)/2, n_grid);
    [Xg, Yg] = meshgrid(xx, yy);
    candidates = [Xg(:), Yg(:)];

    % Keep only points inside the hexagon
    angles = (0:60:300) * pi/180;
    V = [cx + hex_side*cos(angles)', cy + hex_side*sin(angles)'];
    in = inpolygon(candidates(:,1), candidates(:,2), V(:,1), V(:,2));
    fill_pts = candidates(in, :);
end
