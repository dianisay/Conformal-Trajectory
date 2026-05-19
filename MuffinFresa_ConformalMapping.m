close all; clear; clc;

mm = 1e-3;

%================== 1. ROBOT MODEL (8-DOF: XY Gantry + 6R Arm) ==================
arm = importrobot("mycobot.urdf", DataFormat="row");


mc = rigidBodyTree(DataFormat="row");
mc.BaseName = 'world_root';

% Shared workspace reference point for box and scaffold
wsRefX = -0.5;  wsRefY = -0.5;  wsRefZ = 0.1;
wsRef = rigidBody("workspace_ref");
wsRefJnt = rigidBodyJoint("ws_ref_fixed", "fixed");
setFixedTransform(wsRefJnt, trvec2tform([wsRefX, wsRefY, wsRefZ]));
wsRef.Joint = wsRefJnt;
addBody(mc, wsRef, mc.BaseName);

% Box visual (child of workspace_ref)
boxBody = rigidBody("box_visual");
boxJnt = rigidBodyJoint("box_fixed", "fixed");
setFixedTransform(boxJnt, trvec2tform([0, 0, 0]));
boxBody.Joint = boxJnt;
Tvis_box = axang2tform([1 0 0 -pi/2]);
addVisual(boxBody, "Mesh", {"BOXCOMPLETOOO10STL.stl", [0.013 0.013 0.013]}, Tvis_box);
addBody(mc, boxBody, "workspace_ref");

% Scaffold visual (child of workspace_ref)
scaffoldOffX = 0.5;  scaffoldOffY = 0.5;  scaffoldOffZ = -0.45;
scaffoldBody = rigidBody("scaffold_visual");
scaffoldJnt = rigidBodyJoint("scaffold_fixed", "fixed");
setFixedTransform(scaffoldJnt, trvec2tform([scaffoldOffX, scaffoldOffY, scaffoldOffZ]));
scaffoldBody.Joint = scaffoldJnt;
Tvis_scaffold = axang2tform([1 0 0 -pi/2]);
addVisual(scaffoldBody, "Mesh", {"scaffold_curved_void.stl", [0.001 0.001 0.001]}, Tvis_scaffold);
addBody(mc, scaffoldBody, "workspace_ref");

gantryWorldX = -0.5;  gantryWorldY = -0.5;  gantryWorldZ = -0.9;
gantryX = rigidBody("gantry_x");
jx = rigidBodyJoint("prism_x", "prismatic");
jx.JointAxis = [1 0 0];
jx.HomePosition = 0;
setFixedTransform(jx, trvec2tform([gantryWorldX, gantryWorldY, gantryWorldZ]));
gantryX.Joint = jx;
addBody(mc, gantryX, mc.BaseName);

gantryY = rigidBody("gantry_y");
jy = rigidBodyJoint("prism_y", "prismatic");
jy.JointAxis = [0 1 0];
jy.HomePosition = 0;
gantryY.Joint = jy;
addBody(mc, gantryY, "gantry_x");

addSubtree(mc, "gantry_y", arm, ReplaceBase=false);
showdetails(mc)

rbt = rigidBodyTree(DataFormat="row");
floatingBaseBody = rigidBody("floatingBase");
floatingBaseBody.Joint = rigidBodyJoint("j1","floating");
addBody(rbt,floatingBaseBody,rbt.BaseName);
rbt.BaseName = 'world';
addSubtree(rbt,"floatingBase",mc,ReplaceBase=false);

baseOrientation = eul2quat([0 pi 0]);
basePosition = [0.0 0.0 0.0];
floatingRBTConfig = [baseOrientation, basePosition, homeConfiguration(mc)];

% (Robot model figure is deferred — static scaffold+honeycomb shown after
%  trajectory generation in Section 7b.)

%================== 2. LOAD SCAFFOLD STL & ANALYZE SURFACE ==================
% Load the pre-manufactured scaffold with the void (from CAD export)
scaffold_stl = 'scaffold_curved_void.stl';
fprintf('Loading scaffold: %s\n', scaffold_stl);
TR_scaffold = stlread(scaffold_stl);

scaffold_pts  = TR_scaffold.Points;          % Nx3 vertices (mm)
scaffold_conn = TR_scaffold.ConnectivityList; % Mx3 triangle indices

fprintf('  Vertices: %d, Triangles: %d\n', size(scaffold_pts,1), size(scaffold_conn,1));
fprintf('  Original bounding box:\n');
fprintf('    X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f] mm\n', ...
    min(scaffold_pts(:,1)), max(scaffold_pts(:,1)), ...
    min(scaffold_pts(:,2)), max(scaffold_pts(:,2)), ...
    min(scaffold_pts(:,3)), max(scaffold_pts(:,3)));

%--- Rotate STL: Rx90 (verified via preview_stl_rotations_90.m) ---
Rx90 = [1 0 0; 0 0 -1; 0 1 0];
scaffold_pts = (Rx90 * scaffold_pts')';
fprintf('  Applied Rx90 rotation.\n');
fprintf('  Rotated bounding box:\n');
fprintf('    X[%.1f, %.1f] Y[%.1f, %.1f] Z[%.1f, %.1f] mm\n', ...
    min(scaffold_pts(:,1)), max(scaffold_pts(:,1)), ...
    min(scaffold_pts(:,2)), max(scaffold_pts(:,2)), ...
    min(scaffold_pts(:,3)), max(scaffold_pts(:,3)));

%--- Extract cylinder geometry (axis X, Kasa circle fit in YZ plane) ---
A_fit = [scaffold_pts(:,2), scaffold_pts(:,3), ones(size(scaffold_pts,1),1)];
b_fit = scaffold_pts(:,2).^2 + scaffold_pts(:,3).^2;
x_fit = A_fit \ b_fit;
cyl_cy = x_fit(1)/2;
cyl_cz = x_fit(2)/2;
cyl_R  = sqrt(x_fit(3) + cyl_cy^2 + cyl_cz^2);

scaffold_radius = cyl_R;
scaffold_length = max(scaffold_pts(:,1)) - min(scaffold_pts(:,1));  % axial = X
scaffold_width  = pi * scaffold_radius;

fprintf('  Cylinder axis: X, center YZ=[%.2f, %.2f], R=%.2f mm\n', cyl_cy, cyl_cz, cyl_R);
fprintf('  Scaffold length (axial X): %.1f mm, width (arc): %.1f mm\n', ...
    scaffold_length, scaffold_width);

%--- Detect void via sharp-feature edges (dihedral angle method) ---
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
sharp_edge_mask = false(size(uniq_edges,1),1);
for e = 1:size(uniq_edges,1)
    face_ids = faces_all(ic == e);
    if numel(face_ids) == 2
        n1 = face_normals(face_ids(1),:);
        n2 = face_normals(face_ids(2),:);
        ang = acosd(max(-1, min(1, dot(n1,n2))));
        sharp_edge_mask(e) = ang > angle_thr;
    end
end
sharp_edges = uniq_edges(sharp_edge_mask,:);
fprintf('  Sharp edges (dihedral > %d deg): %d\n', angle_thr, size(sharp_edges,1));

% Connected components on sharp-edge graph
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
                queue(end+1) = nb; %#ok<AGROW>
                comp(end+1) = nb;  %#ok<AGROW>
            end
        end
    end
    components{end+1} = all_v(comp); %#ok<AGROW>
end
fprintf('  Sharp-edge components: %d\n', numel(components));

% Score components — pick the rectangular void (internal, on curved radius)
R_cyl = scaffold_radius;
xmin_all = min(scaffold_pts(:,1));  % axial = X
xmax_all = max(scaffold_pts(:,1));
edge_margin = 3;

best_idx = -1; best_score = -inf;
for c = 1:numel(components)
    vid = components{c};
    p = scaffold_pts(vid,:);
    axr = [min(p(:,1)), max(p(:,1))];  % axial (X) range
    nverts = size(p,1);
    touches_ax_end = (axr(1) <= xmin_all + edge_margin) || (axr(2) >= xmax_all - edge_margin);
    mean_rad_err = mean(abs(sqrt((p(:,2)-cyl_cy).^2 + (p(:,3)-cyl_cz).^2) - R_cyl));
    theta_r = atan2(p(:,2)-cyl_cy, p(:,3)-cyl_cz);
    span_area = (max(theta_r)-min(theta_r))*R_cyl * (axr(2)-axr(1));
    score = 0;
    if ~touches_ax_end, score = score + 100; end
    score = score + min(nverts, 80);
    score = score + max(0, 20 - mean_rad_err*4);
    score = score + min(span_area/40, 40);
    if score > best_score
        best_score = score; best_idx = c;
    end
end
if best_idx < 0, error('Could not select a candidate component for the void.'); end

void_vid = components{best_idx};
void_pts = scaffold_pts(void_vid,:);
fprintf('  Selected void component %d: %d vertices\n', best_idx, size(void_pts,1));

% UV bounds of the void on the cylinder surface (axis X)
theta_void = atan2(void_pts(:,2) - cyl_cy, void_pts(:,3) - cyl_cz);
theta_min = min(theta_void);  theta_max = max(theta_void);
x_void_min = min(void_pts(:,1));  x_void_max = max(void_pts(:,1));

void_u_range = [theta_min * R_cyl, theta_max * R_cyl];  % arc-length
void_v_range = [x_void_min, x_void_max];                % axial = X
void_width  = diff(void_u_range);
void_length = diff(void_v_range);
fprintf('  Void size: %.1f mm (arc) x %.1f mm (axial)\n', void_width, void_length);
fprintf('  Void center UV: [%.1f, %.1f] mm\n', mean(void_u_range), mean(void_v_range));

% Shell thickness: radial gap of void vertices on one angular side.
% Radius is measured in YZ plane from the true center.
side_band = (theta_max - theta_min) * 0.1;
left_side = abs(theta_void - theta_min) < side_band;
right_side = abs(theta_void - theta_max) < side_band;
if sum(left_side) >= 4
    side_r = sqrt((void_pts(left_side,2)-cyl_cy).^2 + (void_pts(left_side,3)-cyl_cz).^2);
elseif sum(right_side) >= 4
    side_r = sqrt((void_pts(right_side,2)-cyl_cy).^2 + (void_pts(right_side,3)-cyl_cz).^2);
else
    side_r = sqrt((void_pts(:,2)-cyl_cy).^2 + (void_pts(:,3)-cyl_cz).^2);
end
shell_thickness = max(side_r) - min(side_r);
fprintf('  Shell thickness (radial gap on void side): %.1f mm\n', shell_thickness);

% Build void edge list for visualization
void_set = false(size(scaffold_pts,1),1); void_set(void_vid) = true;
void_edge_mask = void_set(sharp_edges(:,1)) & void_set(sharp_edges(:,2));
void_sharp_edges = sharp_edges(void_edge_mask,:);

% Ideal rectangle corners on cylinder surface (axis X)
th_corners = [theta_min theta_max theta_max theta_min theta_min];
ax_corners = [x_void_min x_void_min x_void_max x_void_max x_void_min];
rect3d = [ax_corners(:), cyl_cy + R_cyl*sin(th_corners(:)), cyl_cz + R_cyl*cos(th_corners(:))];

% Visualization: scaffold + void detection
figure('Name','Scaffold STL & Void Detection','Position',[50 50 1500 500]);

subplot(1,3,1);
trisurf(scaffold_conn, scaffold_pts(:,1), scaffold_pts(:,2), scaffold_pts(:,3), ...
    'FaceAlpha', 0.15, 'EdgeColor', [0.65 0.65 0.65], 'FaceColor', [0.85 0.85 0.85]);
hold on;
for k = 1:size(void_sharp_edges,1)
    p1 = scaffold_pts(void_sharp_edges(k,1),:);
    p2 = scaffold_pts(void_sharp_edges(k,2),:);
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r-', 'LineWidth', 2.5);
end
plot3(rect3d(:,1), rect3d(:,2), rect3d(:,3), 'g-', 'LineWidth', 2.5);
title('Detected Void (red) on Scaffold');
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
axis equal; grid on; view(135, 25);

subplot(1,3,2);
hold on;
for k = 1:size(void_sharp_edges,1)
    p1 = scaffold_pts(void_sharp_edges(k,1),:);
    p2 = scaffold_pts(void_sharp_edges(k,2),:);
    t1 = rad2deg(atan2(p1(2)-cyl_cy, p1(3)-cyl_cz));
    t2 = rad2deg(atan2(p2(2)-cyl_cy, p2(3)-cyl_cz));
    plot([t1 t2], [p1(1) p2(1)], 'r-', 'LineWidth', 2);
end
rectangle('Position', [rad2deg(theta_min), x_void_min, ...
    rad2deg(theta_max-theta_min), x_void_max-x_void_min], ...
    'EdgeColor', 'g', 'LineWidth', 2);
title('Void Boundary (\theta, X_{axial})');
xlabel('\theta (deg)'); ylabel('X (mm)'); grid on;

subplot(1,3,3);
hold on;
for k = 1:size(void_sharp_edges,1)
    p1 = scaffold_pts(void_sharp_edges(k,1),:);
    p2 = scaffold_pts(void_sharp_edges(k,2),:);
    t1 = rad2deg(atan2(p1(2)-cyl_cy, p1(3)-cyl_cz));
    t2 = rad2deg(atan2(p2(2)-cyl_cy, p2(3)-cyl_cz));
    plot([t1 t2], [p1(2) p2(2)], 'r-', 'LineWidth', 2);
end
th_ref = linspace(rad2deg(theta_min), rad2deg(theta_max), 100);
plot(th_ref, cyl_cy + R_cyl*sind(th_ref), 'g--', 'LineWidth', 1.5);
title('Void Boundary (\theta, Y) — green = cy + R\cdotsin(\theta)');
xlabel('\theta (deg)'); ylabel('Y (mm)'); grid on;

%================== 3. CONFORMAL HONEYCOMB GENERATION IN UV SPACE ==================
% Size hexagonal grid to fill the detected void
hex_side = min(void_width, void_length) / 6;  % adaptive hex size based on void
Nx = max(2, floor(void_width / (hex_side * 1.5)));
Ny = max(2, floor(void_length / (hex_side * sqrt(3))));
fprintf('  Honeycomb grid: %dx%d cells, side=%.1f mm\n', Nx, Ny, hex_side);

num_points = 20;
rise = 20;         % mm — travel altitude above surface
wall_height = shell_thickness;
layer_height = 0.4;% mm
num_layers = ceil(wall_height / layer_height);
fprintf('  Wall height = shell thickness: %.1f mm (%d layers)\n', wall_height, num_layers);


% Trace all cells as both outline and fill (full void infill)
outline_idx = [];
fill_idx = [];
for iy = 1:Ny
    for ix = 1:Nx
        outline_idx = [outline_idx; ix iy];
    end
end
fill_idx = outline_idx;  % fill every cell

% Generate flat hex grid in UV parameter space
G_uv = createGrid(Nx, Ny, hex_side);

%================== 3b. TSP CELL VISITATION OPTIMIZATION ==================
% Optimize the order in which honeycomb cells are visited to minimize
% non-productive travel (rise -> translate -> lower between cells).

n_cells = size(outline_idx, 1);
fprintf('  Optimizing cell visitation order (%d cells)...\n', n_cells);

% Compute cell centroids in UV space
cell_centroids = zeros(n_cells, 2);
for i = 1:n_cells
    gx = outline_idx(i,1);
    gy = outline_idx(i,2);
    cell_centroids(i,:) = squeeze(G_uv(gy, gx, :))';
end

% Build distance matrix (Euclidean + 2*rise penalty per hop)
z_rise_penalty = 2 * rise;
D_tsp = zeros(n_cells, n_cells);
for i = 1:n_cells
    for j = 1:n_cells
        if i ~= j
            D_tsp(i,j) = norm(cell_centroids(i,:) - cell_centroids(j,:)) + z_rise_penalty;
        end
    end
end

% Solve open-path TSP via MTZ formulation using intlinprog
tsp_order = solveTSP_MTZ(D_tsp, n_cells);

% Reorder cell indices according to optimal tour
outline_idx = outline_idx(tsp_order, :);
fill_idx = outline_idx;

% Report savings vs sequential order
seq_cost = 0;
for i = 1:n_cells-1
    seq_cost = seq_cost + D_tsp(i, i+1);
end
opt_cost = 0;
for i = 1:n_cells-1
    opt_cost = opt_cost + D_tsp(tsp_order(i), tsp_order(i+1));
end
fprintf('  TSP result: sequential=%.1f mm, optimal=%.1f mm (saved %.1f%%)\n', ...
    seq_cost, opt_cost, 100*(seq_cost - opt_cost)/seq_cost);

%================== 4. MAP HONEYCOMB ONTO CURVED SURFACE ==================
% For each UV point, compute the 3D position on the cylinder and the
% local surface normal. The nozzle orientation aligns with -normal.

% Center the honeycomb pattern on the detected void (not whole scaffold)
grid_u_extent = max(G_uv(:,:,1),[],'all') - min(G_uv(:,:,1),[],'all');
grid_v_extent = max(G_uv(:,:,2),[],'all') - min(G_uv(:,:,2),[],'all');
u_offset = mean(void_u_range) - grid_u_extent / 2;
v_offset = mean(void_v_range) - grid_v_extent / 2;

home_pos_uv = [0; 0; rise];

%--- Outline trajectory (conformal) ---
outline_trajectory = [];
new_pos = home_pos_uv;

for i = 1:size(outline_idx, 1)
    gx = outline_idx(i,1);
    gy = outline_idx(i,2);
    center_uv = squeeze(G_uv(gy, gx, :))';
    pts_uv = hexagonPerimeter(center_uv, hex_side);

    % Travel to first vertex at rise altitude (in UV + height)
    target = [pts_uv(1,:), rise]';
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    % Lower to surface (height = 0 means ON the surface)
    target(3) = 0;
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    % Trace perimeter layer by layer (inward: h goes from 0 to -wall_height)
    for layer = 1:num_layers
        h_layer = -((layer - 1) * layer_height);
        hex_pts = [pts_uv, repmat(h_layer, size(pts_uv,1), 1)]';
        outline_trajectory = [outline_trajectory hex_pts];
        new_pos = hex_pts(:,end);
    end

    % Rise to travel altitude
    target = [pts_uv(1,:), rise]';
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;
end

%--- Fill trajectory (conformal) ---
fill_trajectory = [];
for i = 1:size(fill_idx, 1)
    gx = fill_idx(i,1);
    gy = fill_idx(i,2);
    center_uv = squeeze(G_uv(gy, gx, :))';
    pts_uv = hexagonPerimeter(center_uv, hex_side);

    target = [pts_uv(1,:), rise]';
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    target(3) = 0;
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    for layer = 1:num_layers
        h_layer = -((layer - 1) * layer_height);
        hex_pts = [pts_uv, repmat(h_layer, size(pts_uv,1), 1)]';
        fill_trajectory = [fill_trajectory hex_pts];
        new_pos = hex_pts(:,end);
    end

    target = [pts_uv(1,:), rise]';
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;
end

%--- Deposit trajectory (vertical fill at cell centers) ---
all_idx = [outline_idx; fill_idx];
deposit_trajectory = [];
for i = 1:size(all_idx, 1)
    cx = all_idx(i,1);
    cy = all_idx(i,2);
    cell_center = squeeze(G_uv(cy, cx, :))';

    target = [cell_center, rise]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    target = [cell_center, 0]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    target = [cell_center, -wall_height]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;

    target = [cell_center, rise]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;
end

%================== 5. UV -> XYZ CONFORMAL MAPPING ==================
% Convert the full trajectory from UV parameter space to 3D Euclidean space
% on the semi-cylindrical surface.
% Convention after rotation: cylinder axis = X, curvature in Y-Z plane
%   U = arc-length (theta * R) around circumference in Y-Z
%   V = axial coordinate along X
%   theta = U / R  ->  Y = cy + R*sin(theta), Z = cz + R*cos(theta), X = V

traj_uv = [outline_trajectory, fill_trajectory, deposit_trajectory];
Npts = size(traj_uv, 2);

full_trajectory_xyz = zeros(3, Npts);
normal_vectors = zeros(3, Npts);

for k = 1:Npts
    u = traj_uv(1,k) + u_offset;  % arc-length coordinate
    v = traj_uv(2,k) + v_offset;  % axial coordinate (X)
    h = traj_uv(3,k);             % height above surface (deposition layers)

    theta_k = u / R_cyl;

    Sx = v;
    Sy = cyl_cy + R_cyl * sin(theta_k);
    Sz = cyl_cz + R_cyl * cos(theta_k);

    % Outward radial normal in YZ plane (points away from cylinder axis)
    nx = 0;
    ny = sin(theta_k);
    nz = cos(theta_k);
    n_hat = [nx; ny; nz];

    % TCP = surface + h along outward normal (h=rise for travel, h=0 on surface)
    full_trajectory_xyz(:,k) = [Sx; Sy; Sz] + h * n_hat;
    normal_vectors(:,k) = n_hat;
end

% Convert mm -> m for robot workspace
full_trajectory_m = full_trajectory_xyz * 0.001;
normal_vectors_unit = normal_vectors;  % already unit vectors

% Compensate for the 180°-Y base rotation
full_trajectory_m(1,:) = -full_trajectory_m(1,:);
full_trajectory_m(3,:) = -full_trajectory_m(3,:);
normal_vectors_unit(1,:) = -normal_vectors_unit(1,:);
normal_vectors_unit(3,:) = -normal_vectors_unit(3,:);

% Workspace offset (position the scaffold within robot reach)
z_offset = -0.35;
full_trajectory_m(3,:) = full_trajectory_m(3,:) + z_offset;

%================== 6. COMPUTE NOZZLE ORIENTATION (NORMAL-ALIGNED) ==================
% Build rotation matrix for each TCP pose: Z_tool = -n (nozzle into surface)
R_targets = zeros(3,3,Npts);
for k = 1:Npts
    n = normal_vectors_unit(:,k);
    z_tool = -n;  % nozzle points INTO the surface

    % Choose x_tool perpendicular to z_tool (prefer world X = axis direction)
    x_ref = [1; 0; 0];
    if abs(dot(z_tool, x_ref)) > 0.99
        x_ref = [0; 1; 0];
    end
    x_tool = cross(x_ref, z_tool);
    x_tool = x_tool / norm(x_tool);
    y_tool = cross(z_tool, x_tool);

    R_targets(:,:,k) = [x_tool, y_tool, z_tool];
end

% (Figure 7 removed — scaffold is now rendered via the robot tree in Figure 8.)

%================== 7b. STATIC SCAFFOLD + HONEYCOMB (mm, no robot) ==================
% Show only the DEPOSITED MATERIAL (h <= 0) — the end result, not travel paths.

% Separate deposition vs travel based on height above surface:
% traj_uv(3,:) <= 0 means on/inside the shell; > 0 means travel move.
deposition_mask = traj_uv(3,:) <= 0;
dep_xyz = full_trajectory_xyz(:, deposition_mask);

h_all = traj_uv(3,:);
outer_layer_mask = abs(h_all) < 1e-9;
outer_xyz = full_trajectory_xyz(:, outer_layer_mask);

figure('Name','Scaffold with Conformal Honeycomb','Position',[80 80 1000 700]);

trisurf(scaffold_conn, scaffold_pts(:,1), scaffold_pts(:,2), scaffold_pts(:,3), ...
    'FaceAlpha', 0.20, 'EdgeColor', [0.7 0.7 0.7], 'FaceColor', [0.85 0.75 0.55]);
hold on;

% Void boundary (red edges)
for k = 1:size(void_sharp_edges,1)
    p1 = scaffold_pts(void_sharp_edges(k,1),:);
    p2 = scaffold_pts(void_sharp_edges(k,2),:);
    hv = 'off'; if k == 1, hv = 'on'; end
    plot3([p1(1) p2(1)], [p1(2) p2(2)], [p1(3) p2(3)], 'r-', 'LineWidth', 2, 'HandleVisibility', hv);
end

% Deposited honeycomb structure only (no travel moves)
plot3(dep_xyz(1,:), dep_xyz(2,:), dep_xyz(3,:), 'b.', 'MarkerSize', 2);
% Explicitly overlay the outer surface layer to make curvature visible.
plot3(outer_xyz(1,:), outer_xyz(2,:), outer_xyz(3,:), 'c.', 'MarkerSize', 4);

grid on; axis equal;
xlabel('X (mm)'); ylabel('Y (mm)'); zlabel('Z (mm)');
title('Conformal Honeycomb Infill — End Result');
legend('Scaffold','Void boundary','Honeycomb infill','Outer layer curve','Location','best');
view(135, 25);

% Side-section view to verify curvature in YZ plane (cross-section of shell).
figure('Name','Conformal Infill Side Section (YZ)','Position',[120 120 900 650]);
hold on; grid on;
section_tol = 2.0;  % mm around X=0
shell_side = abs(scaffold_pts(:,1)) < section_tol;
dep_side = abs(dep_xyz(1,:)) < section_tol;
outer_side = abs(outer_xyz(1,:)) < section_tol;
plot(scaffold_pts(shell_side,2), scaffold_pts(shell_side,3), '.', 'Color', [0.6 0.6 0.6], 'MarkerSize', 5);
plot(dep_xyz(2,dep_side), dep_xyz(3,dep_side), 'b.', 'MarkerSize', 5);
plot(outer_xyz(2,outer_side), outer_xyz(3,outer_side), 'c.', 'MarkerSize', 8);
% Reference arc
th_ref = linspace(-pi/2, pi/2, 200);
plot(cyl_cy + cyl_R*sin(th_ref), cyl_cz + cyl_R*cos(th_ref), 'g--', 'LineWidth', 1.5);
xlabel('Y (mm)'); ylabel('Z (mm)');
title('YZ Section at X \approx 0 (curvature check)');
legend('Shell section','Infill section','Outer infill layer', ...
    sprintf('Circle R=%.1f', cyl_R),'Location','best');

%%
%================== 8. INVERSE KINEMATICS (Normal-Aligned) ==================
q_sol = zeros(8, Npts);

P_actual_log  = zeros(3, Npts);
P_desired_log = zeros(3, Npts);
detJ_log      = zeros(1, Npts);
err_log       = zeros(1, Npts);
ctrl_log      = zeros(1, Npts);
singval_log   = zeros(6, Npts);  % all 6 singular values of J*J'

fprintf('Computing IK for %d points (8-DOF, normal-aligned orientation)...\n', Npts);

figure('Name','IK Validation');
show(rbt, [baseOrientation, basePosition, homeConfiguration(mc)]);
title('Trajectory Validation: Desired (Blue) vs. Actual (Red)');
hold on; grid on; axis equal;
view(135,25);

ik = inverseKinematics("RigidBodyTree", mc);
weights = [1 1 1 1 1 1];  % equal weight on position and orientation
initialguess = homeConfiguration(mc);
Rbase = quat2rotm(baseOrientation);

for i = 1:Npts
    P_target = full_trajectory_m(:, i);
    R_target = R_targets(:,:,i);

    T_target = rotm2tform(R_target);
    T_target(1:3,4) = P_target;

    [q_tmp, solInfo] = ik("link6", T_target, weights, initialguess);
    q_sol(:,i) = q_tmp(:);

    % Jacobian analysis (6x8)
    J = geometricJacobian(mc, q_tmp, 'link6');
    JJt = J * J';
    detJ_log(i) = sqrt(det(JJt));
    singval_log(:,i) = svd(JJt);

    % Tracking error (position)
    T_test = getTransform(mc, q_tmp, 'link6');
    P_test = T_test(1:3,4);
    P_desired_log(:,i) = P_target;
    P_actual_log(:,i)  = P_test;
    err_log(i) = norm(P_target - P_test);

    % Control effort
    if i == 1
        ctrl_log(i) = norm(q_tmp(:) - initialguess(:));
    else
        ctrl_log(i) = norm(q_sol(:,i) - q_sol(:,i-1));
    end

    initialguess = q_tmp;

    % Live visualization (every 10th point for speed)
    if mod(i, 10) == 1 || i == Npts
        floatingRBTConfig = [baseOrientation, basePosition, q_sol(:,i)'];
        show(rbt, floatingRBTConfig, 'PreservePlot', false, 'FastUpdate', true);

        P_plot = Rbase * P_target;
        plot3(P_plot(1), P_plot(2), P_plot(3), 'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b');

        tform_actual = getTransform(rbt, floatingRBTConfig, 'link6');
        P_actual_world = tform_actual(1:3, 4);
        plot3(P_actual_world(1), P_actual_world(2), P_actual_world(3), 'r*', 'MarkerSize', 6);
        drawnow;
    end

    if mod(i, 50) == 0
        fprintf('  Point %d/%d — error: %.3f mm, manipulability: %.4f\n', ...
            i, Npts, err_log(i)*1000, detJ_log(i));
    end
end
hold off;
fprintf('IK complete. Max error: %.3f mm, Mean error: %.3f mm\n', ...
    max(err_log)*1000, mean(err_log)*1000);

%================== 9. DIAGNOSTIC PLOTS ==================
dt = 0.05;
t  = (0:Npts-1) * dt;

figure('Name','Trajectory Tracking');
subplot(3,1,1);
plot(t, P_desired_log(1,:), 'b-', 'LineWidth', 1.2); hold on;
plot(t, P_actual_log(1,:),  'r--', 'LineWidth', 1.2);
ylabel('X [m]'); legend('Desired','Actual'); grid on;
title('TCP Position Tracking (Conformal Path)');

subplot(3,1,2);
plot(t, P_desired_log(2,:), 'b-', 'LineWidth', 1.2); hold on;
plot(t, P_actual_log(2,:),  'r--', 'LineWidth', 1.2);
ylabel('Y [m]'); legend('Desired','Actual'); grid on;

subplot(3,1,3);
plot(t, P_desired_log(3,:), 'b-', 'LineWidth', 1.2); hold on;
plot(t, P_actual_log(3,:),  'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Z [m]'); legend('Desired','Actual'); grid on;

% Jacobian diagnostics
figure('Name','Jacobian Analysis');
subplot(3,1,1);
plot(t, detJ_log, 'k-', 'LineWidth', 1.2);
ylabel('$\sqrt{\det(J\,J^T)}$', 'Interpreter', 'latex');
title('Manipulability Along Conformal Path');
grid on;

subplot(3,1,2);
semilogy(t, singval_log(1,:), 'b-', t, singval_log(6,:), 'r-', 'LineWidth', 1.2);
ylabel('Singular Values');
legend('\sigma_{max}','\sigma_{min}'); grid on;
title('Condition of J*J^T (proximity to singularity)');

subplot(3,1,3);
plot(t, singval_log(1,:) ./ max(singval_log(6,:), 1e-10), 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Condition Number');
title('Jacobian Condition Number'); grid on;

% Tracking error and control effort
figure('Name','Error & Effort');
subplot(2,1,1);
plot(t, err_log * 1e3, 'm-', 'LineWidth', 1.2);
ylabel('Tracking Error [mm]');
title('Position Error Along Conformal Path'); grid on;

subplot(2,1,2);
plot(t, ctrl_log, 'Color', [0 0.5 0], 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('$\|\Delta q\|$', 'Interpreter', 'latex');
title('Control Effort (Joint Increment)'); grid on;

%================== 10. TRAJECTORY VALIDATION (No Robot) ==================
% 3D plot of desired (blue) vs actual (red) trajectory — no robot model.
figure('Name','Trajectory Validation','Position',[100 100 1100 800]);

% Desired trajectory (blue)
plot3(P_desired_log(1,:), P_desired_log(2,:), P_desired_log(3,:), ...
    'b-', 'LineWidth', 1.5, 'DisplayName', 'Desired trajectory');
hold on;

% Actual trajectory from FK (red)
plot3(P_actual_log(1,:), P_actual_log(2,:), P_actual_log(3,:), ...
    'r--', 'LineWidth', 1.2, 'DisplayName', 'Actual trajectory (IK)');

grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Trajectory Validation: Desired vs Actual');
legend('Location', 'best');
view(135, 25);

% Add a second view (top-down) as subplot for clarity
figure('Name','Trajectory Validation (Views)','Position',[150 80 1400 550]);

subplot(1,3,1);
plot3(P_desired_log(1,:), P_desired_log(2,:), P_desired_log(3,:), ...
    'b-', 'LineWidth', 1.5);
hold on;
plot3(P_actual_log(1,:), P_actual_log(2,:), P_actual_log(3,:), ...
    'r--', 'LineWidth', 1.0);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Isometric View');
legend('Desired','Actual','Location','best');
view(135, 25);

subplot(1,3,2);
plot(P_desired_log(1,:), P_desired_log(2,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(P_actual_log(1,:), P_actual_log(2,:), 'r--', 'LineWidth', 1.0);
grid on; axis equal;
xlabel('X [m]'); ylabel('Y [m]');
title('Top View (XY)');
legend('Desired','Actual','Location','best');

subplot(1,3,3);
plot(P_desired_log(2,:), P_desired_log(3,:), 'b-', 'LineWidth', 1.5);
hold on;
plot(P_actual_log(2,:), P_actual_log(3,:), 'r--', 'LineWidth', 1.0);
grid on; axis equal;
xlabel('Y [m]'); ylabel('Z [m]');
title('Side View (YZ)');
legend('Desired','Actual','Location','best');

sgtitle('Trajectory Validation: Desired (Blue) vs Actual (Red)');

%================== HELPER FUNCTIONS ==================

% Create grid
function G = createGrid(Nx, Ny, hex_side)
    xSpacing=1.5*hex_side;
    ySpacing=hex_side*sqrt(3);
    [X, Y] = meshgrid(0:xSpacing:(Nx-1)*xSpacing, 0:ySpacing:(Ny-1)*ySpacing);

    % Shift odd columns by ySpacing/2
    Y(:, 1:2:end) = Y(:, 1:2:end) + ySpacing/2;

    G = zeros(Ny, Nx, 2);
    G(:,:,1) = X;
    G(:,:,2) = Y;
end

% Create Hexagon
function pts = hexagonPerimeter(center, hex_side, n)

    if nargin < 3
        n = 20; %points per edge
    end

    cx = center(1);
    cy = center(2);

    % Radius to vertices
    R = hex_side;

    % Flat-top hexagon vertex angles (degrees)
    angles = [0 60 120 180 240 300] * pi/180;

    % Hexagon vertices
    V = [cx + R*cos(angles)', cy + R*sin(angles)'];

    % Close the polygon
    V = [V; V(1,:)];

    % Generate perimeter points
    pts = [];
    for i = 1:6
        x = linspace(V(i,1), V(i+1,1), n);
        y = linspace(V(i,2), V(i+1,2), n);
        pts = [pts; x(1:end-1)', y(1:end-1)'];
    end
end

%Line between points
function target_trajectory=linePoints(start_pos,end_pos,num_points)
% Create a list of 100 XYZ points between start and end
target_trajectory = [linspace(start_pos(1), end_pos(1), num_points); ...
                     linspace(start_pos(2), end_pos(2), num_points); ...
                     linspace(start_pos(3), end_pos(3), num_points)];
end

% TSP solver using Miller-Tucker-Zemlin formulation (open path, intlinprog)
function tour = solveTSP_MTZ(D, n)
% Solves an open-path TSP for n cities using MILP (intlinprog).
% Returns the optimal visitation order as a permutation vector [1..n].
%
% For open path: adds a dummy node (node n+1) with zero-cost arcs
% to convert the open-path problem into a closed tour.

    N = n + 1;  % add dummy node for open path
    
    % Expand distance matrix with dummy node (zero cost to/from all)
    D_ext = zeros(N, N);
    D_ext(1:n, 1:n) = D;
    % Dummy node (N) has zero-cost connections to all real nodes
    
    % Decision variables: x(i,j) binary for N*N arcs + u(i) continuous for N nodes
    % Variable layout: x flattened as (i-1)*N + j, then u as N*N + i
    n_x = N * N;
    n_u = N;
    n_vars = n_x + n_u;
    
    % Index helpers
    xidx = @(i,j) (i-1)*N + j;  % 1-based i,j
    uidx = @(i) n_x + i;
    
    % Objective: minimize sum d(i,j)*x(i,j)
    f = zeros(n_vars, 1);
    for i = 1:N
        for j = 1:N
            if i ~= j
                f(xidx(i,j)) = D_ext(i,j);
            end
        end
    end
    
    % Variable types: x = binary, u = continuous
    intcon = 1:n_x;  % all x variables are integer (binary via bounds)
    
    % Bounds
    lb = zeros(n_vars, 1);
    ub = [ones(n_x, 1); (N-1)*ones(n_u, 1)];
    % Fix x(i,i) = 0
    for i = 1:N
        ub(xidx(i,i)) = 0;
    end
    % Fix u for dummy node (depot)
    lb(uidx(N)) = 0;
    ub(uidx(N)) = 0;
    
    % Equality constraints: each node has exactly 1 outgoing and 1 incoming arc
    Aeq = sparse(2*N, n_vars);
    beq = ones(2*N, 1);
    for i = 1:N
        % sum_j x(i,j) = 1 (outgoing)
        for j = 1:N
            if i ~= j
                Aeq(i, xidx(i,j)) = 1;
            end
        end
        % sum_j x(j,i) = 1 (incoming)
        for j = 1:N
            if j ~= i
                Aeq(N+i, xidx(j,i)) = 1;
            end
        end
    end
    
    % Inequality constraints: MTZ subtour elimination
    % u(i) - u(j) + N*x(i,j) <= N-1, for all i,j in real nodes (not dummy)
    mtz_pairs = [];
    for i = 1:n
        for j = 1:n
            if i ~= j
                mtz_pairs = [mtz_pairs; i j];
            end
        end
    end
    n_mtz = size(mtz_pairs, 1);
    A_ineq = sparse(n_mtz, n_vars);
    b_ineq = (N-1) * ones(n_mtz, 1);
    for k = 1:n_mtz
        i = mtz_pairs(k, 1);
        j = mtz_pairs(k, 2);
        A_ineq(k, uidx(i)) = 1;
        A_ineq(k, uidx(j)) = -1;
        A_ineq(k, xidx(i,j)) = N;
    end
    
    % Solve
    opts = optimoptions('intlinprog', 'Display', 'off', 'MaxTime', 60);
    [x_sol, ~, exitflag] = intlinprog(f, intcon, A_ineq, b_ineq, Aeq, beq, lb, ub, opts);
    
    if exitflag <= 0
        warning('TSP solver did not find optimal solution, using sequential order.');
        tour = 1:n;
        return;
    end
    
    % Extract tour from solution
    X = reshape(x_sol(1:n_x), [N, N]);
    X = round(X);  % clean numerical noise
    
    % Follow the tour starting from the dummy node
    tour_full = zeros(1, N);
    tour_full(1) = N;  % start at dummy
    for step = 2:N
        curr = tour_full(step-1);
        nxt = find(X(curr,:) > 0.5, 1);
        tour_full(step) = nxt;
    end
    
    % Remove dummy node, keep only real nodes in visitation order
    tour = tour_full(tour_full <= n);
end