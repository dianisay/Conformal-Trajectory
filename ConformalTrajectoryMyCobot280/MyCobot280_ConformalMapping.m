function result = MyCobot280_ConformalMapping(config)
% MYCOBOT280_CONFORMALMAPPING
% Clean production pipeline for conformal honeycomb trajectory generation
% and optional execution on MyCobot 280 through mc_bridge.py.
%
% Keeps only what is needed for final application:
%   1) Load scaffold STL and estimate cylinder geometry
%   2) Detect the void region on the scaffold
%   3) Generate conformal UV honeycomb toolpath
%   4) Map UV->XYZ and convert to robot waypoint rows [X Y Z]
%   5) Save waypoints and (optionally) send to Python bridge

if nargin < 1
	config = struct();
end
config = apply_defaults(config);

fprintf('=== MyCobot280 Conformal Mapping (clean) ===\n');
fprintf('Scaffold STL: %s\n', config.scaffoldStl);

% -------------------------------------------------------------------------
% 1) Load scaffold + estimate cylindrical surface
% -------------------------------------------------------------------------
TR = stlread(config.scaffoldStl);
pts = TR.Points;
conn = TR.ConnectivityList;

% Match the orientation used in the validated script.
Rx90 = [1 0 0; 0 0 -1; 0 1 0];
pts = (Rx90 * pts')';

[cyl_cy, cyl_cz, R_cyl] = fit_cylinder_yz(pts);
fprintf('Cylinder fit: centerYZ=[%.3f, %.3f], R=%.3f mm\n', cyl_cy, cyl_cz, R_cyl);

% -------------------------------------------------------------------------
% 2) Detect void boundary from sharp edges
% -------------------------------------------------------------------------
[void_vid, theta_min, theta_max, x_void_min, x_void_max] = detect_void_component(pts, conn, cyl_cy, cyl_cz, R_cyl, config);

void_pts = pts(void_vid, :);
theta_void = atan2(void_pts(:,2) - cyl_cy, void_pts(:,3) - cyl_cz);
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

void_u_range = [theta_min * R_cyl, theta_max * R_cyl];
void_v_range = [x_void_min, x_void_max];
void_width = diff(void_u_range);
void_length = diff(void_v_range);

fprintf('Void size (arc x axial): %.3f x %.3f mm\n', void_width, void_length);
fprintf('Estimated shell thickness: %.3f mm\n', shell_thickness);

% -------------------------------------------------------------------------
% 3) Conformal UV honeycomb generation
% -------------------------------------------------------------------------
hex_side = min(void_width, void_length) / config.hexDivisor;
Nx = max(2, floor(void_width / (hex_side * 1.5)));
Ny = max(2, floor(void_length / (hex_side * sqrt(3))));

rise = config.travelRiseMm;
layer_h = config.layerHeightMm;
num_layers = max(1, ceil(max(shell_thickness, layer_h) / layer_h));

fprintf('Grid: %dx%d, hex_side=%.3f mm, layers=%d\n', Nx, Ny, hex_side, num_layers);

G_uv = createGrid(Nx, Ny, hex_side);
[outline_idx, fill_idx] = full_cell_index(Nx, Ny);

grid_u_extent = max(G_uv(:,:,1), [], 'all') - min(G_uv(:,:,1), [], 'all');
grid_v_extent = max(G_uv(:,:,2), [], 'all') - min(G_uv(:,:,2), [], 'all');
u_offset = mean(void_u_range) - grid_u_extent / 2;
v_offset = mean(void_v_range) - grid_v_extent / 2;

num_points = config.linePoints;
new_pos = [0; 0; rise];

outline_traj = [];
for i = 1:size(outline_idx, 1)
	gx = outline_idx(i,1);
	gy = outline_idx(i,2);
	center_uv = squeeze(G_uv(gy, gx, :))';
	pts_uv = hexagonPerimeter(center_uv, hex_side, config.hexEdgeSamples);

	target = [pts_uv(1,:), rise]';
	outline_traj = [outline_traj, linePoints(new_pos, target, num_points)]; %#ok<AGROW>
	new_pos = target;

	target(3) = 0;
	outline_traj = [outline_traj, linePoints(new_pos, target, num_points)]; %#ok<AGROW>
	new_pos = target;

	for layer = 1:num_layers
		h_layer = -((layer - 1) * layer_h);
		hex_pts = [pts_uv, repmat(h_layer, size(pts_uv,1), 1)]';
		outline_traj = [outline_traj, hex_pts]; %#ok<AGROW>
		new_pos = hex_pts(:, end);
	end

	target = [pts_uv(1,:), rise]';
	outline_traj = [outline_traj, linePoints(new_pos, target, num_points)]; %#ok<AGROW>
	new_pos = target;
end

fill_traj = [];
for i = 1:size(fill_idx, 1)
	gx = fill_idx(i,1);
	gy = fill_idx(i,2);
	center_uv = squeeze(G_uv(gy, gx, :))';
	pts_uv = hexagonPerimeter(center_uv, hex_side, config.hexEdgeSamples);

	target = [pts_uv(1,:), rise]';
	fill_traj = [fill_traj, linePoints(new_pos, target, num_points)]; %#ok<AGROW>
	new_pos = target;

	target(3) = 0;
	fill_traj = [fill_traj, linePoints(new_pos, target, num_points)]; %#ok<AGROW>
	new_pos = target;

	for layer = 1:num_layers
		h_layer = -((layer - 1) * layer_h);
		hex_pts = [pts_uv, repmat(h_layer, size(pts_uv,1), 1)]';
		fill_traj = [fill_traj, hex_pts]; %#ok<AGROW>
		new_pos = hex_pts(:, end);
	end

	target = [pts_uv(1,:), rise]';
	fill_traj = [fill_traj, linePoints(new_pos, target, num_points)]; %#ok<AGROW>
	new_pos = target;
end

traj_uv = [outline_traj, fill_traj];

% -------------------------------------------------------------------------
% 4) UV->XYZ map, robot-frame transform, waypoint extraction
% -------------------------------------------------------------------------
Npts = size(traj_uv, 2);
xyz_mm = zeros(3, Npts);

for k = 1:Npts
	u = traj_uv(1,k) + u_offset;
	v = traj_uv(2,k) + v_offset;
	h = traj_uv(3,k);

	theta_k = u / R_cyl;
	Sx = v;
	Sy = cyl_cy + R_cyl * sin(theta_k);
	Sz = cyl_cz + R_cyl * cos(theta_k);

	n_hat = [0; sin(theta_k); cos(theta_k)];
	xyz_mm(:,k) = [Sx; Sy; Sz] + h * n_hat;
end

% Use the same axis convention used by the original validated mapping.
xyz_mm(1,:) = -xyz_mm(1,:);
xyz_mm(3,:) = -xyz_mm(3,:);

% Final translation in mm to place the trajectory into the real robot workspace.
xyz_mm(1,:) = xyz_mm(1,:) + config.robotOffsetMm(1);
xyz_mm(2,:) = xyz_mm(2,:) + config.robotOffsetMm(2);
xyz_mm(3,:) = xyz_mm(3,:) + config.robotOffsetMm(3);

waypoints_xyz = xyz_mm';
waypoints_xyz = waypoints_xyz(~any(~isfinite(waypoints_xyz), 2), :);

if config.decimateStep > 1
	waypoints_xyz = waypoints_xyz(1:config.decimateStep:end, :);
end

% Optional bounds clamp for MyCobot safety envelope in mm.
if config.enableClamp
	waypoints_xyz(:,1) = min(max(waypoints_xyz(:,1), config.xMin), config.xMax);
	waypoints_xyz(:,2) = min(max(waypoints_xyz(:,2), config.yMin), config.yMax);
	waypoints_xyz(:,3) = min(max(waypoints_xyz(:,3), config.zMin), config.zMax);
end

% -------------------------------------------------------------------------
% 5) Save outputs + optional Python execution
% -------------------------------------------------------------------------
if ~exist(config.outputDir, 'dir')
	mkdir(config.outputDir);
end

csvPath = fullfile(config.outputDir, config.outputCsv);
matPath = fullfile(config.outputDir, config.outputMat);

writematrix(waypoints_xyz, csvPath);
save(matPath, 'waypoints_xyz', 'traj_uv', 'void_u_range', 'void_v_range', 'R_cyl', 'cyl_cy', 'cyl_cz');

fprintf('Saved CSV: %s\n', csvPath);
fprintf('Saved MAT: %s\n', matPath);
fprintf('Waypoints: %d\n', size(waypoints_xyz,1));

sendResult = struct('ok', false, 'message', 'sendToRobot disabled');
if config.sendToRobot
	sendResult = send_waypoints_python(waypoints_xyz, config);
end

result = struct();
result.ok = true;
result.csvPath = csvPath;
result.matPath = matPath;
result.waypointCount = size(waypoints_xyz,1);
result.sendResult = sendResult;
end

% ========================= Local Functions ================================

function config = apply_defaults(config)
baseDir = fileparts(mfilename('fullpath'));

defaults = struct(...
	'scaffoldStl', fullfile(baseDir, 'scaffold_curved_void.stl'), ...
	'outputDir', baseDir, ...
	'outputCsv', 'mycobot280_waypoints_xyz.csv', ...
	'outputMat', 'mycobot280_waypoints_xyz.mat', ...
	'hexDivisor', 4, ...
	'hexEdgeSamples', 8, ...
	'linePoints', 5, ...
	'travelRiseMm', 20, ...
	'layerHeightMm', 0.4, ...
	'robotOffsetMm', [0, 0, 0], ...
	'decimateStep', 4, ...
	'enableClamp', false, ...
	'xMin', -280, 'xMax', 280, ...
	'yMin', -280, 'yMax', 280, ...
	'zMin', 0,    'zMax', 350, ...
	'sharpAngleDeg', 35, ...
	'edgeMarginMm', 3, ...
	'sendToRobot', true, ...
	'pythonExecutable', '', ...
	'pythonModuleDir', baseDir, ...
	'pythonBridgeModule', 'mc_bridge', ...
	'robotSpeed', 80, ...
	'robotWait', true, ...
	'robotTimeoutS', 20.0, ...
	'robotSettleS', 0.35, ...
	'coordinateMode', 'local', ...
	'startAngles', [0, -40, -130, 80, 0, 50], ...
	'startSpeed', 50, ...
	'startSettleS', 3.0);

fields = fieldnames(defaults);
for i = 1:numel(fields)
	f = fields{i};
	if ~isfield(config, f) || isempty(config.(f))
		config.(f) = defaults.(f);
	end
end
end

function [cy, cz, R] = fit_cylinder_yz(pts)
A = [pts(:,2), pts(:,3), ones(size(pts,1),1)];
b = pts(:,2).^2 + pts(:,3).^2;
x = A \ b;
cy = x(1) / 2;
cz = x(2) / 2;
R = sqrt(x(3) + cy^2 + cz^2);
end

function [void_vid, theta_min, theta_max, x_void_min, x_void_max] = detect_void_component(pts, conn, cyl_cy, cyl_cz, R_cyl, config)
nf = size(conn, 1);
face_normals = zeros(nf, 3);
for f = 1:nf
	v1 = pts(conn(f,1),:);
	v2 = pts(conn(f,2),:);
	v3 = pts(conn(f,3),:);
	n = cross(v2 - v1, v3 - v1);
	face_normals(f,:) = n / (norm(n) + eps);
end

edges_all = [conn(:,[1 2]); conn(:,[2 3]); conn(:,[3 1])];
faces_all = [(1:nf)'; (1:nf)'; (1:nf)'];
edges_sorted = sort(edges_all, 2);
[uniq_edges, ~, ic] = unique(edges_sorted, 'rows');

sharp_mask = false(size(uniq_edges,1),1);
for e = 1:size(uniq_edges,1)
	face_ids = faces_all(ic == e);
	if numel(face_ids) == 2
		n1 = face_normals(face_ids(1),:);
		n2 = face_normals(face_ids(2),:);
		ang = acosd(max(-1, min(1, dot(n1, n2))));
		sharp_mask(e) = ang > config.sharpAngleDeg;
	end
end
sharp_edges = uniq_edges(sharp_mask, :);

all_v = unique(sharp_edges(:));
vmap = containers.Map(num2cell(all_v), num2cell(1:numel(all_v)));
adj = cell(numel(all_v),1);
for k = 1:size(sharp_edges,1)
	a = vmap(sharp_edges(k,1));
	b = vmap(sharp_edges(k,2));
	adj{a}(end+1) = b; %#ok<AGROW>
	adj{b}(end+1) = a; %#ok<AGROW>
end

visited = false(numel(all_v),1);
components = {};
for s = 1:numel(all_v)
	if visited(s)
		continue;
	end
	q = s;
	visited(s) = true;
	comp = s;
	while ~isempty(q)
		cur = q(1);
		q(1) = [];
		for nb = adj{cur}
			if ~visited(nb)
				visited(nb) = true;
				q(end+1) = nb; %#ok<AGROW>
				comp(end+1) = nb; %#ok<AGROW>
			end
		end
	end
	components{end+1} = all_v(comp); %#ok<AGROW>
end

xmin_all = min(pts(:,1));
xmax_all = max(pts(:,1));

best_idx = -1;
best_score = -inf;
for c = 1:numel(components)
	vid = components{c};
	p = pts(vid,:);
	axr = [min(p(:,1)), max(p(:,1))];
	nverts = size(p,1);
	touches_ax_end = (axr(1) <= xmin_all + config.edgeMarginMm) || (axr(2) >= xmax_all - config.edgeMarginMm);
	mean_rad_err = mean(abs(sqrt((p(:,2)-cyl_cy).^2 + (p(:,3)-cyl_cz).^2) - R_cyl));
	theta_r = atan2(p(:,2)-cyl_cy, p(:,3)-cyl_cz);
	span_area = (max(theta_r)-min(theta_r))*R_cyl * (axr(2)-axr(1));

	score = 0;
	if ~touches_ax_end
		score = score + 100;
	end
	score = score + min(nverts, 80);
	score = score + max(0, 20 - mean_rad_err*4);
	score = score + min(span_area/40, 40);

	if score > best_score
		best_score = score;
		best_idx = c;
	end
end

if best_idx < 0
	error('Could not select a candidate void component.');
end

void_vid = components{best_idx};
void_pts = pts(void_vid,:);
theta_void = atan2(void_pts(:,2) - cyl_cy, void_pts(:,3) - cyl_cz);
theta_min = min(theta_void);
theta_max = max(theta_void);
x_void_min = min(void_pts(:,1));
x_void_max = max(void_pts(:,1));
end

function [outline_idx, fill_idx] = full_cell_index(Nx, Ny)
outline_idx = zeros(Nx * Ny, 2);
k = 1;
for iy = 1:Ny
	for ix = 1:Nx
		outline_idx(k,:) = [ix, iy];
		k = k + 1;
	end
end
fill_idx = outline_idx;
end

function G = createGrid(Nx, Ny, hex_side)
xSpacing = 1.5 * hex_side;
ySpacing = hex_side * sqrt(3);
[X, Y] = meshgrid(0:xSpacing:(Nx-1)*xSpacing, 0:ySpacing:(Ny-1)*ySpacing);
Y(:, 1:2:end) = Y(:, 1:2:end) + ySpacing / 2;
G = zeros(Ny, Nx, 2);
G(:,:,1) = X;
G(:,:,2) = Y;
end

function pts = hexagonPerimeter(center, hex_side, n)
if nargin < 3
	n = 20;
end
cx = center(1);
cy = center(2);
angles = [0 60 120 180 240 300] * pi/180;
V = [cx + hex_side*cos(angles)', cy + hex_side*sin(angles)'];
V = [V; V(1,:)];

pts = [];
for i = 1:6
	x = linspace(V(i,1), V(i+1,1), n);
	y = linspace(V(i,2), V(i+1,2), n);
	pts = [pts; x(1:end-1)', y(1:end-1)']; %#ok<AGROW>
end
end

function segment = linePoints(start_pos, end_pos, num_points)
segment = [linspace(start_pos(1), end_pos(1), num_points); ...
		   linspace(start_pos(2), end_pos(2), num_points); ...
		   linspace(start_pos(3), end_pos(3), num_points)];
end

function sendResult = send_waypoints_python(waypoints_xyz, config)
sendResult = struct('ok', false, 'message', 'not executed');

pe = pyenv;
if pe.Status == "NotLoaded"
	if ~isempty(config.pythonExecutable)
		pyenv('Version', config.pythonExecutable, 'ExecutionMode', 'OutOfProcess');
	else
		pyenv('ExecutionMode', 'OutOfProcess');
	end
elseif pe.Status == "Loaded" && pe.ExecutionMode == "InProcess"
	error('Python must run OutOfProcess for reliable robot bridge calls.');
end

pyPaths = cell(py.sys.path);
moduleDir = char(config.pythonModuleDir);
hasModuleDir = any(cellfun(@(p) strcmp(char(p), moduleDir), pyPaths));
if ~hasModuleDir
	insert(py.sys.path, int32(0), moduleDir);
end

bridge = py.importlib.import_module(config.pythonBridgeModule);
py.importlib.reload(bridge);

csvPath = fullfile(config.outputDir, config.outputCsv);
fprintf('Sending waypoints to Python via CSV (fast path): %s\n', csvPath);

res = bridge.execute_waypoints_draw_mode_csv( ...
	char(csvPath), ...
	pyargs('draw_speed', int32(config.robotSpeed), ...
		   'wait_per_point', double(config.robotSettleS), ...
		   'start_angles', py.list(num2cell(config.startAngles)), ...
		   'start_speed', int32(config.startSpeed), ...
		   'settle_s', double(config.startSettleS), ...
		   'coordinate_mode', char(config.coordinateMode), ...
		   'z_min', double(config.zMin), ...
		   'z_max', double(config.zMax)));

sendResult.ok = logical(res{'ok'});
sendResult.message = char(res{'message'});
sendResult.count = double(res{'count'});
fprintf('Python bridge result: ok=%d, count=%d, message=%s\n', sendResult.ok, sendResult.count, sendResult.message);
end
