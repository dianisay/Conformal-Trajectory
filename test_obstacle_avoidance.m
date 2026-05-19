close all; clear; clc;
warning('off','all')   % turn all warnings off
%warning('on','all')    % turn all back on

mm = 1e-3;

%% =================== 1. ROBOT MODEL (same as MuffinFresa_ConformalMapping) ===================
arm = importrobot("mycobot.urdf", DataFormat="row");

mc = rigidBodyTree(DataFormat="row");
mc.BaseName = 'world_root';

wsRefX = -0.5;  wsRefY = -0.5;  wsRefZ = 0.1;
wsRef = rigidBody("workspace_ref");
wsRefJnt = rigidBodyJoint("ws_ref_fixed", "fixed");
setFixedTransform(wsRefJnt, trvec2tform([wsRefX, wsRefY, wsRefZ]));
wsRef.Joint = wsRefJnt;
addBody(mc, wsRef, mc.BaseName);

boxBody = rigidBody("box_visual");
boxJnt = rigidBodyJoint("box_fixed", "fixed");
setFixedTransform(boxJnt, trvec2tform([0, 0, 0]));
boxBody.Joint = boxJnt;
Tvis_box = axang2tform([1 0 0 -pi/2]);
addVisual(boxBody, "Mesh", {"BOXCOMPLETOOO10STL.stl", [0.013 0.013 0.013]}, Tvis_box);
addBody(mc, boxBody, "workspace_ref");

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

rbt = rigidBodyTree(DataFormat="row");
floatingBaseBody = rigidBody("floatingBase");
floatingBaseBody.Joint = rigidBodyJoint("j1","floating");
addBody(rbt, floatingBaseBody, rbt.BaseName);
rbt.BaseName = 'world';
addSubtree(rbt, "floatingBase", mc, ReplaceBase=false);

baseOrientation = eul2quat([0 pi 0]);
basePosition = [0.0 0.0 0.0];

fprintf('Robot model assembled (%d bodies, %d DOF).\n', ...
    numel(mc.Bodies), numel(homeConfiguration(mc)));

%% =================== 2. TRAJECTORY GENERATION (compact, no figures) ===================
scaffold_stl = 'scaffold_curved_void.stl';
TR_scaffold = stlread(scaffold_stl);
scaffold_pts  = TR_scaffold.Points;
scaffold_conn = TR_scaffold.ConnectivityList;

Rx90 = [1 0 0; 0 0 -1; 0 1 0];
scaffold_pts = (Rx90 * scaffold_pts')';

A_fit = [scaffold_pts(:,2), scaffold_pts(:,3), ones(size(scaffold_pts,1),1)];
b_fit = scaffold_pts(:,2).^2 + scaffold_pts(:,3).^2;
x_fit = A_fit \ b_fit;
cyl_cy = x_fit(1)/2;
cyl_cz = x_fit(2)/2;
cyl_R  = sqrt(x_fit(3) + cyl_cy^2 + cyl_cz^2);

scaffold_radius = cyl_R;
scaffold_length = max(scaffold_pts(:,1)) - min(scaffold_pts(:,1));
R_cyl = scaffold_radius;

% --- Void detection (sharp edges + connected components) ---
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

xmin_all = min(scaffold_pts(:,1));
xmax_all = max(scaffold_pts(:,1));
edge_margin = 3;
best_idx = -1; best_score = -inf;
for c = 1:numel(components)
    vid = components{c};
    p = scaffold_pts(vid,:);
    axr = [min(p(:,1)), max(p(:,1))];
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
    if score > best_score, best_score = score; best_idx = c; end
end
if best_idx < 0, error('No void component found.'); end

void_vid = components{best_idx};
void_pts = scaffold_pts(void_vid,:);
theta_void = atan2(void_pts(:,2) - cyl_cy, void_pts(:,3) - cyl_cz);
theta_min = min(theta_void);  theta_max = max(theta_void);
x_void_min = min(void_pts(:,1));  x_void_max = max(void_pts(:,1));

void_u_range = [theta_min * R_cyl, theta_max * R_cyl];
void_v_range = [x_void_min, x_void_max];
void_width  = diff(void_u_range);
void_length = diff(void_v_range);

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

% --- Honeycomb grid ---
hex_side = min(void_width, void_length) / 6;
Nx = max(2, floor(void_width / (hex_side * 1.5)));
Ny = max(2, floor(void_length / (hex_side * sqrt(3))));
num_points = 20;
rise = 20;
wall_height = shell_thickness;
layer_height = 0.4;
num_layers = ceil(wall_height / layer_height);

outline_idx = [];
for iy = 1:Ny
    for ix = 1:Nx
        outline_idx = [outline_idx; ix iy]; %#ok<AGROW>
    end
end
fill_idx = outline_idx;

G_uv = createGrid(Nx, Ny, hex_side);

grid_u_extent = max(G_uv(:,:,1),[],'all') - min(G_uv(:,:,1),[],'all');
grid_v_extent = max(G_uv(:,:,2),[],'all') - min(G_uv(:,:,2),[],'all');
u_offset = mean(void_u_range) - grid_u_extent / 2;
v_offset = mean(void_v_range) - grid_v_extent / 2;
home_pos_uv = [0; 0; rise];

% --- Outline trajectory ---
outline_trajectory = [];
new_pos = home_pos_uv;
for i = 1:size(outline_idx, 1)
    gx = outline_idx(i,1); gy = outline_idx(i,2);
    center_uv = squeeze(G_uv(gy, gx, :))';
    pts_uv = hexagonPerimeter(center_uv, hex_side);

    target = [pts_uv(1,:), rise]';
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
    target(3) = 0;
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;

    for layer = 1:num_layers
        h_layer = -((layer - 1) * layer_height);
        hex_pts = [pts_uv, repmat(h_layer, size(pts_uv,1), 1)]';
        outline_trajectory = [outline_trajectory hex_pts]; %#ok<AGROW>
        new_pos = hex_pts(:,end);
    end

    target = [pts_uv(1,:), rise]';
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
end

% --- Fill trajectory ---
fill_trajectory = [];
for i = 1:size(fill_idx, 1)
    gx = fill_idx(i,1); gy = fill_idx(i,2);
    center_uv = squeeze(G_uv(gy, gx, :))';
    pts_uv = hexagonPerimeter(center_uv, hex_side);

    target = [pts_uv(1,:), rise]';
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
    target(3) = 0;
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;

    for layer = 1:num_layers
        h_layer = -((layer - 1) * layer_height);
        hex_pts = [pts_uv, repmat(h_layer, size(pts_uv,1), 1)]';
        fill_trajectory = [fill_trajectory hex_pts]; %#ok<AGROW>
        new_pos = hex_pts(:,end);
    end

    target = [pts_uv(1,:), rise]';
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
end

% --- Deposit trajectory ---
all_idx = [outline_idx; fill_idx];
deposit_trajectory = [];
for i = 1:size(all_idx, 1)
    cx = all_idx(i,1); cy_i = all_idx(i,2);
    cell_center = squeeze(G_uv(cy_i, cx, :))';

    target = [cell_center, rise]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
    target = [cell_center, 0]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
    target = [cell_center, -wall_height]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
    target = [cell_center, rise]';
    deposit_trajectory = [deposit_trajectory linePoints(new_pos, target, num_points)]; %#ok<AGROW>
    new_pos = target;
end

%% =================== 3. UV -> XYZ + ORIENTATION (Section 5 & 6) ===================
traj_uv = [outline_trajectory, fill_trajectory, deposit_trajectory];
Npts_total = size(traj_uv, 2);

full_trajectory_xyz = zeros(3, Npts_total);
normal_vectors = zeros(3, Npts_total);

for k = 1:Npts_total
    u = traj_uv(1,k) + u_offset;
    v = traj_uv(2,k) + v_offset;
    h = traj_uv(3,k);

    theta_k = u / R_cyl;
    Sx = v;
    Sy = cyl_cy + R_cyl * sin(theta_k);
    Sz = cyl_cz + R_cyl * cos(theta_k);

    nx = 0;  ny = sin(theta_k);  nz = cos(theta_k);
    n_hat = [nx; ny; nz];

    full_trajectory_xyz(:,k) = [Sx; Sy; Sz] + h * n_hat;
    normal_vectors(:,k) = n_hat;
end

full_trajectory_m = full_trajectory_xyz * 0.001;
normal_vectors_unit = normal_vectors;

full_trajectory_m(1,:) = -full_trajectory_m(1,:);
full_trajectory_m(3,:) = -full_trajectory_m(3,:);
normal_vectors_unit(1,:) = -normal_vectors_unit(1,:);
normal_vectors_unit(3,:) = -normal_vectors_unit(3,:);

z_offset = -0.35;
full_trajectory_m(3,:) = full_trajectory_m(3,:) + z_offset;

R_targets = zeros(3,3,Npts_total);
for k = 1:Npts_total
    n = normal_vectors_unit(:,k);
    z_tool = -n;
    x_ref = [1; 0; 0];
    if abs(dot(z_tool, x_ref)) > 0.99, x_ref = [0; 1; 0]; end
    x_tool = cross(x_ref, z_tool);
    x_tool = x_tool / norm(x_tool);
    y_tool = cross(z_tool, x_tool);
    R_targets(:,:,k) = [x_tool, y_tool, z_tool];
end

fprintf('Trajectory generated: %d total points.\n', Npts_total);

%% =================== 4. EXTRACT JOINT LIMITS ===================
n_dof = numel(homeConfiguration(mc));
jlim = zeros(n_dof, 2);
joint_idx = 0;
for b = 1:numel(mc.Bodies)
    jnt = mc.Bodies{b}.Joint;
    if ~strcmp(jnt.Type, 'fixed')
        joint_idx = joint_idx + 1;
        lim = jnt.PositionLimits;
        if any(isinf(lim))
            jlim(joint_idx, :) = [-0.5, 0.5];
        else
            jlim(joint_idx, :) = lim;
        end
    end
end

fprintf('Joint limits (8 DOF):\n');
joint_names = {'prism_x','prism_y','J1','J2','J3','J4','J5','J6'};
for j = 1:n_dof
    fprintf('  %s: [%.3f, %.3f]\n', joint_names{j}, jlim(j,1), jlim(j,2));
end

%% =================== 5. SELECT PROBLEMATIC SUBSET ===================
idx_start = 800;
idx_end   = 1400;
idx_end   = min(idx_end, Npts_total);
subset    = idx_start:idx_end;
Nsub      = numel(subset);

P_sub   = full_trajectory_m(:, subset);
R_sub   = R_targets(:,:, subset);

fprintf('\nSubset: points %d–%d (%d points)\n', idx_start, idx_end, Nsub);

%% =================== 6. BASELINE: STANDARD IK ===================
fprintf('\n--- Pass 1: Standard IK (baseline) ---\n');

ik_std = inverseKinematics("RigidBodyTree", mc);
weights = [1 1 1 1 1 1];
q_guess_std = homeConfiguration(mc);

q_std       = zeros(n_dof, Nsub);
err_std     = zeros(1, Nsub);
mu_std      = zeros(1, Nsub);
jlim_viol_std = false(1, Nsub);

for i = 1:Nsub
    T_target = rotm2tform(R_sub(:,:,i));
    T_target(1:3,4) = P_sub(:,i);

    [q_tmp, ~] = ik_std("link6", T_target, weights, q_guess_std);
    q_std(:,i) = q_tmp(:);

    J = geometricJacobian(mc, q_tmp, 'link6');
    mu_std(i) = sqrt(max(0, real(det(J * J'))));

    T_test = getTransform(mc, q_tmp, 'link6');
    err_std(i) = norm(T_target(1:3,4) - T_test(1:3,4));

    for j = 1:n_dof
        if q_tmp(j) < jlim(j,1) - 1e-4 || q_tmp(j) > jlim(j,2) + 1e-4
            jlim_viol_std(i) = true;
            break;
        end
    end

    q_guess_std = q_tmp;

    if mod(i, 50) == 0
        fprintf('  [STD] %d/%d  err=%.3f mm  mu=%.4f\n', ...
            i, Nsub, err_std(i)*1000, mu_std(i));
    end
end

fprintf('Standard IK — max err: %.3f mm, mean err: %.3f mm, limit violations: %d/%d\n', ...
    max(err_std)*1000, mean(err_std)*1000, sum(jlim_viol_std), Nsub);
fprintf('  Points > 0.5mm: %d, Points > 1mm: %d\n', ...
    sum(err_std > 0.5e-3), sum(err_std > 1e-3));

%% =================== 7. APF + SUPER-TWISTING IK (Multi-Seed + APF) ===================
fprintf('\n--- Pass 2: Multi-Seed APF + Super-Twisting IK ---\n');

% --- Parameters ---
params = struct();
params.pos_tol       = 1e-4;       % 0.1 mm convergence target
params.err_threshold = 0.5e-3;     % 0.5 mm — trigger advanced solving
params.num_restarts  = 15;         % null-space multi-start attempts
params.perturb_scale = 0.6;        % null-space perturbation amplitude [rad]
params.random_scale  = 0.8;        % full-space random perturbation amplitude
% APF + STW (Phase 3)
params.max_iter      = 200;
params.dt            = 0.01;
params.alpha         = 0.1;
params.eta           = 0.001;
params.limit_margin  = 0.12;
params.mu_threshold  = 0.02;
params.lambda_max    = 0.15;
params.K1            = 0.3;
params.K2            = 0.1;
params.v0            = 1.0;
params.kv            = 4.0;
params.e_u           = 1e-6;
params.omega_max     = 0.5;

q_apf       = zeros(n_dof, Nsub);
err_apf     = zeros(1, Nsub);
mu_apf      = zeros(1, Nsub);
iter_apf    = zeros(1, Nsub);
jlim_viol_apf = false(1, Nsub);
apf_active  = false(1, Nsub);
phase_log   = zeros(1, Nsub);     % which phase solved it (0=std, 1/2/3)

ik_apf = inverseKinematics("RigidBodyTree", mc);
q_home = homeConfiguration(mc);

% Two warm-start chains:
q_prev_any  = q_home;   % sequential chain (always updated)
q_prev_good = q_home;   % clean chain (only updated on success)

weight_sets = {[1 1 1 1 1 1], ...
               [1 1 1 0.3 0.3 0.3], ...
               [1 1 1 0.1 0.1 0.1], ...
               [1 1 1 0.01 0.01 0.01]};

% #region agent log
dbg_log = fullfile(tempdir, 'debug-62765e.log');
fprintf('DEBUG LOG: %s\n', dbg_log);
if exist(dbg_log, 'file'), delete(dbg_log); end
dbg_t_total = tic;
% #endregion

for i = 1:Nsub
    % #region agent log
    dbg_t_pt = tic;
    % #endregion

    T_target = rotm2tform(R_sub(:,:,i));
    T_target(1:3,4) = P_sub(:,i);
    p_des = P_sub(:,i);

    % === Seed pool: 3 independent starting points ===
    seeds = {q_prev_any, q_prev_good, q_home};

    best_q   = q_prev_any;
    best_err = inf;

    % #region agent log
    dbg_t_ph1 = tic;
    dbg_ph1_ik_calls = 0;
    % #endregion

    % --- Phase 1: Try 3 seeds with full weights (fast screening) ---
    for s = 1:numel(seeds)
        [q_try, ~] = ik_apf("link6", T_target, [1 1 1 1 1 1], seeds{s});
        % #region agent log
        dbg_ph1_ik_calls = dbg_ph1_ik_calls + 1;
        % #endregion
        T_fk = getTransform(mc, q_try, 'link6');
        err = norm(p_des - T_fk(1:3,4));
        if err < best_err
            best_err = err; best_q = q_try;
        end
        if best_err < params.pos_tol, break; end
    end

    % If still bad, try relaxed orientation from the best seed
    if best_err > params.err_threshold
        for w = 2:numel(weight_sets)
            [q_try, ~] = ik_apf("link6", T_target, weight_sets{w}, best_q);
            % #region agent log
            dbg_ph1_ik_calls = dbg_ph1_ik_calls + 1;
            % #endregion
            T_fk = getTransform(mc, q_try, 'link6');
            err = norm(p_des - T_fk(1:3,4));
            if err < best_err
                best_err = err; best_q = q_try;
            end
            if best_err < params.pos_tol, break; end
        end
    end

    % #region agent log
    dbg_ph1_ms = toc(dbg_t_ph1)*1000;
    % #endregion

    solved_phase = 0;
    % #region agent log
    dbg_ph2_ms = 0; dbg_ph2b_ms = 0; dbg_ph3_ms = 0;
    dbg_ph2_ik_calls = 0; dbg_ph2b_ik_calls = 0;
    % #endregion

    if best_err > params.err_threshold
        apf_active(i) = true;
        solved_phase = 1;

        % #region agent log
        dbg_t_ph2 = tic;
        % #endregion

        % --- Phase 2: Focused null-space restarts (1 seed, few attempts) ---
        if best_err > params.pos_tol
            J_ns = geometricJacobian(mc, best_q, 'link6');
            N_proj = eye(n_dof) - pinv(J_ns) * J_ns;

            for trial = 1:5
                delta_ns = randn(1, n_dof) * params.perturb_scale;
                q_pert = best_q + delta_ns * N_proj';
                q_pert = max(jlim(:,1)', min(jlim(:,2)', q_pert));

                [q_try, ~] = ik_apf("link6", T_target, [1 1 1 1 1 1], q_pert);
                % #region agent log
                dbg_ph2_ik_calls = dbg_ph2_ik_calls + 1;
                % #endregion
                T_fk = getTransform(mc, q_try, 'link6');
                err = norm(p_des - T_fk(1:3,4));
                if err < best_err
                    best_err = err; best_q = q_try; solved_phase = 2;
                end
                if best_err < params.pos_tol, break; end
            end
        end

        % #region agent log
        dbg_ph2_ms = toc(dbg_t_ph2)*1000;
        dbg_t_ph2b = tic;
        % #endregion

        % --- Phase 2b: A few random restarts ---
        if best_err > params.pos_tol
            for trial = 1:3
                q_rand = q_home + randn(1, n_dof) * params.random_scale;
                q_rand = max(jlim(:,1)', min(jlim(:,2)', q_rand));
                [q_try, ~] = ik_apf("link6", T_target, [1 1 1 1 1 1], q_rand);
                % #region agent log
                dbg_ph2b_ik_calls = dbg_ph2b_ik_calls + 1;
                % #endregion
                T_fk = getTransform(mc, q_try, 'link6');
                err = norm(p_des - T_fk(1:3,4));
                if err < best_err
                    best_err = err; best_q = q_try; solved_phase = 2;
                end
                if best_err < params.pos_tol, break; end
            end
        end

        % #region agent log
        dbg_ph2b_ms = toc(dbg_t_ph2b)*1000;
        dbg_t_ph3 = tic;
        % #endregion

        % --- Phase 3: APF + STW gradient refinement from best candidate ---
        if best_err > params.pos_tol
            [q_refined, refine_info] = apf_stw_refine(mc, T_target, best_q, jlim, params);
            if refine_info.pos_err < best_err
                best_err = refine_info.pos_err;
                best_q = q_refined;
                solved_phase = 3;
            end
            iter_apf(i) = refine_info.iter;
        end

        % #region agent log
        dbg_ph3_ms = toc(dbg_t_ph3)*1000;
        % #endregion
    end

    q_apf(:,i) = best_q(:);
    err_apf(i) = best_err;
    phase_log(i) = solved_phase;

    J_final = geometricJacobian(mc, best_q, 'link6');
    mu_apf(i) = sqrt(max(0, real(det(J_final * J_final'))));

    for j = 1:n_dof
        if best_q(j) < jlim(j,1) - 1e-4 || best_q(j) > jlim(j,2) + 1e-4
            jlim_viol_apf(i) = true; break;
        end
    end

    % Update chains
    q_prev_any = q_apf(:,i)';
    if best_err < params.err_threshold
        q_prev_good = q_apf(:,i)';
    end

    % #region agent log — write timing for every 10th point + all problematic points
    dbg_pt_ms = toc(dbg_t_pt)*1000;
    if mod(i,10)==0 || apf_active(i)
        fid = fopen(dbg_log, 'a');
        fprintf(fid, '{"sessionId":"62765e","hypothesisId":"H1-H5","location":"test_obstacle_avoidance.m:loop","message":"point_timing","data":{"i":%d,"active":%d,"ph1_ms":%.1f,"ph2_ms":%.1f,"ph2b_ms":%.1f,"ph3_ms":%.1f,"total_ms":%.1f,"ph1_ik":%d,"ph2_ik":%d,"ph2b_ik":%d,"err_mm":%.4f,"phase":%d},"timestamp":%d}\n', ...
            i, apf_active(i), dbg_ph1_ms, dbg_ph2_ms, dbg_ph2b_ms, dbg_ph3_ms, dbg_pt_ms, dbg_ph1_ik_calls, dbg_ph2_ik_calls, dbg_ph2b_ik_calls, best_err*1000, solved_phase, round(posixtime(datetime('now'))*1000));
        fclose(fid);
    end
    % #endregion

    if mod(i, 50) == 0
        tag = '    ';
        if apf_active(i), tag = sprintf('Ph%d ', solved_phase); end
        fprintf('  [%s] %d/%d  err=%.3f mm  mu=%.4f\n', ...
            tag, i, Nsub, err_apf(i)*1000, mu_apf(i));
    end
end

% #region agent log — write total elapsed
fid = fopen(dbg_log, 'a');
fprintf(fid, '{"sessionId":"62765e","hypothesisId":"H5","location":"test_obstacle_avoidance.m:end","message":"total_elapsed","data":{"elapsed_s":%.1f,"Nsub":%d},"timestamp":%d}\n', ...
    toc(dbg_t_total), Nsub, round(posixtime(datetime('now'))*1000));
fclose(fid);
% #endregion

fprintf('APF+STW IK — max err: %.3f mm, mean err: %.3f mm, limit violations: %d/%d\n', ...
    max(err_apf)*1000, mean(err_apf)*1000, sum(jlim_viol_apf), Nsub);
fprintf('APF activated on %d/%d points (%.1f%%)\n', sum(apf_active), Nsub, ...
    100*sum(apf_active)/Nsub);
fprintf('Solved by phase: Ph1=%d, Ph2=%d, Ph3=%d, unchanged=%d\n', ...
    sum(phase_log==1), sum(phase_log==2), sum(phase_log==3), sum(phase_log==0));

%% =================== 8. DIAGNOSTIC COMPARISON PLOTS ===================
t_sub = (0:Nsub-1) * 0.05;
pt_idx = subset;

% --- Figure 1: Before / After tracking error ---
figure('Name','Tracking Error Comparison','Position',[50 50 1100 500]);

subplot(2,1,1);
plot(pt_idx, err_std*1000, 'r-', 'LineWidth', 1.0);
hold on;
plot(pt_idx, err_apf*1000, 'b-', 'LineWidth', 1.2);
yline(0.5, 'k--', 'LineWidth', 0.8);
ylabel('Position Error [mm]');
legend('Standard IK', 'APF + STW', '0.5 mm threshold', 'Location', 'best');
title('Tracking Error Comparison');
grid on;

subplot(2,1,2);
improvement = (err_std - err_apf) * 1000;
bar(pt_idx, improvement, 1, 'FaceColor', [0.2 0.6 0.3], 'EdgeColor', 'none');
hold on; yline(0, 'k-');
ylabel('\Delta Error [mm] (positive = improvement)');
xlabel('Trajectory Point Index');
title('Error Reduction (Standard - APF)');
grid on;

% --- Figure 2: Manipulability comparison ---
figure('Name','Manipulability Comparison','Position',[80 80 1100 500]);

subplot(2,1,1);
plot(pt_idx, mu_std, 'r-', 'LineWidth', 1.0);
hold on;
plot(pt_idx, mu_apf, 'b-', 'LineWidth', 1.2);
yline(params.mu_threshold, 'k--', 'LineWidth', 0.8);
ylabel('$\sqrt{\det(J\,J^T)}$', 'Interpreter', 'latex');
legend('Standard IK', 'APF + STW', '\mu threshold', 'Location', 'best');
title('Manipulability Along Subset');
grid on;

subplot(2,1,2);
hold on;
ph1_mask = phase_log == 1;
ph2_mask = phase_log == 2;
ph3_mask = phase_log == 3;
if any(ph1_mask), stem(pt_idx(ph1_mask), ones(1,sum(ph1_mask)), 'filled', 'MarkerSize', 4, 'Color', [0.2 0.7 0.2]); end
if any(ph2_mask), stem(pt_idx(ph2_mask), 2*ones(1,sum(ph2_mask)), 'filled', 'MarkerSize', 4, 'Color', [0.2 0.2 0.8]); end
if any(ph3_mask), stem(pt_idx(ph3_mask), 3*ones(1,sum(ph3_mask)), 'filled', 'MarkerSize', 4, 'Color', [0.8 0.2 0.2]); end
ylabel('Solving Phase');
xlabel('Trajectory Point Index');
title('Which Phase Solved Each Point (1=Relaxed Wt, 2=Null-Space, 3=APF+STW)');
yticks([1 2 3]); yticklabels({'Ph1: Relaxed','Ph2: Null-Space','Ph3: APF+STW'});
ylim([0 4]);
grid on;

% --- Figure 3: Joint trajectories (limit avoidance) ---
figure('Name','Joint Trajectories','Position',[110 110 1200 700]);
for j = 1:n_dof
    subplot(4,2,j);
    plot(pt_idx, q_std(j,:), 'r-', 'LineWidth', 0.9);
    hold on;
    plot(pt_idx, q_apf(j,:), 'b-', 'LineWidth', 1.1);
    yline(jlim(j,1), 'k--', 'LineWidth', 0.7);
    yline(jlim(j,2), 'k--', 'LineWidth', 0.7);
    fill_y = [jlim(j,1), jlim(j,1) + params.limit_margin*(jlim(j,2)-jlim(j,1))];
    yline(fill_y(2), 'Color', [0.9 0.7 0.7], 'LineStyle', ':');
    yline(jlim(j,2) - params.limit_margin*(jlim(j,2)-jlim(j,1)), ...
        'Color', [0.9 0.7 0.7], 'LineStyle', ':');
    ylabel(joint_names{j});
    if j == 1, legend('Standard','APF','Limits','Location','best'); end
    if j >= 7, xlabel('Point Index'); end
    grid on;
    title(sprintf('%s  [%.2f, %.2f]', joint_names{j}, jlim(j,1), jlim(j,2)));
end
sgtitle('Joint Trajectories — Limit Avoidance');

% --- Figure 4: APF potential field slice ---
figure('Name','APF Potential Field','Position',[140 140 900 700]);

worst_std = find(err_std == max(err_std), 1);
q_at_worst = q_std(:, worst_std)';
T_at_worst = rotm2tform(R_sub(:,:,worst_std));
T_at_worst(1:3,4) = P_sub(:, worst_std);

j_scan = [3, 4];
Ngrid = 60;
q3_range = linspace(jlim(j_scan(1),1), jlim(j_scan(1),2), Ngrid);
q4_range = linspace(jlim(j_scan(2),1), jlim(j_scan(2),2), Ngrid);
[Q3, Q4] = meshgrid(q3_range, q4_range);
U_field = zeros(size(Q3));

for r = 1:Ngrid
    for c = 1:Ngrid
        q_probe = q_at_worst;
        q_probe(j_scan(1)) = Q3(r,c);
        q_probe(j_scan(2)) = Q4(r,c);

        T_fk = getTransform(mc, q_probe, 'link6');
        pos_err = norm(T_at_worst(1:3,4) - T_fk(1:3,4));
        U_attract = pos_err^2;

        U_repel = 0;
        for j = 1:n_dof
            range_j = jlim(j,2) - jlim(j,1);
            margin = params.limit_margin * range_j;
            d_low = q_probe(j) - jlim(j,1);
            d_high = jlim(j,2) - q_probe(j);
            if d_low < margin && d_low > 0
                U_repel = U_repel + params.eta / (d_low + 1e-6);
            end
            if d_high < margin && d_high > 0
                U_repel = U_repel + params.eta / (d_high + 1e-6);
            end
        end

        U_field(r,c) = U_attract + params.alpha * U_repel;
    end
end

subplot(1,2,1);
contourf(rad2deg(Q3), rad2deg(Q4), log10(U_field + 1e-10), 30, 'LineStyle', 'none');
colorbar; hold on;
plot(rad2deg(q_at_worst(j_scan(1))), rad2deg(q_at_worst(j_scan(2))), ...
    'rx', 'MarkerSize', 14, 'LineWidth', 2);
if worst_std <= size(q_apf, 2)
    q_apf_worst = q_apf(:, worst_std)';
    plot(rad2deg(q_apf_worst(j_scan(1))), rad2deg(q_apf_worst(j_scan(2))), ...
        'go', 'MarkerSize', 12, 'LineWidth', 2);
    legend('', 'Std IK soln', 'APF soln', 'Location', 'best');
else
    legend('', 'Std IK soln', 'Location', 'best');
end
xlabel(sprintf('%s [deg]', joint_names{j_scan(1)}));
ylabel(sprintf('%s [deg]', joint_names{j_scan(2)}));
title(sprintf('APF Potential (log_{10}) at worst point #%d', subset(worst_std)));
grid on;

subplot(1,2,2);
contourf(rad2deg(Q3), rad2deg(Q4), log10(U_field + 1e-10), 30, 'LineStyle', 'none');
colorbar; hold on;
plot(rad2deg(q_at_worst(j_scan(1))), rad2deg(q_at_worst(j_scan(2))), ...
    'rx', 'MarkerSize', 14, 'LineWidth', 2);
xlim(rad2deg(q_at_worst(j_scan(1))) + [-30, 30]);
ylim(rad2deg(q_at_worst(j_scan(2))) + [-30, 30]);
if worst_std <= size(q_apf, 2)
    plot(rad2deg(q_apf_worst(j_scan(1))), rad2deg(q_apf_worst(j_scan(2))), ...
        'go', 'MarkerSize', 12, 'LineWidth', 2);
end
xlabel(sprintf('%s [deg]', joint_names{j_scan(1)}));
ylabel(sprintf('%s [deg]', joint_names{j_scan(2)}));
title('Zoomed View Around Worst Point');
grid on;

sgtitle(sprintf('APF Potential Field Slice (%s vs %s)', ...
    joint_names{j_scan(1)}, joint_names{j_scan(2)}));

% --- Summary table ---
fprintf('\n========== SUMMARY ==========\n');
fprintf('  Metric               Standard IK    APF+STW IK\n');
fprintf('  Max error [mm]       %10.3f    %10.3f\n', max(err_std)*1000, max(err_apf)*1000);
fprintf('  Mean error [mm]      %10.3f    %10.3f\n', mean(err_std)*1000, mean(err_apf)*1000);
fprintf('  Median error [mm]    %10.3f    %10.3f\n', median(err_std)*1000, median(err_apf)*1000);
fprintf('  Points > 0.5 mm      %10d    %10d\n', sum(err_std > 0.5e-3), sum(err_apf > 0.5e-3));
fprintf('  Points > 1.0 mm      %10d    %10d\n', sum(err_std > 1e-3), sum(err_apf > 1e-3));
fprintf('  Joint limit viols    %10d    %10d\n', sum(jlim_viol_std), sum(jlim_viol_apf));
fprintf('  Mean manipulability  %10.4f    %10.4f\n', mean(mu_std), mean(mu_apf));
fprintf('  Min manipulability   %10.4f    %10.4f\n', min(mu_std), min(mu_apf));
fprintf('=============================\n');


%% =================== APF + SUPER-TWISTING REFINEMENT (Phase 3) ===================
function [q_sol, info] = apf_stw_refine(mc, T_target, q_init, jlim, params)
    n = numel(q_init);
    q = q_init(:)';
    Omega = zeros(1, n);
    q_dot_prev = zeros(1, n);

    R_des = T_target(1:3, 1:3);
    p_des = T_target(1:3, 4);

    best_q = q;
    T_init = getTransform(mc, q, 'link6');
    best_err = norm(p_des - T_init(1:3,4));
    stall_count = 0;
    final_iter = 0;

    for iter = 1:params.max_iter
        T_curr = getTransform(mc, q, 'link6');
        p_curr = T_curr(1:3, 4);
        R_curr = T_curr(1:3, 1:3);

        pos_err = p_des - p_curr;
        R_err = R_des * R_curr';
        rot_err = 0.5 * [R_err(3,2) - R_err(2,3); ...
                         R_err(1,3) - R_err(3,1); ...
                         R_err(2,1) - R_err(1,2)];
        dx = [pos_err; rot_err];

        pos_err_val = norm(pos_err);

        if pos_err_val < best_err - 1e-7
            best_err = pos_err_val;
            best_q = q;
            stall_count = 0;
        else
            stall_count = stall_count + 1;
        end

        final_iter = iter;
        if pos_err_val < params.pos_tol, break; end
        if stall_count > 80, break; end

        J = geometricJacobian(mc, q, 'link6');
        JJt = J * J';
        mu_val = sqrt(max(0, real(det(JJt))));

        if mu_val < params.mu_threshold
            lambda = params.lambda_max * (1 - (mu_val / params.mu_threshold)^2);
        else
            lambda = 0;
        end
        J_dls = J' / (JJt + lambda^2 * eye(6));
        q_attract = (J_dls * dx)';

        q_repel = zeros(1, n);
        for j = 1:n
            range_j = jlim(j,2) - jlim(j,1);
            if range_j < 1e-6, continue; end
            margin = params.limit_margin * range_j;

            d_low  = q(j) - jlim(j,1);
            d_high = jlim(j,2) - q(j);

            if d_low < margin
                d_safe = max(d_low, 1e-6);
                q_repel(j) = q_repel(j) + params.eta / d_safe^2;
            end
            if d_high < margin
                d_safe = max(d_high, 1e-6);
                q_repel(j) = q_repel(j) - params.eta / d_safe^2;
            end
        end

        E_og = q_attract + params.alpha * q_repel;

        d_goal = pos_err_val;
        v_d = min(params.v0, params.kv * sqrt(d_goal));
        E_qd = v_d * E_og / max(norm(E_og), params.e_u);

        S_q = q_dot_prev - E_qd;
        abs_S = abs(S_q);
        sgn_S = sign(S_q + 1e-15);

        stw_prop = -params.K1 * sqrt(abs_S) .* sgn_S;
        Omega = Omega - params.K2 * sgn_S * params.dt;
        Omega = max(-params.omega_max, min(params.omega_max, Omega));

        q_dot = E_qd + stw_prop + Omega;
        q_dot_prev = q_dot;

        q = q + params.dt * q_dot;
        q = max(jlim(:,1)', min(jlim(:,2)', q));
    end

    q_sol = best_q;
    info.iter = final_iter;
    info.pos_err = best_err;
end


%% =================== HELPER FUNCTIONS (from MuffinFresa_ConformalMapping) ===================
function G = createGrid(Nx, Ny, hex_side)
    xSpacing=1.5*hex_side;
    ySpacing=hex_side*sqrt(3);
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
        pts = [pts; x(1:end-1)', y(1:end-1)']; %#ok<AGROW>
    end
end

function target_trajectory = linePoints(start_pos, end_pos, num_points)
    target_trajectory = [linspace(start_pos(1), end_pos(1), num_points); ...
                         linspace(start_pos(2), end_pos(2), num_points); ...
                         linspace(start_pos(3), end_pos(3), num_points)];
end
