function [logData, report] = gcode_executor(traj, s, mc, config)
%GCODE_EXECUTOR Execute a parsed G-code trajectory on XY + MyCobot hardware.
%   [LOGDATA, REPORT] = GCODE_EXECUTOR(TRAJ, S, MC, CONFIG) executes the
%   parsed trajectory returned by GCODE_PARSER. XY motion is sent through
%   the serial object S, Z motion is sent through the MyCobot bridge MC,
%   and extrusion is logged as a placeholder for future hardware control.

    if nargin < 2, s = []; end
    if nargin < 3, mc = []; end
    if nargin < 4 || isempty(config), config = struct(); end

    if ischar(traj) || isstring(traj)
        traj = gcode_parser(traj);
    end

    config = apply_defaults(config);
    [logData, report] = exec_gcode_trajectory(traj, s, mc, config);
end

function [logData, report] = exec_gcode_trajectory(traj, s, mc, config)
    if ~isstruct(traj) || ~isfield(traj, 'sequence')
        error('gcode_executor:InvalidTrajectory', 'Trajectory must come from gcode_parser.');
    end

    nMoves = count_moves(traj.sequence);
    logData = initialize_log(nMoves, numel(traj.sequence));
    controller = initialize_keyboard_controller(config.enableKeyboardControl);
    cleanupObj = onCleanup(@() cleanup_keyboard_controller(controller)); %#ok<NASGU>

    state = struct(...
        'lastXY', [0, 0], ...
        'lastZ', 0, ...
        'lastE', 0, ...
        'lastFeed', config.defaultXYFeedrate, ...
        'aborted', false);

    moveIndex = 0;
    totalXYDistance = 0;
    totalZDistance = 0;
    totalMaterial = 0;
    t0 = tic;
    finalPose = [NaN, NaN, NaN, NaN, NaN, NaN];

    if config.homeOnStart
        homeEntry = struct('code', 'G28', 'params', struct(), 'raw', 'G28');
        [state, ~] = execute_non_move_command(homeEntry, s, mc, config, state);
    end

    for i = 1:numel(traj.sequence)
        if should_abort(controller)
            state.aborted = true;
            break;
        end
        wait_if_paused(controller);

        entry = traj.sequence{i};
        logData.sequence_log{i} = entry;

        if strcmp(entry.type, 'move')
            moveIndex = moveIndex + 1;
            target = apply_coordinate_transform(entry.target, config);
            if isnan(target(5))
                target(5) = state.lastFeed;
            else
                state.lastFeed = target(5);
            end

            target(1:2) = clamp_xy_target(target(1:2), config);
            target(3) = min(max(target(3), config.robotZMin), config.robotZMax);

            [xyActual, xyStatus] = move_xy_gcode(target, s, config, state.lastXY);
            [zActual, zStatus, poseAfter] = move_robot_z(target(3), mc, config);
            [materialDelta, materialTotal] = extrude_material(target(4), state.lastE, totalMaterial);

            desired = target(1:3);
            actual = [xyActual, zActual];
            startPose = [state.lastXY, state.lastZ];
            totalXYDistance = totalXYDistance + norm(desired(1:2) - startPose(1:2));
            totalZDistance = totalZDistance + abs(desired(3) - startPose(3));
            state.lastXY = xyActual;
            state.lastZ = zActual;
            state.lastE = target(4);
            totalMaterial = materialTotal;
            if ~isempty(poseAfter)
                finalPose = poseAfter;
            else
                finalPose(1:3) = [xyActual, zActual];
            end

            logData = log_telemetry(logData, moveIndex, toc(t0), desired, actual, target(4), materialDelta, xyStatus, zStatus, entry);
        else
            [state, note] = execute_non_move_command(entry, s, mc, config, state);
            logData.event_log{i} = note;
        end
    end

    logData = trim_log(logData, moveIndex);
    logData.material_total = totalMaterial;

    report = build_report(traj, logData, totalXYDistance, totalZDistance, totalMaterial, toc(t0), state.aborted, finalPose);

    if config.plotTelemetry && moveIndex > 0
        plot_telemetry(logData, report);
    end
end

function nMoves = count_moves(sequence)
    nMoves = 0;
    for i = 1:numel(sequence)
        if isfield(sequence{i}, 'type') && strcmp(sequence{i}.type, 'move')
            nMoves = nMoves + 1;
        end
    end
end

function logData = initialize_log(nMoves, nSequence)
    logData = struct();
    logData.timestamp = nan(nMoves,1);
    logData.ref_log = nan(nMoves,3);
    logData.r_log = nan(nMoves,3);
    logData.e_log = nan(nMoves,3);
    logData.e_cmd = nan(nMoves,1);
    logData.material_delta = zeros(nMoves,1);
    logData.material_total = 0;
    logData.xy_status = cell(nMoves,1);
    logData.z_status = cell(nMoves,1);
    logData.sequence_log = cell(nSequence,1);
    logData.event_log = cell(nSequence,1);
    logData.power_log = nan(nMoves,1);
end

function controller = initialize_keyboard_controller(enableKeyboardControl)
    controller = struct('enabled', false, 'figure', []);
    if ~enableKeyboardControl
        return;
    end

    try
        controller.figure = figure( ...
            'Name', 'G-code execution controls', ...
            'NumberTitle', 'off', ...
            'MenuBar', 'none', ...
            'ToolBar', 'none', ...
            'Color', 'w', ...
            'HandleVisibility', 'callback', ...
            'KeyPressFcn', @keyboard_callback);
        controller.enabled = true;
        setappdata(controller.figure, 'gcode_pause', false);
        setappdata(controller.figure, 'gcode_abort', false);
        annotation(controller.figure, 'textbox', [0.1 0.3 0.8 0.4], ...
            'String', {'Space: pause/resume', 'Q: abort execution'}, ...
            'FitBoxToText', 'on', 'EdgeColor', 'none');
        drawnow;
    catch
        controller.enabled = false;
        controller.figure = [];
    end
end

function keyboard_callback(src, evt)
    if strcmp(evt.Key, 'space')
        paused = getappdata(src, 'gcode_pause');
        setappdata(src, 'gcode_pause', ~paused);
    elseif strcmpi(evt.Key, 'q')
        setappdata(src, 'gcode_abort', true);
    end
end

function cleanup_keyboard_controller(controller)
    if controller.enabled && ishghandle(controller.figure)
        try
            close(controller.figure);
        catch
        end
    end
end

function tf = should_abort(controller)
    tf = false;
    if controller.enabled && ishghandle(controller.figure)
        tf = islogical(getappdata(controller.figure, 'gcode_abort')) && getappdata(controller.figure, 'gcode_abort');
    end
end

function wait_if_paused(controller)
    while controller.enabled && ishghandle(controller.figure)
        paused = getappdata(controller.figure, 'gcode_pause');
        aborted = getappdata(controller.figure, 'gcode_abort');
        if aborted || ~paused
            return;
        end
        drawnow;
        pause(0.1);
    end
end

function target = apply_coordinate_transform(target, config)
    xyz = target(1:3);
    if isa(config.transformFcn, 'function_handle')
        xyz = config.transformFcn(xyz);
    end
    xyz = xyz(:).';
    xyz(1:2) = xyz(1:2) .* config.xyScale + config.xyOffset;
    xyz(3) = xyz(3) * config.zScale + config.zOffset;
    target(1:3) = xyz;
end

function [actualXY, status] = move_xy_gcode(target, s, config, estimatedXY)
    targetXY = clamp_xy_target(target(1:2), config);
    targetFeed = target(5);
    if isnan(targetFeed) || targetFeed <= 0
        targetFeed = config.defaultXYFeedrate;
    end

    status = struct('ok', true, 'source', 'DRYRUN', 'command', '', 'feedback', false);
    status.command = build_xy_command(targetXY, targetFeed, estimatedXY);

    if config.dryRun || isempty(s)
        actualXY = targetXY;
        return;
    end

    actualXY = estimatedXY;
    try
        writeline(s, 'G90');
        pause(0.01);
        drain_serial(s, 1);
        writeline(s, status.command);
    catch ME
        status.ok = false;
        status.source = 'SERIAL_ERROR';
        status.error = ME.message;
        actualXY = targetXY;
        return;
    end

    tStart = tic;
    lastMeasured = estimatedXY;
    stallCount = 0;
    feedbackSeen = false;
    while toc(tStart) < config.xyTimeout
        pause(config.xyPollPeriod);
        [xm, ym, ok, src] = getXY(s);
        if ok && ~any(isnan([xm, ym]))
            actualXY = [xm, ym];
            status.source = src;
            status.feedback = true;
            feedbackSeen = true;
            if norm(actualXY - targetXY) <= config.xyTolerance
                break;
            end
            if norm(actualXY - lastMeasured) <= config.xyStallTolerance
                stallCount = stallCount + 1;
            else
                stallCount = 0;
            end
            lastMeasured = actualXY;
            if stallCount >= config.xyStallSamples
                status.ok = false;
                status.source = [src, ':STALL'];
                break;
            end
        end
    end

    if ~feedbackSeen
        actualXY = targetXY;
        status.source = 'EST';
    end
end

function cmd = build_xy_command(targetXY, feedrate, estimatedXY)
    includeX = isempty(estimatedXY) || numel(estimatedXY) < 2 || isnan(estimatedXY(1)) || abs(targetXY(1) - estimatedXY(1)) > eps;
    includeY = isempty(estimatedXY) || numel(estimatedXY) < 2 || isnan(estimatedXY(2)) || abs(targetXY(2) - estimatedXY(2)) > eps;
    tokens = {'G1'};
    if includeX
        tokens{end+1} = sprintf('X%.3f', targetXY(1)); %#ok<AGROW>
    end
    if includeY
        tokens{end+1} = sprintf('Y%.3f', targetXY(2)); %#ok<AGROW>
    end
    tokens{end+1} = sprintf('F%.0f', feedrate); %#ok<AGROW>
    cmd = strjoin(tokens, ' ');
end

function targetXY = clamp_xy_target(targetXY, config)
    targetXY(1) = min(max(targetXY(1), config.Xmin), config.Xmax);
    targetXY(2) = min(max(targetXY(2), config.Ymin), config.Ymax);
    if isfinite(config.Rsafe)
        radius = hypot(targetXY(1), targetXY(2));
        if radius > config.Rsafe && radius > 0
            scale = config.Rsafe / radius;
            targetXY = targetXY * scale;
        end
    end
end

function [actualZ, status, poseAfter] = move_robot_z(targetZ, mc, config)
    status = struct('ok', true, 'source', 'DRYRUN', 'pose', []);
    poseAfter = [];
    if isnan(targetZ)
        actualZ = NaN;
        return;
    end

    targetZ = min(max(targetZ, config.robotZMin), config.robotZMax);
    if config.dryRun || isempty(mc)
        actualZ = targetZ;
        poseAfter = [NaN, NaN, actualZ, NaN, NaN, NaN];
        return;
    end

    pose = safe_get_pose(mc);
    if isempty(pose)
        pose = config.robotPoseReference;
    end
    if isempty(pose)
        pose = [0, 0, targetZ, 0, 0, 0];
    end

    pose(3) = targetZ;
    try
        mc.move_cartesian(pose(1), pose(2), pose(3), pose(4), pose(5), pose(6), int32(config.myCobotSpeed), true, config.myCobotTimeout);
        poseAfter = safe_get_pose(mc);
        if isempty(poseAfter)
            actualZ = targetZ;
            poseAfter = [NaN, NaN, actualZ, NaN, NaN, NaN];
        else
            actualZ = poseAfter(3);
            status.pose = poseAfter;
            status.source = 'MYCOBOT';
        end
    catch ME
        actualZ = targetZ;
        poseAfter = [NaN, NaN, actualZ, NaN, NaN, NaN];
        status.ok = false;
        status.source = 'MYCOBOT_ERROR';
        status.error = ME.message;
    end
end

function pose = safe_get_pose(mc)
    pose = [];
    try
        poseObj = mc.get_pose();
        coords = pylist_to_double(poseObj{'coords'});
        if ~isempty(coords)
            pose = reshape(double(coords), 1, []);
            if numel(pose) < 6
                pose(6) = NaN;
            else
                pose = pose(1:6);
            end
        end
    catch
        pose = [];
    end
end

function [materialDelta, materialTotal] = extrude_material(targetE, previousE, currentTotal)
    if nargin < 3
        currentTotal = 0;
    end
    if isnan(targetE)
        materialDelta = 0;
        materialTotal = currentTotal;
        return;
    end

    materialDelta = targetE - previousE;
    if materialDelta < 0
        materialDelta = 0;
    end
    materialTotal = currentTotal + materialDelta;
end

function logData = log_telemetry(logData, index, timestamp, desired, actual, eCommand, materialDelta, xyStatus, zStatus, entry)
    logData.timestamp(index) = timestamp;
    logData.ref_log(index,:) = desired;
    logData.r_log(index,:) = actual;
    logData.e_log(index,:) = desired - actual;
    logData.e_cmd(index) = eCommand;
    logData.material_delta(index) = materialDelta;
    logData.xy_status{index} = xyStatus;
    logData.z_status{index} = zStatus;
    logData.event_log{index} = entry;
end

function [state, note] = execute_non_move_command(entry, s, mc, config, state)
    note = entry.raw;
    params = struct();
    if isfield(entry, 'params')
        params = entry.params;
    end

    switch upper(entry.code)
        case 'G28'
            if isempty(fieldnames(params)) || isfield(params, 'X')
                state.lastXY(1) = 0;
            end
            if isempty(fieldnames(params)) || isfield(params, 'Y')
                state.lastXY(2) = 0;
            end
            if isempty(fieldnames(params)) || isfield(params, 'Z')
                state.lastZ = 0;
            end
            if config.dryRun
                return;
            end
            if ~isempty(s)
                send_serial_command(s, build_home_command(params));
            end
            if ~isempty(mc) && (isempty(fieldnames(params)) || isfield(params, 'Z')) && config.homeMyCobotOnG28
                try
                    mc.home(py.list(num2cell(config.myCobotHomePosition)));
                catch
                end
            end

        case {'G90', 'G91', 'G20', 'G21'}
            if ~config.dryRun && ~isempty(s)
                send_serial_command(s, upper(entry.raw));
            end

        case 'G92'
            if isfield(params, 'X') && ~isempty(params.X)
                state.lastXY(1) = params.X;
            end
            if isfield(params, 'Y') && ~isempty(params.Y)
                state.lastXY(2) = params.Y;
            end
            if isfield(params, 'Z') && ~isempty(params.Z)
                state.lastZ = params.Z;
            end
            if isfield(params, 'E') && ~isempty(params.E)
                state.lastE = params.E;
            end
            if ~config.dryRun && ~isempty(s)
                xyCmd = build_g92_command(params);
                if ~isempty(xyCmd)
                    send_serial_command(s, xyCmd);
                end
            end

        otherwise
            if ~config.dryRun && ~isempty(s) && should_forward_serial_command(entry.code, config)
                send_serial_command(s, upper(entry.raw));
            end
    end
end

function send_serial_command(s, commandText)
    if isempty(commandText)
        return;
    end
    writeline(s, commandText);
    pause(0.02);
    drain_serial(s, 3);
end

function tf = should_forward_serial_command(code, config)
    tf = false;
    if strncmpi(code, 'M', 1)
        tf = config.forwardMCodesToXY;
    elseif strncmpi(code, 'G', 1)
        tf = config.forwardOtherCommandsToXY;
    end
end

function cmd = build_home_command(params)
    fields = intersect(fieldnames(params), {'X','Y'});
    if isempty(fields)
        cmd = 'G28';
        return;
    end
    tokens = {'G28'};
    for i = 1:numel(fields)
        tokens{end+1} = upper(fields{i}); %#ok<AGROW>
    end
    cmd = strjoin(tokens, ' ');
end

function cmd = build_g92_command(params)
    tokens = {'G92'};
    axesNames = {'X','Y'};
    for i = 1:numel(axesNames)
        axisName = axesNames{i};
        if isfield(params, axisName) && ~isempty(params.(axisName))
            tokens{end+1} = sprintf('%s%.3f', axisName, params.(axisName)); %#ok<AGROW>
        end
    end
    if numel(tokens) == 1
        cmd = '';
    else
        cmd = strjoin(tokens, ' ');
    end
end

function drain_serial(s, maxReads)
    if nargin < 2
        maxReads = 1;
    end
    for i = 1:maxReads
        if s.NumBytesAvailable <= 0
            return;
        end
        try %#ok<TRYNC>
            readline(s);
        end
    end
end

function logData = trim_log(logData, moveIndex)
    fields = {'timestamp','ref_log','r_log','e_log','e_cmd','material_delta','power_log','xy_status','z_status'};
    for i = 1:numel(fields)
        fieldName = fields{i};
        value = logData.(fieldName);
        logData.(fieldName) = value(1:moveIndex,:);
    end
end

function report = build_report(traj, logData, totalXYDistance, totalZDistance, totalMaterial, elapsedTime, aborted, finalPose)
    report = struct();
    report.source_file = traj.sourceFile;
    report.command_count = numel(traj.sequence);
    report.waypoint_count = size(traj.waypoints, 1);
    report.total_time_elapsed = elapsedTime;
    report.distance_traveled_xy = totalXYDistance;
    report.distance_traveled_z = totalZDistance;
    report.material_used = totalMaterial;
    report.aborted = aborted;
    report.telemetry = logData;

    if isempty(logData.e_log)
        report.rmse_x = NaN;
        report.rmse_y = NaN;
        report.rmse_z = NaN;
        report.rmse_3d = NaN;
        report.rmse_xyz = [NaN, NaN, NaN];
        report.final_pose_actual = [NaN, NaN, NaN];
        report.final_pose_expected = [NaN, NaN, NaN];
        report.final_pose_error = [NaN, NaN, NaN];
        report.final_pose = finalPose;
        return;
    end

    report.rmse_x = sqrt(mean(logData.e_log(:,1).^2, 'omitnan'));
    report.rmse_y = sqrt(mean(logData.e_log(:,2).^2, 'omitnan'));
    report.rmse_z = sqrt(mean(logData.e_log(:,3).^2, 'omitnan'));
    report.rmse_3d = sqrt(mean(sum(logData.e_log.^2, 2), 'omitnan'));
    report.rmse_xyz = [report.rmse_x, report.rmse_y, report.rmse_z];
    report.final_pose_actual = logData.r_log(end,:);
    report.final_pose_expected = logData.ref_log(end,:);
    report.final_pose_error = logData.e_log(end,:);
    if isempty(finalPose) || all(isnan(finalPose))
        report.final_pose = [report.final_pose_actual, NaN, NaN, NaN];
    else
        report.final_pose = finalPose;
    end
end

function plot_telemetry(logData, report)
    valid = all(~isnan(logData.r_log), 2) & all(~isnan(logData.ref_log), 2);
    if ~any(valid)
        return;
    end

    t = logData.timestamp(valid);
    ref = logData.ref_log(valid,:);
    act = logData.r_log(valid,:);
    err = logData.e_log(valid,:);

    figure('Name', 'G-code trajectory 3D', 'Color', 'w');
    plot3(ref(:,1), ref(:,2), ref(:,3), '.-', 'LineWidth', 1.1); hold on;
    plot3(act(:,1), act(:,2), act(:,3), '.-', 'LineWidth', 1.1);
    grid on; axis equal;
    xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
    title(sprintf('Desired vs actual trajectory (RMSE_{3D}=%.2f mm)', report.rmse_3d));
    legend('Desired', 'Actual', 'Location', 'best');

    figure('Name', 'G-code axis errors', 'Color', 'w');
    plot(t, err(:,1), 'LineWidth', 1.1); hold on;
    plot(t, err(:,2), 'LineWidth', 1.1);
    plot(t, err(:,3), 'LineWidth', 1.1);
    grid on;
    xlabel('Time [s]'); ylabel('Error [mm]');
    legend('e_x', 'e_y', 'e_z', 'Location', 'best');
    title('Axis tracking errors');
end

function config = apply_defaults(config)
    defaults = struct(...
        'dryRun', false, ...
        'homeOnStart', false, ...
        'homeMyCobotOnG28', true, ...
        'defaultXYFeedrate', 3000, ...
        'enableKeyboardControl', true, ...
        'plotTelemetry', true, ...
        'forwardMCodesToXY', false, ...
        'forwardOtherCommandsToXY', false, ...
        'xyTolerance', 1.0, ...
        'xyPollPeriod', 0.05, ...
        'xyTimeout', 20.0, ...
        'xyStallTolerance', 0.05, ...
        'xyStallSamples', 20, ...
        'Xmin', -280, ...
        'Xmax', 280, ...
        'Ymin', -280, ...
        'Ymax', 280, ...
        'Rsafe', 260, ...
        'robotZMin', 80, ...
        'robotZMax', 320, ...
        'myCobotSpeed', 50, ...
        'myCobotTimeout', 20.0, ...
        'myCobotHomePosition', [0, -80, 120, 60, 30, 0], ...
        'robotPoseReference', [0, 0, 120, 0, 0, 0], ...
        'xyScale', [1, 1], ...
        'xyOffset', [0, 0], ...
        'zScale', 1, ...
        'zOffset', 0, ...
        'transformFcn', []);

    defaultFields = fieldnames(defaults);
    for i = 1:numel(defaultFields)
        fieldName = defaultFields{i};
        if ~isfield(config, fieldName) || isempty(config.(fieldName))
            config.(fieldName) = defaults.(fieldName);
        end
    end
end
