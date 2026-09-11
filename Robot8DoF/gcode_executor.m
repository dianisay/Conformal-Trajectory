function report = gcode_executor(program, s, mc, options)
%GCODE_EXECUTOR Execute parsed G-code with the XY platform and MyCobot arm.
%   REPORT = GCODE_EXECUTOR(PROGRAM, S, MC, OPTIONS) accepts the output of
%   GCODE_PARSER (or a G-code file path), an XY serialport handle S, a
%   loaded mc_bridge Python module MC, and execution options.

    if nargin < 4 || isempty(options)
        options = struct();
    end
    options = normalizeOptions(options);

    if ischar(program) || isstring(program)
        program = gcode_parser(program);
    end

    if ~isstruct(program) || ~isfield(program, 'commands')
        error('gcode_executor:InvalidProgram', 'PROGRAM must be a parsed G-code struct or file path.');
    end

    commands = program.commands;
    nCommands = numel(commands);
    telemetry = repmat(emptyTelemetry(), nCommands, 1);
    materialUsed = 0;

    if ~isempty(s)
        writeline(s, 'G90');
        pause(options.xy_command_delay_s);
    end

    if ~isempty(mc)
        try
            mc.set_move_mode(int32(options.mycobot_move_mode));
        catch ME
            warning('gcode_executor:MoveMode', 'Unable to set MyCobot move mode: %s', ME.message);
        end
    end

    t0 = tic;
    for i = 1:nCommands
        cmd = commands(i);
        waitIfPaused(options, s);

        entry = emptyTelemetry();
        entry.index = i;
        entry.line_number = cmd.line_number;
        entry.command = cmd.code;
        entry.command_type = cmd.type;
        entry.timestamp_s = toc(t0);
        entry.desired = [cmd.position, cmd.feedrate];
        entry.extrusion_delta = cmd.extrusion_delta;
        entry.status = "skipped";

        try
            switch char(cmd.type)
                case 'motion'
                    executeMotion(cmd, s, mc, options);
                    materialUsed = materialUsed + max(0, cmd.extrusion_delta);
                    if cmd.has_e
                        handleExtrusion(cmd.extrusion_delta, cmd, options);
                    end
                    entry.status = "ok";
                case 'home'
                    executeHome(cmd, s, mc, options);
                    entry.status = "ok";
                case 'set_position'
                    if ~isempty(s)
                        writeline(s, buildG92(cmd));
                        pause(options.xy_command_delay_s);
                    end
                    entry.status = "ok";
                case {'set_temperature','wait_temperature','raw','pause'}
                    if ~isempty(s)
                        writeline(s, char(cmd.text));
                        pause(options.xy_command_delay_s);
                    end
                    entry.status = "ok";
                otherwise
                    entry.status = "ignored";
            end
        catch ME
            entry.status = "error";
            entry.note = string(ME.message);
            telemetry(i) = updateTelemetry(entry, s, mc);
            rethrow(ME);
        end

        telemetry(i) = updateTelemetry(entry, s, mc);
        fprintf('[%d/%d] %s %s\n', i, nCommands, char(cmd.code), char(telemetry(i).status));
    end

    finalPose = nan(1, 6);
    if ~isempty(mc)
        finalPose = readMyCobotPose(mc);
    end
    [rmseXYZ, axisCount] = computeRmse(telemetry);

    report = struct();
    report.source_file = program.source_file;
    report.command_count = nCommands;
    report.waypoint_count = program.waypoint_count;
    report.material_used = materialUsed;
    report.rmse_xyz = rmseXYZ;
    report.rmse_axis_count = axisCount;
    report.final_pose = finalPose;
    report.telemetry = telemetry;
    report.completed_at = datetime('now');

    fprintf('\n=== G-code execution report ===\n');
    fprintf('Commands executed: %d\n', nCommands);
    fprintf('Waypoints executed: %d\n', program.waypoint_count);
    fprintf('Material used (E delta): %.3f\n', materialUsed);
    fprintf('RMSE [X Y Z]: [%.3f %.3f %.3f]\n', rmseXYZ);
    fprintf('Final MyCobot pose: [%.3f %.3f %.3f %.3f %.3f %.3f]\n', finalPose);
end

function options = normalizeOptions(options)
    defaults = struct( ...
        'xy_command_delay_s', 0.05, ...
        'xy_settle_time_s', 0.10, ...
        'mycobot_move_mode', 1, ...
        'mycobot_speed', 60, ...
        'mycobot_wait', true, ...
        'mycobot_timeout_s', 20, ...
        'couple_xy_to_mycobot', false, ...
        'mycobot_orientation', [NaN NaN NaN], ...
        'mycobot_offset', [0 0 0], ...
        'home_mycobot', false, ...
        'mycobot_home_joints', [0 -80 120 60 30 0], ...
        'pause_file', "", ...
        'pause_callback', [], ...
        'extruder_callback', []);

    names = fieldnames(defaults);
    for i = 1:numel(names)
        name = names{i};
        if ~isfield(options, name) || isempty(options.(name))
            options.(name) = defaults.(name);
        end
    end
end

function executeMotion(cmd, s, mc, options)
    if ~isempty(s) && (cmd.has_x || cmd.has_y || cmd.has_f)
        writeline(s, buildXYMove(cmd));
        pause(options.xy_settle_time_s);
    end

    if ~isempty(mc) && cmd.has_z
        coords = readMyCobotPose(mc);
        target = coords;
        if any(isnan(target))
            target = [0 0 0 0 0 0];
        end
        if options.couple_xy_to_mycobot
            if cmd.has_x, target(1) = cmd.position(1) + options.mycobot_offset(1); end
            if cmd.has_y, target(2) = cmd.position(2) + options.mycobot_offset(2); end
        end
        target(3) = cmd.position(3) + options.mycobot_offset(3);

        ori = options.mycobot_orientation;
        for k = 1:3
            if ~isnan(ori(k))
                target(3 + k) = ori(k);
            end
        end

        mc.move_cartesian(target(1), target(2), target(3), target(4), target(5), target(6), ...
            int32(options.mycobot_speed), logical(options.mycobot_wait), double(options.mycobot_timeout_s));
    end
end

function executeHome(cmd, s, mc, options)
    axes = char(cmd.home_axes);
    if ~isempty(s)
        if isempty(axes)
            writeline(s, 'G28');
        else
            writeline(s, sprintf('G28 %s', axes));
        end
        pause(options.xy_settle_time_s);
    end

    if ~isempty(mc) && options.home_mycobot
        mc.home(py.list(num2cell(options.mycobot_home_joints)));
    end
end

function line = buildXYMove(cmd)
    line = 'G1';
    if cmd.has_x
        line = sprintf('%s X%.3f', line, cmd.position(1));
    end
    if cmd.has_y
        line = sprintf('%s Y%.3f', line, cmd.position(2));
    end
    if cmd.has_f && ~isnan(cmd.feedrate)
        line = sprintf('%s F%.3f', line, cmd.feedrate);
    end
end

function line = buildG92(cmd)
    line = 'G92';
    if cmd.has_x
        line = sprintf('%s X%.3f', line, cmd.position(1));
    end
    if cmd.has_y
        line = sprintf('%s Y%.3f', line, cmd.position(2));
    end
    if cmd.has_z
        line = sprintf('%s Z%.3f', line, cmd.position(3));
    end
    if cmd.has_e
        line = sprintf('%s E%.3f', line, cmd.position(4));
    end
end

function handleExtrusion(extrusionDelta, cmd, options)
    if isa(options.extruder_callback, 'function_handle')
        options.extruder_callback(extrusionDelta, cmd);
    elseif extrusionDelta > 0
        fprintf('Extruder placeholder: deposit %.3f units at line %d\n', extrusionDelta, cmd.line_number);
    end
end

function telemetry = updateTelemetry(telemetry, s, mc)
    telemetry.actual_xy = [NaN NaN];
    telemetry.xy_source = "NONE";
    telemetry.actual_pose = nan(1, 6);

    if ~isempty(s)
        [xr, yr, ok, src] = getXY(s);
        if ok
            telemetry.actual_xy = [xr yr];
            telemetry.xy_source = string(src);
        end
    end

    if ~isempty(mc)
        telemetry.actual_pose = readMyCobotPose(mc);
    end
end

function pose = readMyCobotPose(mc)
    pose = nan(1, 6);
    try
        poseInfo = mc.get_pose();
        pose = pylist_to_double(poseInfo{'coords'});
    catch
    end
    if isempty(pose)
        pose = nan(1, 6);
    end
    pose = reshape(double(pose), 1, []);
    if numel(pose) < 6
        pose(6) = NaN;
    else
        pose = pose(1:6);
    end
end

function waitIfPaused(options, s)
    paused = shouldPause(options);
    while paused
        pause(0.2);
        paused = shouldPause(options);
    end
end

function tf = shouldPause(options)
    tf = false;
    if strlength(string(options.pause_file)) > 0 && exist(char(options.pause_file), 'file') == 2
        tf = true;
        return;
    end
    if isa(options.pause_callback, 'function_handle')
        tf = logical(options.pause_callback());
    end
end

function [rmseXYZ, axisCount] = computeRmse(telemetry)
    motionMask = arrayfun(@(t) strcmp(char(t.command_type), 'motion'), telemetry);
    telemetry = telemetry(motionMask);

    if isempty(telemetry)
        rmseXYZ = [NaN NaN NaN];
        axisCount = [0 0 0];
        return;
    end

    desired = vertcat(telemetry.desired);
    actual = nan(numel(telemetry), 3);
    for i = 1:numel(telemetry)
        actual(i, 1:2) = telemetry(i).actual_xy;
        actual(i, 3) = telemetry(i).actual_pose(3);
    end

    rmseXYZ = nan(1, 3);
    axisCount = zeros(1, 3);
    for axis = 1:3
        valid = ~isnan(desired(:, axis)) & ~isnan(actual(:, axis));
        axisCount(axis) = nnz(valid);
        if any(valid)
            rmseXYZ(axis) = sqrt(mean((desired(valid, axis) - actual(valid, axis)).^2));
        end
    end
end

function telemetry = emptyTelemetry()
    telemetry = struct( ...
        'index', 0, ...
        'line_number', 0, ...
        'command', "", ...
        'command_type', "", ...
        'timestamp_s', 0, ...
        'desired', nan(1, 5), ...
        'actual_xy', [NaN NaN], ...
        'xy_source', "NONE", ...
        'actual_pose', nan(1, 6), ...
        'extrusion_delta', 0, ...
        'status', "", ...
        'note', "");
end
