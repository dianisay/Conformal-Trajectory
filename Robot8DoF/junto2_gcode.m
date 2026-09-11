function [logData, report, traj] = junto2_gcode(gcodeFile, config)
%JUNTO2_GCODE Entry point for executing precomputed G-code trajectories.
%   [LOGDATA, REPORT, TRAJ] = JUNTO2_GCODE(GCODEFILE, CONFIG) loads the
%   existing Robot8DoF calibration assets, initializes the XY serial link
%   and MyCobot bridge, previews the parsed path, and executes it through
%   GCODE_EXECUTOR.

    if nargin < 1 || isempty(gcodeFile)
        defaultFile = fullfile(fileparts(mfilename('fullpath')), 'print_trajectory.gcode');
        if isfile(defaultFile)
            gcodeFile = defaultFile;
        else
            gcodeFile = input('Enter the absolute path to the G-code file: ', 's');
        end
    end
    if nargin < 2 || isempty(config)
        config = struct();
    end

    config = apply_entry_defaults(config);
    baseDir = fileparts(mfilename('fullpath'));
    config = load_calibration_assets(baseDir, config);

    traj = gcode_parser(gcodeFile);
    preview_trajectory(traj, config);

    if isfield(config, 'mode') && strcmpi(char(config.mode), 'vision')
        run(fullfile(baseDir, 'junto2.m'));
        logData = [];
        report = struct();
        return;
    end

    s = [];
    mc = [];

    if ~config.dryRun
        s = initialize_xy_serial(config);
        mc = initialize_mycobot(baseDir, config);
    end

    [logData, report] = gcode_executor(traj, s, mc, config);
    print_completion_report(gcodeFile, report);
end

function config = apply_entry_defaults(config)
    defaults = struct(...
        'dryRun', false, ...
        'mode', 'gcode', ...
        'xyPort', 'COM5', ...
        'xyBaud', 115200, ...
        'pythonExecutable', '', ...
        'homeOnStart', true, ...
        'homeMyCobotOnG28', true, ...
        'myCobotHomePosition', [0, -80, 120, 60, 30, 0], ...
        'defaultXYFeedrate', 3000, ...
        'myCobotSpeed', 50, ...
        'robotZMin', 80, ...
        'robotZMax', 320, ...
        'plotTelemetry', true, ...
        'enableKeyboardControl', true, ...
        'previewTrajectory', true, ...
        'loadCalibration', true, ...
        'xyScale', [1, 1], ...
        'xyOffset', [0, 0], ...
        'zScale', 1, ...
        'zOffset', 0, ...
        'transformFcn', []);

    fields = fieldnames(defaults);
    for i = 1:numel(fields)
        fieldName = fields{i};
        if ~isfield(config, fieldName) || isempty(config.(fieldName))
            config.(fieldName) = defaults.(fieldName);
        end
    end
end

function config = load_calibration_assets(baseDir, config)
    if ~config.loadCalibration
        return;
    end

    cameraFile = fullfile(baseDir, 'cameraParams.mat');
    if isfile(cameraFile)
        try
            config.cameraCalibration = load(cameraFile);
        catch ME
            warning('junto2_gcode:CameraCalibration', 'Could not load camera calibration: %s', ME.message);
        end
    end

    netFile = fullfile(baseDir, 'trainedNet.mat');
    if isfile(netFile)
        try
            config.trainedNet = load(netFile);
        catch ME
            warning('junto2_gcode:TrainedNet', 'Could not load trainedNet.mat: %s', ME.message);
        end
    end
end

function s = initialize_xy_serial(config)
    s = serialport(config.xyPort, config.xyBaud);
    configureTerminator(s, 'LF');
    flush(s);
    disp('XY serial connection opened.');
    writeline(s, 'G90');
    pause(0.05);
end

function mc = initialize_mycobot(baseDir, config)
    try
        pyPaths = cell(py.sys.path);
        hasBaseDir = any(cellfun(@(p) strcmp(char(p), baseDir), pyPaths));
    catch
        hasBaseDir = false;
    end
    if ~hasBaseDir
        insert(py.sys.path, int32(0), baseDir);
    end

    pe = pyenv;
    if pe.Status == "NotLoaded"
        if ~isempty(config.pythonExecutable)
            pyenv('Version', config.pythonExecutable, 'ExecutionMode', 'OutOfProcess');
        else
            pyenv('ExecutionMode', 'OutOfProcess');
        end
    elseif pe.Status == "Loaded" && pe.ExecutionMode == "InProcess"
        error('junto2_gcode:PythonMode', 'Python must run OutOfProcess for mc_bridge.');
    end

    mc = py.importlib.import_module('mc_bridge');
    py.importlib.reload(mc);
    disp('mc_bridge imported for G-code execution.');

    try
        mc.home(py.list(num2cell(config.myCobotHomePosition)));
    catch ME
        warning('junto2_gcode:MyCobotHome', 'MyCobot home failed: %s', ME.message);
    end
end

function preview_trajectory(traj, config)
    if ~config.previewTrajectory || isempty(traj.waypoints)
        return;
    end

    try
        figure('Name', 'G-code trajectory preview', 'Color', 'w');
        plot3(traj.waypoints(:,1), traj.waypoints(:,2), traj.waypoints(:,3), '.-', 'LineWidth', 1.1);
        grid on; axis equal;
        xlabel('X [mm]'); ylabel('Y [mm]'); zlabel('Z [mm]');
        title(sprintf('Preview: %d waypoints', size(traj.waypoints, 1)));
    catch ME
        warning('junto2_gcode:Preview', 'Could not create preview: %s', ME.message);
    end
end

function print_completion_report(gcodeFile, report)
    fprintf('\n=== G-code execution complete ===\n');
    fprintf('File: %s\n', gcodeFile);
    fprintf('Elapsed time: %.2f s\n', report.total_time_elapsed);
    fprintf('XY distance: %.2f mm\n', report.distance_traveled_xy);
    fprintf('Z distance: %.2f mm\n', report.distance_traveled_z);
    fprintf('Material used: %.2f\n', report.material_used);
    fprintf('RMSE: X=%.2f mm, Y=%.2f mm, Z=%.2f mm, 3D=%.2f mm\n', ...
        report.rmse_x, report.rmse_y, report.rmse_z, report.rmse_3d);
    fprintf('Final expected pose: [%.2f %.2f %.2f]\n', report.final_pose_expected);
    fprintf('Final actual pose:   [%.2f %.2f %.2f]\n', report.final_pose_actual);
    fprintf('Final pose error:    [%.2f %.2f %.2f]\n', report.final_pose_error);
    if report.aborted
        fprintf('Execution stopped before completion.\n');
    end
end
