function test_gcode_workflow()
%TEST_GCODE_WORKFLOW Focused parser/executor checks for the G-code workflow.

    workDir = tempname;
    mkdir(workDir);
    cleaner = onCleanup(@() cleanup_temp_dir(workDir)); %#ok<NASGU>

    gcodeFile = fullfile(workDir, 'sample.gcode');
    fid = fopen(gcodeFile, 'w');
    fprintf(fid, '; layer_height: 0.40\n');
    fprintf(fid, '; tool_diameter = 0.25\n');
    fprintf(fid, 'G28\n');
    fprintf(fid, 'G92 X0 Y0 Z0 E0\n');
    fprintf(fid, 'G90\n');
    fprintf(fid, 'G1 X10 Y10 Z5 F1000 E2\n');
    fprintf(fid, 'G1 X20\n');
    fprintf(fid, ' Y15 E3\n');
    fprintf(fid, 'G91\n');
    fprintf(fid, 'G1 X5 Z2 E0.5\n');
    fprintf(fid, 'G90\n');
    fclose(fid);

    [traj, meta] = gcode_parser(gcodeFile);
    assert(size(traj.waypoints, 1) == 3, 'Expected 3 parsed waypoints.');
    assert(all(abs(traj.waypoints(1,:) - [10 10 5 2 1000]) < 1e-9), 'First waypoint mismatch.');
    assert(all(abs(traj.waypoints(2,:) - [20 15 5 3 1000]) < 1e-9), 'Continuation waypoint mismatch.');
    assert(all(abs(traj.waypoints(3,:) - [25 15 7 3.5 1000]) < 1e-9), 'Relative waypoint mismatch.');
    assert(isfield(meta, 'layer_height') && abs(meta.layer_height - 0.4) < 1e-9, 'Metadata layer height missing.');
    assert(isfield(meta, 'tool_diameter') && abs(meta.tool_diameter - 0.25) < 1e-9, 'Metadata tool diameter missing.');

    cfg = struct('dryRun', true, 'plotTelemetry', false, 'enableKeyboardControl', false, 'homeOnStart', false, 'robotZMin', 0);
    [logData, report] = gcode_executor(traj, [], [], cfg);
    assert(size(logData.r_log, 1) == 3, 'Dry-run execution log length mismatch.');
    assert(all(abs(logData.r_log(end,:) - [25 15 7]) < 1e-9), 'Dry-run final pose mismatch.');
    assert(abs(report.material_used - 3.5) < 1e-9, 'Dry-run material accumulation mismatch.');
    assert(abs(report.distance_traveled_xy - (hypot(10,10) + hypot(10,5) + 5)) < 1e-9, 'XY distance mismatch.');
    assert(abs(report.distance_traveled_z - 7) < 1e-9, 'Z distance mismatch.');
end

function cleanup_temp_dir(workDir)
    if exist(workDir, 'dir')
        rmdir(workDir, 's');
    end
end
