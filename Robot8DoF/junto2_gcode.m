clearvars
clc

scriptDir = fileparts(mfilename('fullpath'));
mode = "gcode";     % "vision" -> run the original junto2.m

gcodeFile = fullfile(scriptDir, 'print_trajectory.gcode');
port = 'COM5';
baudRate = 115200;
pyExe = "C:\Users\MonDi\AppData\Local\Programs\Python\Python311\python.exe";
q_home = [0, -80, 120, 60, 30, 0];

if strcmpi(mode, "vision")
    run(fullfile(scriptDir, 'junto2.m'));
    return
end

fprintf('Loading G-code program: %s\n', gcodeFile);
program = gcode_parser(gcodeFile);
fprintf('Parsed %d commands and %d waypoints.\n', program.command_count, program.waypoint_count);

s = serialport(port, baudRate);
configureTerminator(s, "LF");
flush(s);
disp('Conexión serial abierta correctamente');

writeline(s, 'G90');
pause(0.1);
writeline(s, 'M83');   % keep XY firmware extrusion handling disabled unless needed
pause(0.1);

pe = pyenv;
if pe.Status == "Loaded" && pe.ExecutionMode == "OutOfProcess"
    terminate(pyenv);
elseif pe.Status == "Loaded" && pe.ExecutionMode == "InProcess"
    error('Python está InProcess. Reinicia MATLAB.');
end
pyenv('Version', pyExe, 'ExecutionMode', 'OutOfProcess');

mc = py.importlib.import_module('mc_bridge');
py.importlib.reload(mc);
disp('✅ mc_bridge importado.');
disp('→ HOME...');
disp(mc.home(py.list(num2cell(q_home))));

options = struct();
options.home_mycobot = false;
options.mycobot_home_joints = q_home;
options.mycobot_move_mode = 1;
options.mycobot_speed = 60;
options.mycobot_wait = true;
options.mycobot_timeout_s = 20;
options.pause_file = fullfile(scriptDir, 'gcode.pause');
options.extruder_callback = @(delta, cmd) fprintf('Extruder placeholder: EΔ=%.3f at line %d\n', delta, cmd.line_number);

report = gcode_executor(program, s, mc, options);

fprintf('\n=== Completion summary ===\n');
fprintf('RMSE [X Y Z]: [%.3f %.3f %.3f]\n', report.rmse_xyz);
fprintf('Final pose: [%.3f %.3f %.3f %.3f %.3f %.3f]\n', report.final_pose);
fprintf('Material used (E delta): %.3f\n', report.material_used);
