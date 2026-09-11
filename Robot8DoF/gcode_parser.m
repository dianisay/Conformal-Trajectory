function program = gcode_parser(filePath)
%GCODE_PARSER Parse a standard G-code file into structured trajectory data.
%   PROGRAM = GCODE_PARSER(FILEPATH) parses common G-code commands and
%   returns a struct with per-line command metadata and resolved waypoints
%   in [X Y Z E F] format.

    if ~(ischar(filePath) || (isstring(filePath) && isscalar(filePath)))
        error('gcode_parser:InvalidInput', 'FILEPATH must be a character vector or scalar string.');
    end
    filePath = char(filePath);
    if exist(filePath, 'file') ~= 2
        error('gcode_parser:FileNotFound', 'G-code file not found: %s', filePath);
    end

    fid = fopen(filePath, 'r');
    if fid < 0
        error('gcode_parser:OpenFailed', 'Unable to open G-code file: %s', filePath);
    end
    cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

    state.absoluteMotion = true;
    state.absoluteExtrusion = true;
    state.position = [0 0 0 0];   % [X Y Z E]
    state.feedrate = NaN;

    commands = repmat(emptyCommand(), 0, 1);
    waypoints = zeros(0, 5);

    lineNumber = 0;
    while true
        rawLine = fgetl(fid);
        if ~ischar(rawLine)
            break;
        end
        lineNumber = lineNumber + 1;

        [cleanLine, commentText] = stripComments(rawLine);
        if strlength(string(strtrim(cleanLine))) == 0
            continue;
        end

        words = regexp(upper(strtrim(cleanLine)), '[A-Z][-+]?\d*\.?\d*(?:[Ee][-+]?\d+)?', 'match');
        if isempty(words)
            continue;
        end

        cmd = emptyCommand();
        cmd.line_number = lineNumber;
        cmd.raw = string(rawLine);
        cmd.text = string(strtrim(cleanLine));
        cmd.comment = string(commentText);
        cmd.has_x = false; cmd.has_y = false; cmd.has_z = false;
        cmd.has_e = false; cmd.has_f = false; cmd.has_s = false;
        cmd.position = state.position;
        cmd.feedrate = state.feedrate;
        cmd.set_position = nan(1,4);

        for idx = 1:numel(words)
            word = words{idx};
            letter = word(1);
            valueText = strtrim(word(2:end));
            if isempty(valueText)
                value = NaN;
            else
                value = str2double(valueText);
            end

            switch letter
                case {'G','M','T'}
                    if strlength(cmd.code) == 0 && ~isnan(value)
                        cmd.code = sprintf('%c%d', letter, round(value));
                    end
                case 'X'
                    cmd.has_x = true; cmd.set_position(1) = value;
                case 'Y'
                    cmd.has_y = true; cmd.set_position(2) = value;
                case 'Z'
                    cmd.has_z = true; cmd.set_position(3) = value;
                case 'E'
                    cmd.has_e = true; cmd.set_position(4) = value;
                case 'F'
                    cmd.has_f = true; cmd.feedrate = value;
                case 'S'
                    cmd.has_s = true; cmd.temperature = value;
                case 'R'
                    cmd.extra_r = value;
                otherwise
                    cmd.params.(letter) = value;
            end
        end

        if strlength(cmd.code) == 0
            cmd.code = string(words{1});
        end

        code = char(cmd.code);
        switch code
            case 'G90'
                state.absoluteMotion = true;
                cmd.type = "motion_mode";
            case 'G91'
                state.absoluteMotion = false;
                cmd.type = "motion_mode";
            case 'M82'
                state.absoluteExtrusion = true;
                cmd.type = "extrusion_mode";
            case 'M83'
                state.absoluteExtrusion = false;
                cmd.type = "extrusion_mode";
            case {'G0','G1'}
                cmd.type = "motion";
                [state, cmd] = resolveMotion(state, cmd);
                waypoints(end+1, :) = [cmd.position, cmd.feedrate]; %#ok<AGROW>
            case 'G28'
                cmd.type = "home";
                cmd.home_axes = homeAxes(cmd);
                if strlength(cmd.home_axes) == 0
                    state.position(1:3) = 0;
                else
                    if contains(cmd.home_axes, 'X'), state.position(1) = 0; end
                    if contains(cmd.home_axes, 'Y'), state.position(2) = 0; end
                    if contains(cmd.home_axes, 'Z'), state.position(3) = 0; end
                end
                cmd.position = state.position;
                cmd.feedrate = state.feedrate;
            case 'G92'
                cmd.type = "set_position";
                axesMask = [cmd.has_x, cmd.has_y, cmd.has_z, cmd.has_e];
                vals = cmd.set_position;
                state.position(axesMask) = vals(axesMask);
                cmd.position = state.position;
            case 'M104'
                cmd.type = "set_temperature";
            case 'M109'
                cmd.type = "wait_temperature";
                cmd.wait_for_temperature = true;
            case {'M0','M1'}
                cmd.type = "pause";
            otherwise
                cmd.type = "raw";
            end

        cmd.motion_absolute = state.absoluteMotion;
        cmd.extrusion_absolute = state.absoluteExtrusion;
        commands(end+1, 1) = cmd; %#ok<AGROW>
    end

    program = struct();
    program.source_file = string(filePath);
    program.command_count = numel(commands);
    program.waypoint_count = size(waypoints, 1);
    program.waypoints = waypoints;
    program.commands = commands;
    program.final_state = state;
end

function cmd = emptyCommand()
    cmd = struct( ...
        'line_number', 0, ...
        'raw', "", ...
        'text', "", ...
        'comment', "", ...
        'code', "", ...
        'type', "", ...
        'params', struct(), ...
        'position', nan(1,4), ...
        'feedrate', NaN, ...
        'temperature', NaN, ...
        'wait_for_temperature', false, ...
        'set_position', nan(1,4), ...
        'home_axes', "", ...
        'motion_absolute', true, ...
        'extrusion_absolute', true, ...
        'extrusion_delta', 0, ...
        'has_x', false, ...
        'has_y', false, ...
        'has_z', false, ...
        'has_e', false, ...
        'has_f', false, ...
        'has_s', false, ...
        'extra_r', NaN);
end

function [state, cmd] = resolveMotion(state, cmd)
    target = state.position;
    axisMask = [cmd.has_x, cmd.has_y, cmd.has_z];
    newVals = cmd.set_position(1:3);
    if any(axisMask)
        if state.absoluteMotion
            target(axisMask) = newVals(axisMask);
        else
            target(axisMask) = target(axisMask) + newVals(axisMask);
        end
    end

    if cmd.has_e
        eVal = cmd.set_position(4);
        if state.absoluteExtrusion
            target(4) = eVal;
            cmd.extrusion_delta = eVal - state.position(4);
        else
            target(4) = target(4) + eVal;
            cmd.extrusion_delta = eVal;
        end
    else
        cmd.extrusion_delta = 0;
    end

    if cmd.has_f
        state.feedrate = cmd.feedrate;
    else
        cmd.feedrate = state.feedrate;
    end

    state.position = target;
    cmd.position = target;
end

function axesText = homeAxes(cmd)
    names = '';
    if cmd.has_x, names = [names 'X']; end %#ok<AGROW>
    if cmd.has_y, names = [names 'Y']; end %#ok<AGROW>
    if cmd.has_z, names = [names 'Z']; end %#ok<AGROW>
    axesText = string(names);
end

function [cleanLine, commentText] = stripComments(rawLine)
    commentParts = strings(0,1);

    parenTokens = regexp(rawLine, '\(([^\)]*)\)', 'tokens');
    if ~isempty(parenTokens)
        for i = 1:numel(parenTokens)
            commentParts(end+1,1) = string(parenTokens{i}{1}); %#ok<AGROW>
        end
    end
    noParen = regexprep(rawLine, '\([^\)]*\)', ' ');

    semicolonIdx = strfind(noParen, ';');
    if isempty(semicolonIdx)
        cleanLine = noParen;
    else
        cleanLine = noParen(1:semicolonIdx(1)-1);
        semicolonComment = strtrim(noParen(semicolonIdx(1)+1:end));
        if ~isempty(semicolonComment)
            commentParts(end+1,1) = string(semicolonComment); %#ok<AGROW>
        end
    end

    commentText = strjoin(commentParts, ' | ');
end
