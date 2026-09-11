function [traj, meta] = gcode_parser(filePath)
%GCODE_PARSER Parse a G-code file into absolute waypoints and command data.
%   [TRAJ, META] = GCODE_PARSER(FILEPATH) parses motion commands (G0/G1),
%   state commands (G28/G90/G91/G92/M82/M83), M-codes, and slicer-style
%   metadata comments. Waypoints are returned as an N-by-5 matrix:
%   [X Y Z E Feedrate].
%
%   Additional fields:
%     traj.sequence       - ordered cell array of move/command structs
%     traj.commands       - non-move command structs
%     traj.command_count  - total parsed command count
%     traj.waypoint_count - total motion waypoint count
%     traj.meta           - metadata extracted from comments

    if nargin < 1 || isempty(filePath)
        error('gcode_parser:MissingPath', 'A G-code file path is required.');
    end
    if ~(ischar(filePath) || isstring(filePath))
        error('gcode_parser:InvalidPath', 'The G-code file path must be text.');
    end

    filePath = char(filePath);
    if ~isfile(filePath)
        error('gcode_parser:FileNotFound', 'G-code file not found: %s', filePath);
    end

    rawText = fileread(filePath);
    rawText = strrep(rawText, sprintf('\r\n'), sprintf('\n'));
    rawText = strrep(rawText, sprintf('\r'), sprintf('\n'));
    rawLines = regexp(rawText, '\n', 'split');

    logicalLines = collect_logical_lines(rawLines);

    state = struct(...
        'X', 0, 'Y', 0, 'Z', 0, 'E', 0, 'F', NaN, ...
        'motionAbsolute', true, ...
        'extrusionAbsolute', true, ...
        'extrusionModeExplicit', false, ...
        'unitScale', 1.0, ...
        'units', 'mm');

    waypoints = zeros(0,5);
    commands = cell(0,1);
    sequence = cell(0,1);
    meta = struct();
    meta.sourceFile = filePath;
    meta.comments = {};
    meta.units = 'mm';

    for i = 1:numel(logicalLines)
        entry = logicalLines{i};

        if ~isempty(entry.comment)
            meta.comments{end+1,1} = entry.comment; %#ok<AGROW>
            meta = update_meta_from_comment(meta, entry.comment);
        end

        if isempty(entry.code)
            continue;
        end

        segments = split_command_segments(entry.code);
        for j = 1:numel(segments)
            segment = strtrim(segments{j});
            if isempty(segment)
                continue;
            end

            [cmd, params] = parse_command_segment(segment);
            if isempty(cmd)
                continue;
            end

            cmd = upper(cmd);
            record = struct(...
                'type', 'command', ...
                'code', cmd, ...
                'params', params, ...
                'lineNumber', entry.lineNumber, ...
                'line_number', entry.lineNumber, ...
                'raw', segment, ...
                'text', segment, ...
                'comment', entry.comment);

            switch cmd
                case {'G0', 'G00', 'G1', 'G01'}
                    [state, target] = apply_motion(state, params);
                    record.type = 'move';
                    record.target = target;
                    record.position = target(1:4);
                    record.feedrate = target(5);
                    waypoints(end+1,:) = target; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'G20'
                    state.unitScale = 25.4;
                    state.units = 'inch';
                    meta.units = 'inch';
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'G21'
                    state.unitScale = 1.0;
                    state.units = 'mm';
                    meta.units = 'mm';
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'G90'
                    state.motionAbsolute = true;
                    if ~state.extrusionModeExplicit
                        state.extrusionAbsolute = true;
                    end
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'G91'
                    state.motionAbsolute = false;
                    if ~state.extrusionModeExplicit
                        state.extrusionAbsolute = false;
                    end
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'M82'
                    state.extrusionAbsolute = true;
                    state.extrusionModeExplicit = true;
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'M83'
                    state.extrusionAbsolute = false;
                    state.extrusionModeExplicit = true;
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'G92'
                    state = apply_set_position(state, params);
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                case 'G28'
                    state = apply_home(state, params);
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>

                otherwise
                    commands{end+1,1} = record; %#ok<AGROW>
                    sequence{end+1,1} = record; %#ok<AGROW>
            end
        end
    end

    traj = struct();
    traj.sourceFile = filePath;
    traj.source_file = filePath;
    traj.waypoints = waypoints;
    traj.commands = commands;
    traj.meta = meta;
    traj.sequence = sequence;
    traj.command_count = numel(sequence);
    traj.waypoint_count = size(waypoints, 1);
    traj.final_state = state;
    traj.positioningMode = ternary(state.motionAbsolute, 'absolute', 'relative');
    traj.extrusionMode = ternary(state.extrusionAbsolute, 'absolute', 'relative');
end

function logicalLines = collect_logical_lines(rawLines)
    logicalLines = cell(0,1);
    pendingCode = '';
    pendingComment = '';
    pendingLine = 0;

    for i = 1:numel(rawLines)
        line = strtrim(rawLines{i});
        if isempty(line)
            flush_pending();
            continue;
        end

        [code, comment] = split_inline_comment(line);
        code = strtrim(code);
        comment = strtrim(comment);

        if isempty(code)
            flush_pending();
            logicalLines{end+1,1} = struct('code', '', 'comment', comment, 'lineNumber', i); %#ok<AGROW>
            continue;
        end

        if is_continuation_line(code) && ~isempty(pendingCode)
            pendingCode = strtrim([pendingCode, ' ', code]);
            if ~isempty(comment)
                pendingComment = strtrim(join_comments(pendingComment, comment));
            end
        else
            flush_pending();
            pendingCode = code;
            pendingComment = comment;
            pendingLine = i;
        end
    end

    flush_pending();

    function flush_pending()
        if ~isempty(pendingCode) || ~isempty(pendingComment)
            logicalLines{end+1,1} = struct( ...
                'code', strtrim(pendingCode), ...
                'comment', strtrim(pendingComment), ...
                'lineNumber', pendingLine); %#ok<AGROW>
        end
        pendingCode = '';
        pendingComment = '';
        pendingLine = 0;
    end
end

function [code, comment] = split_inline_comment(line)
    idx = find(line == ';', 1, 'first');
    if isempty(idx)
        code = line;
        comment = '';
    else
        code = line(1:idx-1);
        comment = line(idx+1:end);
    end
end

function tf = is_continuation_line(code)
    tf = isempty(regexp(code, '^\s*[GMTgmt]\s*[-+]?\d+', 'once'));
end

function out = join_comments(a, b)
    if isempty(a)
        out = b;
    elseif isempty(b)
        out = a;
    else
        out = [a, ' | ', b];
    end
end

function segments = split_command_segments(code)
    starts = regexp(code, '[GMTgmt]\s*[-+]?\d+', 'start');
    if isempty(starts)
        segments = {code};
        return;
    end

    segments = cell(numel(starts), 1);
    for k = 1:numel(starts)
        stopIdx = length(code);
        if k < numel(starts)
            stopIdx = starts(k+1) - 1;
        end
        segments{k} = strtrim(code(starts(k):stopIdx));
    end
end

function [cmd, params] = parse_command_segment(segment)
    token = regexp(segment, '^\s*([GMTgmt])\s*([-+]?\d+)', 'tokens', 'once');
    if isempty(token)
        cmd = '';
        params = struct();
        return;
    end

    cmd = [upper(token{1}), token{2}];
    remainder = segment(regexp(segment, '^\s*[GMTgmt]\s*[-+]?\d+', 'end', 'once') + 1:end);
    pairs = regexp(remainder, '([A-Za-z])\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))?', 'tokens');

    params = struct();
    for i = 1:numel(pairs)
        key = upper(pairs{i}{1});
        value = [];
        if numel(pairs{i}) >= 2 && ~isempty(pairs{i}{2})
            value = str2double(pairs{i}{2});
            if isnan(value)
                value = [];
            end
        end
        params.(key) = value;
    end
end

function [state, target] = apply_motion(state, params)
    axesNames = {'X','Y','Z'};
    for i = 1:numel(axesNames)
        axisName = axesNames{i};
        if isfield(params, axisName) && ~isempty(params.(axisName))
            value = state.unitScale * params.(axisName);
            if state.motionAbsolute
                state.(axisName) = value;
            else
                state.(axisName) = state.(axisName) + value;
            end
        end
    end

    if isfield(params, 'E') && ~isempty(params.E)
        value = state.unitScale * params.E;
        if state.extrusionAbsolute
            state.E = value;
        else
            state.E = state.E + value;
        end
    end

    if isfield(params, 'F') && ~isempty(params.F)
        state.F = state.unitScale * params.F;
    end

    target = [state.X, state.Y, state.Z, state.E, state.F];
end

function state = apply_set_position(state, params)
    fields = fieldnames(params);
    for i = 1:numel(fields)
        axisName = upper(fields{i});
        if isempty(params.(fields{i}))
            continue;
        end
        if ismember(axisName, {'X','Y','Z','E'})
            state.(axisName) = state.unitScale * params.(axisName);
        elseif strcmp(axisName, 'F')
            state.F = state.unitScale * params.(axisName);
        end
    end
end

function state = apply_home(state, params)
    homeAxes = {'X','Y','Z'};
    specifiedAxes = intersect(homeAxes, fieldnames(params));
    if isempty(specifiedAxes)
        specifiedAxes = homeAxes;
    end

    for i = 1:numel(specifiedAxes)
        state.(specifiedAxes{i}) = 0;
    end
end

function meta = update_meta_from_comment(meta, comment)
    token = regexp(comment, '^\s*([^:=]+?)\s*[:=]\s*(.+?)\s*$', 'tokens', 'once');
    if isempty(token)
        return;
    end

    fieldName = make_valid_field_name(strtrim(lower(token{1})));
    rawValue = strtrim(token{2});
    value = parse_comment_value(rawValue);
    meta.(fieldName) = value;
end

function value = parse_comment_value(rawValue)
    value = str2double(rawValue);
    if ~isnan(value)
        return;
    end

    numberToken = regexp(rawValue, '[-+]?(?:\d+(?:\.\d*)?|\.\d+)', 'match', 'once');
    if ~isempty(numberToken)
        numberValue = str2double(numberToken);
        if ~isnan(numberValue)
            value = numberValue;
            return;
        end
    end

    value = rawValue;
end

function fieldName = make_valid_field_name(rawName)
    try
        fieldName = matlab.lang.makeValidName(rawName);
    catch
        fieldName = lower(strtrim(rawName));
        fieldName = regexprep(fieldName, '[^a-zA-Z0-9_]', '_');
        fieldName = regexprep(fieldName, '_+', '_');
        if isempty(fieldName)
            fieldName = 'comment_value';
        end
        if ~isempty(regexp(fieldName(1), '[0-9]', 'once'))
            fieldName = ['x_', fieldName];
        end
    end
end

function out = ternary(condition, trueValue, falseValue)
    if condition
        out = trueValue;
    else
        out = falseValue;
    end
end
