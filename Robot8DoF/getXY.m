function [XR, YR, ok, src] = getXY(s)
%GETXY Read the current XY position from Marlin, GRBL, or Klipper.

    XR = NaN; YR = NaN; ok = false; src = 'NONE';

    function txt = readBurst(timeout_s)
        if nargin < 1, timeout_s = 0.25; end
        raw = strings(0);
        t0 = tic;
        while toc(t0) < timeout_s
            if s.NumBytesAvailable > 0
                try
                    raw(end+1) = readline(s); %#ok<AGROW>
                catch
                    break;
                end
            else
                pause(0.01);
            end
        end
        txt = strjoin(raw, ' ');
    end

    try
        flush(s);
        writeline(s, 'M114');
        pause(0.03);
        txt = readBurst(0.25);
        rx = regexp(txt, 'X\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        ry = regexp(txt, 'Y\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rx) && ~isempty(ry)
            XR = str2double(rx{1});
            YR = str2double(ry{1});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'M114'; return; end
        end
    catch
    end

    try
        flush(s);
        write(s, '?', 'char');
        pause(0.05);
        txt = readBurst(0.25);
        rW = regexp(txt, 'WPos\s*:\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rW)
            XR = str2double(rW{1});
            YR = str2double(rW{2});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'GRBL:WPos'; return; end
        end
        rM = regexp(txt, 'MPos\s*:\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rM)
            XR = str2double(rM{1});
            YR = str2double(rM{2});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'GRBL:MPos'; return; end
        end
    catch
    end

    try
        flush(s);
        writeline(s, 'GET_POSITION');
        pause(0.05);
        txt = readBurst(0.25);
        rx = regexp(txt, 'x\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        ry = regexp(txt, 'y\s*:\s*(-?\d+\.?\d*)', 'tokens', 'once');
        if ~isempty(rx) && ~isempty(ry)
            XR = str2double(rx{1});
            YR = str2double(ry{1});
            ok = ~(isnan(XR) || isnan(YR));
            if ok, src = 'KLIPPER'; return; end
        end
    catch
    end
end
