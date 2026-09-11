function varargout = exec_gcode(varargin)
%EXEC_GCODE Convenience wrapper around JUNTO2_GCODE.
    [varargout{1:nargout}] = junto2_gcode(varargin{:});
end
