function v = pylist_to_double(pyobj)
%PYLIST_TO_DOUBLE Convert a simple Python list or list-of-lists to double.

    C = cell(pyobj);
    if isempty(C)
        v = [];
        return;
    end
    if all(cellfun(@(x) ~iscell(x), C))
        v = cellfun(@double, C, 'UniformOutput', true);
    else
        n = numel(C);
        m = numel(cell(C{1}));
        v = zeros(n, m);
        for i = 1:n
            v(i, :) = cellfun(@double, cell(C{i}), 'UniformOutput', true);
        end
    end
end
