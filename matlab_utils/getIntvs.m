function intvs = getIntvs(arr)
% Get intervals from indices
    if ~isempty(arr)
        intvs = [arr(1), arr(1)];
        for i=2:length(arr)
            if arr(i) - arr(i-1) > 1
                intvs(end,2) = arr(i-1);
                intvs = [intvs; arr(i), arr(i)];
            end
        end
        intvs(end,2) = arr(end);
    else
        intvs = [];
    end
end