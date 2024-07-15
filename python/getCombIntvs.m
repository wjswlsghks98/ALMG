function possi_comb = getCombIntvs(comb)
    possi_comb = [];
    for i = 1:length(comb)
        seg = comb{i};
        lb = seg(1,1); ub = seg(end,end);
        seg_shape = size(seg);
        for j = 1:seg_shape(1)
            row = seg(j,:);
            if seg_shape(2) ~= 1
                for k = 2:seg_shape(2)
                    if row(k-1) < row(k)
                        possi_comb = [possi_comb;row(k-1),row(k)-1,j,i];
                        if row(k) == ub
                            possi_comb = [possi_comb;row(k),row(k),j,i];
                            break
                        end
                    else
                        possi_comb = [possi_comb;row(k-1),ub,j,i];
                        break
                    end
                end
            else
                possi_comb = [possi_comb;lb,ub,j,i];
            end
        end
    end     
end