function intvs = FitRoundabout(pts,cov)
    % Fit Arcs for roundabout state intervals
    [m,n] = size(cov);
    [m_,n_] = size(pts);

    % If covariance for only one point is given, replicate and reshape the covariance data.
    if m == 2 && n == 2
        cov = repmat(reshape(cov,4,1),1,n_);
    end

    % Ignore the z-coordinates if passed.
    if m_ == 3
        pts = pts(1:2,:);
    end
    % Arc spline approximation of roundabout interval.
    arc_fit = FitCircles(pts,cov);
    arc_fit.L_min = 10;
    arc_fit.optimize(true);
    intvs = arc_fit.params.intv;

    if size(intvs,1) > 3
        xyR = arc_fit.getParams();
        curv = arc_fit.getCurvatures();
        curv_idxs = 1:length(curv);
        % Filter out arc spline approximation intervals that are close to LKM 
        % First / Last arcs that are close to lines.
        curv_of_interest = curv(xyR(3,:) < 100); 
        curv_idx_of_interest = curv_idxs(xyR(3,:) < 100);

        if curv_of_interest(1) * curv_of_interest(end) < 0
            error('First and the last curvature of roundabout should have same signs. Check if arc spline approximation is accurate enough')
        else
            diff_idxs = [];
            for i=2:length(curv_of_interest)-1
                if curv_of_interest(1) * curv_of_interest(i) < 0
                    diff_idxs = [diff_idxs, i];    
                end
            end
            
            if isempty(diff_idxs)
                error('There must be curvature sign difference in roundabouts. Check if arc spline approximation is accurate enough')
            else
                sampled_intv = getIntvs(diff_idxs);
                if size(sampled_intv,1) > 1
                    error('Impossible curvature configuration in roundabout. Check if arc spline approximation is accurate enough')
                else
                    lb = sampled_intv(1); ub = sampled_intv(2);
                    arc_seg_lb = curv_idx_of_interest(lb);
                    arc_seg_ub = curv_idx_of_interest(ub);
                    % Concatenate pt_idxs from arc segment 1 ~ arc_seg_lb
                    %                                      arc_seg_lb ~ arc_seg_ub
                    %                                      arc_seg_ub ~ N
                    pt_idx1 = intvs(arc_seg_lb,1);
                    pt_idx2 = intvs(arc_seg_ub,2);
                    N = intvs(end,2);
                    intvs = [1, pt_idx1;
                             pt_idx1, pt_idx2;
                             pt_idx2, N];
                end
            end
        end

    elseif size(intvs,1) < 3
        error('Roundabout cannot be approximated with less than 3 arc segments. Check if arc spline approximation is accurate enough')
    end
    
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
end