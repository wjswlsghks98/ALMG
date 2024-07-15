classdef FitCircles < handle
    %FitCircles module approximates a given point dataset using multiple
    % arcs, data point covariance data.
    % 
    % [Input arguments]
    % pts: Data points (2*n matrix)
    % cov: Covariance of data points (4*n matrix) --> each column will be reshaped into 2*2 covariance matrix.
    % * optional arguments
    % min_flag: Indicator to start approximation from single arc (Boolean)
    % upper_bound: Upper bound of the number of arc segments (Scalar)
    % 
    % [Usage examples]
    % 1. Default 
    % approx = FitCircles(pts,covs)
    % approx.optimize(true); % view approximation results step by step.
    % approx.visualize();
    % 
    % 2. MinFlag
    % approx = FitCircles(pts,covs,true)
    % approx.optimize(false);
    % approx.visualize();
    %
    % 3. MinFlag, UpperBound
    % approx = FitCircles(pts,covs,true,3)
    % approx.optimize(false);
    % approx.visualize();
    %
    % * Tips: Depending on the scale of the dataset, you may need to tune
    % some parameters inside this module: Recursive segmentation accuracy
    % threshold, Anchor model covariance, etc.
    %
    % [Info]
    % This is a slight modification of our previous work, 
    % "Reliability-based G1 Continuous Arc Spline Approximation" by 
    % Jinhwan Jeon, Yoonjin Hwang and Seibum Ben Choi* 
    %
    % Instead of middle node, we reduce the number of parameters and
    % constraints to optimize. This results in a faster and stable 
    % convergence towards an acceptable local minima.
    % 
    % Implemented by Jinhwan Jeon,2024

    properties
        pts % Data points
        cov % Data covariance
        params % Arc Parameters
        offset % Linear translation
        L_min % Minimum arc length
        static_methods % Class storing all static methods
        arc_num
    end

    methods (Access = public)
        %% Constructor
        function obj = FitCircles(varargin)
            warning('off','MATLAB:nearlySingularMatrix');
            obj.offset = mean(varargin{1},2);
            obj.pts = varargin{1} - obj.offset;
            obj.cov = varargin{2};
            
            min_flag = false;

            if length(varargin) == 3
                min_flag = varargin{3};  
                obj.arc_num = [1, inf];              
            elseif length(varargin) == 4
                min_flag = varargin{3};
                obj.arc_num = [1, varargin{4}]; % Bounds for number of arc segments : [lb, ub]
            else
                obj.arc_num = [-1, inf];
            end

            obj.params = struct();
            obj.L_min = 15;
            obj.static_methods = StaticMethods();
            obj.params.intv = [1,size(obj.pts,2)];
            obj.initialize(min_flag);
        end
        
        %% Optimize
        function obj = optimize(obj,disp_flag)
            maxInvalidN = 10;
            while true
                % if disp_flag
                %     disp(['Number of Arc Segments: ',num2str(size(obj.params.intv,1))])
                % end
                
                % obj.visualize(); drawnow;
                if size(obj.params.intv,1) > 1                    
                    x0 = [reshape(obj.params.arcNodes,2*(size(obj.params.intv,1)+1),1); reshape(obj.params.ks,size(obj.params.intv,1),1)];
                    lb = []; ub = []; A = []; b = []; Aeq = []; beq = [];
                    options = optimoptions('lsqnonlin','Algorithm','interior-point','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'Display','none');
                    % if disp_flag
                    %     options = optimoptions('lsqnonlin','Algorithm','interior-point','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'Display','iter-detailed');
                    % else
                    %     options = optimoptions('lsqnonlin','Algorithm','interior-point','SpecifyObjectiveGradient',true,'SpecifyConstraintGradient',true,'Display','none');
                    % end
                    
                    % Perform constrained optimization
                    x = lsqnonlin(@obj.cost_func,x0,lb,ub,A,b,Aeq,beq,@obj.nonlcon,options);

                    % Update optimized parameters
                    obj.params.arcNodes = reshape(x(1:2*size(obj.params.arcNodes,2)),2,size(obj.params.arcNodes,2));
                    obj.params.ks = reshape(x(2*size(obj.params.arcNodes,2)+1:end),1,size(obj.params.ks,2));
                end
                
                % Show current optimization results  
                % if disp_flag     
                %     obj.visualize();
                %     drawnow; hold off; % Turn this off if the solver is generally stable enough
                % end
                
                % Data Association
                obj.params.intv = obj.associate(obj.params.arcNodes);

                % Check validity and add new segments if needed
                [~,valid_flag,idx] = obj.checkValidity(obj.params.arcNodes,obj.params.ks,obj.params.intv,obj.pts,obj.cov,disp_flag,maxInvalidN);
                
                if length(obj.params.ks) == obj.arc_num(2) % If the number of arc segments is equal to the upper limit, finish.
                    break;
                else
                    if ~valid_flag
                        obj.updateParams(idx);
                    else
                        % if disp_flag
                        %     disp(['[Final Number of segments: ',num2str(size(obj.params.intv,1)),']'])
                        % end
                        break;
                    end
                end
            end

            % After optimization is finished, offset is added back to the
            % optimized parameters and data
            obj.pts = obj.pts + obj.offset;
            obj.params.arcNodes = obj.params.arcNodes + obj.offset;
            % obj.visualize();
        end
        
        %% Visualize
        function visualize(obj)
            figure(1);
            p_data = plot(obj.pts(1,:),obj.pts(2,:),'r.'); hold on; axis equal;
            p_arcN = plot(obj.params.arcNodes(1,:),obj.params.arcNodes(2,:),'bo');
            arc_length = 0;
            for i=1:length(obj.params.ks)
                A1 = obj.params.arcNodes(:,i); A2 = obj.params.arcNodes(:,i+1);
                k = obj.params.ks(i);
                v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
                C1 = 1/2 * (A1 + A2) + k * v;
                hsq = 1/2 * norm(A1 - A2);
                w = hsq/sqrt(hsq^2 + k^2);

                u = linspace(0,1,1e2);
                y = zeros(2,length(u));
                for s=1:length(u)
                    y(:,s) = ((1-u(s))^2 * A1 + 2 * u(s) * (1-u(s)) * w * C1 + u(s)^2 * A2)/((1-u(s))^2 + 2*u(s)*(1-u(s))*w + u(s)^2);
                end
                
                dist = norm(hsq^2/k * v);
                th = atan2(dist,hsq); 
                arc_length = arc_length + 2 * th * sqrt(hsq^2 + dist);
                p_approx = plot(y(1,:),y(2,:),'k--','LineWidth',1);
            end
            xlabel('Global X'); ylabel('Global Y');
            title('Multiple Arc Approximation')
            legend([p_data,p_approx,p_arcN],'Data Points','Multiple Arc Approximation','Arc Node');            
            disp(['Total Arc Length: ',num2str(arc_length),'m'])
        end
        
        %% Visualize Trajectory-based Event Detection Results
        function visualize_predict(obj,eventIntvs)
            figure(3); clf;
            hold on; grid on; axis equal;
            % f.Position(3:4) = [720, 480];
            % f.Position(2) = f.Position(2) - 200;
            p_arcN = plot(obj.params.arcNodes(1,:),obj.params.arcNodes(2,:),'bo');            
            legendstr = [{'Arc Node'},{'Lane Keeping Maneuver'}];
            yt = cell(1,6);
            markersize = 3;
            
            for i=1:length(obj.params.ks)
                A1 = obj.params.arcNodes(:,i);
                A2 = obj.params.arcNodes(:,i+1);
                k = obj.params.ks(i);
                v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
                C1 = 1/2 * (A1 + A2) + k * v;
                u = linspace(0,1,1e2);
                y = zeros(2,length(u));
    
                hsq = 1/2 * norm(A1 - A2);
                w = hsq/sqrt(hsq^2 + k^2);
    
                for s=1:length(u)
                    y(:,s) = ((1-u(s))^2 * A1 + 2 * u(s) * (1-u(s)) * w * C1 + u(s)^2 * A2)/((1-u(s))^2 + 2*u(s)*(1-u(s))*w + u(s)^2);
                end

                for j = 1 : length(eventIntvs(:,1))
                    if i >= eventIntvs(j,1) && i <= eventIntvs(j,2)
                        n = eventIntvs(j,3)+1;
                        yt{n} = [yt{n},y];
                    else
                        LKM = plot(y(1,:),y(2,:),'ko','Markersize',markersize);
                    end
                end    
            end
           
            if ~isempty(yt{1})
                LKM = plot(yt{1}(1,:),yt{1}(2,:),'ko','Markersize',markersize);
            end
            if ~isempty(yt{2})
                RT = plot(yt{2}(1,:),yt{2}(2,:),'bo','Markersize',markersize);
                legendstr = [legendstr,{'Right Turn'}];
            else
                RT = [];
            end
            if ~isempty(yt{3})
                LT = plot(yt{3}(1,:),yt{3}(2,:),'go','Markersize',markersize);
                legendstr = [legendstr,{'Left Turn'}];
            else
                LT = [];
            end
            if ~isempty(yt{4})
                RLC = plot(yt{4}(1,:),yt{4}(2,:),'yo','Markersize',markersize);
                legendstr = [legendstr,{'Right Lane Change'}];
            else
                RLC = [];
            end
            if ~isempty(yt{5})
                LLC = plot(yt{5}(1,:),yt{5}(2,:),'mo','Markersize',markersize);
                legendstr = [legendstr,{'Left Lane Change'}];
            else
                LLC = [];
            end
            if ~isempty(yt{6})
                RA = plot(yt{6}(1,:),yt{6}(2,:),'co','Markersize',markersize);
                legendstr = [legendstr,{'Roundabout'}];
            else
                RA = [];
            end
            % fontsize1 = 18;
            % fontsize2 = 14;
            fontsize1 = 14; fontsize2 = 12;
            xlabel('Global X','FontSize',fontsize1); ylabel('Global Y','FontSize',fontsize1);
            title('Driving Scenario Classification','FontSize',fontsize1)
            lgd = legend([p_arcN,LKM,RT,LT,RLC,LLC,RA],legendstr,'FontSize',fontsize2);
            lgd.NumColumns = 2;
            hold off;
        end

        %% Get Parameters [x, y, R]
        function xyR = getParams(obj)
            % Return variable xyR is in the following format
            % [xc1, xc2, ... xcN;
            %  yc1, yc2, ... ycN;
            %   R1,  R2. ...  RN];
            xyR = zeros(3,length(obj.params.ks));
            for i=1:length(obj.params.ks)
                A1 = obj.params.arcNodes(:,i);
                A2 = obj.params.arcNodes(:,i+1);
                k = obj.params.ks(i);
                v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
                hsq = 1/2 * norm(A1 - A2);
                Xc = 1/2 * (A1 + A2) - hsq^2/k * v;
                xyR(1:2,i) = Xc;
                xyR(3,i) = norm(Xc - A1);                
            end
        end

        %% Get Curvature
        function curv = getCurvatures(obj)
            curv = zeros(1,length(obj.params.ks));
            for i=1:length(obj.params.ks)
                A1 = obj.params.arcNodes(:,i); A2 = obj.params.arcNodes(:,i+1);
                k = obj.params.ks(i);
                v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
                C1 = 1/2 * (A1 + A2) + k * v;
                hsq = 1/2 * norm(A1 - A2);
                diff1 = C1 - A1; diff2 = A2 - C1;
                dist = norm(hsq^2/k * v);
                th = atan2(dist,hsq); 
                L = 2 * th * sqrt(hsq^2 + dist);
                theta1 = atan2(diff1(2),diff1(1));
                theta2 = atan2(diff2(2),diff2(1));
                curv(i) = (theta2 - theta1)/L;
            end
        end
        
        %% Get Nodes
        function nodes = getNodes(obj)
            nodes = zeros(2,2*length(obj.params.ks)+1);
            nodes(:,1) = obj.params.arcNodes(:,1);
            for i=1:length(obj.params.ks)
                % Compute midNode from parameters
                A1 = obj.params.arcNodes(:,i); A2 = obj.params.arcNodes(:,i+1);
                k = obj.params.ks(i);
                v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
                C1 = 1/2 * (A1 + A2) + k * v;
                hsq = 1/2 * norm(A1 - A2);
                w = hsq/sqrt(hsq^2 + k^2);
                nodes(:,2*i) = (1/2 * A1 + w * C1 + 1/2 * A2)/(1 + w);
                nodes(:,2*i+1) = A2;
            end
        end

    end

    methods (Access = private)
        %% Visualize
        function visualizeInternal(obj,x)
            figure(2);
            arcNodes = reshape(x(1:2*size(obj.params.arcNodes,2)),2,size(obj.params.arcNodes,2));
            ks = reshape(x(2*size(obj.params.arcNodes,2)+1:end),1,length(obj.params.ks));

            p_data = plot(obj.pts(1,:),obj.pts(2,:),'r.'); hold on; axis equal;
            p_arcN = plot(arcNodes(1,:),arcNodes(2,:),'bo');
            arc_length = 0;
            for i=1:length(ks)
                A1 = arcNodes(:,i); A2 = arcNodes(:,i+1);
                k = ks(i);
                v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
                C1 = 1/2 * (A1 + A2) + k * v;
                hsq = 1/2 * norm(A1 - A2);
                w = hsq/sqrt(hsq^2 + k^2);

                u = linspace(0,1,1e2);
                y = zeros(2,length(u));
                for s=1:length(u)
                    y(:,s) = ((1-u(s))^2 * A1 + 2 * u(s) * (1-u(s)) * w * C1 + u(s)^2 * A2)/((1-u(s))^2 + 2*u(s)*(1-u(s))*w + u(s)^2);
                end
                
                dist = norm(hsq^2/k * v);
                th = atan2(dist,hsq); 
                arc_length = arc_length + 2 * th * sqrt(hsq^2 + dist);
                p_approx = plot(y(1,:),y(2,:),'k--','LineWidth',1);
            end
            xlabel('Global X'); ylabel('Global Y');
            title('Multiple Arc Approximation')
            legend([p_data,p_approx,p_arcN],'Data Points','Multiple Arc Approximation','Arc Node');  
            hold off; drawnow;
            % disp(['Total Arc Length: ',num2str(arc_length),'m'])
        end

        %% Initialization
        function obj = initialize(obj,min_flag)
            oneArc = FitCircle(obj.pts,obj.cov);
            oneArc.optimize();
            maxInvalidN = 20;   
            [~,valid_flag,~] = obj.checkValidity(oneArc.params.arcNodes,oneArc.params.k,[1,size(oneArc.pts,2)],oneArc.pts,oneArc.cov,false,maxInvalidN);
            
            if valid_flag || obj.arc_num(1) == 1 || min_flag
                obj.params.arcNodes = oneArc.params.arcNodes;
                obj.params.ks = oneArc.params.k;
            else
                intvs = unique(obj.segmentation(obj.pts,[1,size(obj.pts,2)],1));
                % Find intervals for 3D fitting.
                lb = 1; ub = 2;
                circle_intvs = [];
                while ub <= length(intvs)
                    % disp([lb, ub])
                    % disp(['ub_idx: ',num2str(ub)])
                    lb_idx = intvs(lb); ub_idx = intvs(ub);

                    if ub_idx - lb_idx + 1 <= 3
                        ub = ub + 1;
                        ub_idx = intvs(ub);
                    end

                    pts_sample = obj.pts(:,lb_idx:ub_idx);
                    cov_sample = obj.cov(:,lb_idx:ub_idx);
                    % disp([lb_idx, ub_idx])
                    sample = FitCircle(pts_sample,cov_sample);
                    sample.optimize();

                    arcNodes = sample.params.arcNodes; 
                    ks = sample.params.k;
                    [~,valid_flag2,~] = obj.checkValidity(arcNodes,ks,[1,size(pts_sample,2)],pts_sample,cov_sample,false,maxInvalidN);
                    
                    if valid_flag2
                        ub = ub + 1;
                        if ub == length(intvs)
                            ub_idx = intvs(ub);
                            circle_intvs = [circle_intvs; lb_idx, ub_idx];
                            break;
                        end
                    else
                        
                        if lb ~= ub-1
                            circle_intvs = [circle_intvs; lb_idx, intvs(ub-1)];
                            lb = ub - 1;
                        else 
                            % If one line interval cannot be represented by
                            % a single arc, just assume we can, when
                            % initializing.
                            circle_intvs = [circle_intvs; lb_idx, intvs(ub)];
                            lb = ub; ub = lb + 1;
                        end
                    end
                end

                % disp(['  -->  Initialization is done with ',num2str(size(circle_intvs,1)),' arc segments'])
                
                obj.params.arcNodes = zeros(2,size(circle_intvs,1)+1);
                obj.params.ks = zeros(1,size(circle_intvs,1));
                obj.params.intv = circle_intvs;

                % Re-compute arc parameters and create initial values
                for i=1:size(circle_intvs,1)
                    lb = circle_intvs(i,1); ub = circle_intvs(i,2);
                    pts_sample = obj.pts(:,lb:ub); cov_sample = obj.cov(:,lb:ub);
                    sample = FitCircle(pts_sample,cov_sample);
                    sample.optimize();

                    m1 = sample.params.arcNodes(:,1); m2 = sample.params.arcNodes(:,2);
                    
                    obj.params.ks(:,i) = sample.params.k;
                    if size(circle_intvs,1) == 1
                        obj.params.arcNodes(:,i) = m1;
                        obj.params.arcNodes(:,i+1) = m2;
                    else
                        if i == 1
                            obj.params.arcNodes(:,i) = m1;
                            obj.params.arcNodes(:,i+1) = m2/2;
                        elseif i == size(circle_intvs,1)
                            obj.params.arcNodes(:,i) = obj.params.arcNodes(:,i) + m1/2;
                            obj.params.arcNodes(:,i+1) = m2;
                        else
                            obj.params.arcNodes(:,i) = obj.params.arcNodes(:,i) + m1/2;
                            obj.params.arcNodes(:,i+1) = obj.params.arcNodes(:,i+1) + m2/2;
                        end
                    end
                end
            end
        end
        
        %% Recursive Segmentation
        function intvs = segmentation(obj,pts,intv,depth)
            % (1). Connect first and end intervals with a line
            % (2). Find the maximum vertical error data point
            % (3). Divide the data points of interest into two w.r.t point found at (b).
            % (4). Repeat (1)~(3) for every divided segments.
            % * If maximum error computed at (2) is lower than threshold, 
            % stop dividing current segment further and propagate backwards. 

            % Adjust this value to increase or decrease the number of line
            % interpolation. Typically, values smaller than 0.3m is
            % recommended.
            %
            % Implemented by Jinhwan Jeon, 2022 (Master's Thesis)
            line_acc_thres = 0.2; % 0.2

            % Create Line and visualize current segmentation state 
            init_point = pts(:,intv(1));
            last_point = pts(:,intv(2));
            direc_vec = (last_point - init_point)/norm(last_point - init_point);
            diff_vecs = pts(:,intv(1):intv(2)) - init_point;
            proj_ds = direc_vec' * diff_vecs;
            org_ds = vecnorm(diff_vecs);
            ds = (org_ds.^2 - proj_ds.^2).^(1/2);
            [max_val, max_idx] = max(ds);
            
            % Visualize 
            % if depth == 1 || depth == 2 || depth == 3 || depth == 4
            %     f = figure(100+depth); f.Position(3:4) = [720, 480];
            %     p_d = plot(pts(1,:),pts(2,:),'r.'); hold on; axis equal;
            % 
            %     x = linspace(init_point(1),last_point(1),1e3);
            %     y = linspace(init_point(2),last_point(2),1e3);
            %     p_lineapprox = plot(x,y,'k--');
            %     plot([x(1) x(end)],[y(1), y(end)],'bs','MarkerSize',12,'LineWidth',2)
            % end
            
            % Find existence of intersection
            % Even if more than 1 intersection occurs, divide current segment with
            % two segments. For stability, search intersection from the middle
            % If the maximum error caused by "Line Interpolation" is lower than
            % threshold, end segmentation(return current search interval). 
            % Else, divide segmentation problem using the maximum error point
            % [max_val,max_idx] = max(abs(d));
            % disp([intv(1), intv(2), depth, max_idx, max_val])
            % if depth == 3
            %     error('1')
            % end
            fontsize1 = 18;
            fontsize2 = 14;
            % title('[Proposed] Multiple Arc Optimization','FontSize',fontsize1)
            
            % if depth == 1 || depth == 2 || depth == 3 || depth == 4
            %     p_maxE = plot(pts(1,intv(1)+max_idx-1),pts(2,intv(1)+max_idx-1),'cs','MarkerSize',12,'LineWidth',2);
            %     xlabel('Local X','FontSize',fontsize1); ylabel('Local Y','FontSize',fontsize1);
            %     title('Recursive Linearization of Data Points','FontSize',fontsize1)
            %     legend([p_d,p_lineapprox,p_maxE],'Data Points','Linear Approximation','Point of Maximum Line Fit Error','FontSize',fontsize2)
            % 
            %     pause(0.1);
            % end
            
            % End segmentation or divide current segment into 2 and
            % continue search (Divide and Conquer)
            if max_val < line_acc_thres
                % Current line interpolation is accurate enough
                intvs = intv;
            else
                intv1 = obj.segmentation(pts,[intv(1) intv(1)+max_idx-1],depth+1);
                intv2 = obj.segmentation(pts,[intv(1)+max_idx-1 intv(2)],depth+1);
                intvs = [intv1 intv2];
            end
        
        end
        
        %% Cost Function
        function [res,jac] = cost_func(obj,x)
            % obj.visualizeInternal(x);
            arcNodes = reshape(x(1:2*(size(obj.params.intv,1)+1)),2,size(obj.params.intv,1)+1);
            ks = reshape(x(2*(size(obj.params.intv,1)+1)+1:end),1,size(obj.params.intv,1));
            
            % Perform data association
            intv = obj.associate(arcNodes);
            % Concatenate residual and jacobians
            [resA,jacA] = obj.createACBlock(arcNodes,ks,intv);
            [resM,jacM] = obj.createMEBlock(arcNodes,ks,intv);
            res = [resA;resM];
            jac = [jacA;jacM];
        end
        
        %% Anchor Block
        function [res,jac] = createACBlock(obj,arcNodes,ks,intv)
            idxs = [1,intv(:,2)'];
            % idxs = [1,intv(end,2)];
            res = zeros(2*length(idxs),1);
            I = zeros(1,4*length(idxs)); J = I; V = I;
            for i=1:length(idxs)
                if i == 1 || i == length(idxs)
                    ac_cov = diag([1e-4,1e-4]); %-6
                else
                    ac_cov = 10^(4) * eye(2);% diag([1e-4,1e-4]); 
                end                
                pt = obj.pts(:,idxs(i));
                arcNode = arcNodes(:,i);
                res(2*i-1:2*i) = obj.InvMahalanobis(obj.static_methods.getACRes(pt,arcNode),ac_cov);
                [I1,J1,V1] = obj.sparseFormat(2*i-1:2*i,2*i-1:2*i,obj.InvMahalanobis(eye(2),ac_cov));
                I(4*(i-1)+1:4*i) = I1; J(4*(i-1)+1:4*i) = J1; V(4*(i-1)+1:4*i) = V1;
            end
            jac = sparse(I,J,V,length(res),2*size(arcNodes,2)+length(ks));
        end
        
        %% Measurement Block
        function [res,jac] = createMEBlock(obj,arcNodes,ks,intv)
            res = []; jac = [];
            for i=1:size(intv,1)
                lb = intv(i,1); ub = intv(i,2);
                res_seg = zeros(2*(ub-lb+1),1);
                % res_seg = [];
                % I_seg = [];
                I_seg = zeros(1,2*5*(ub-lb+1));
                J_seg = I_seg; V_seg = I_seg;
                
                A1 = arcNodes(:,i); A2 = arcNodes(:,i+1);
                k = ks(i);

                for j=lb:ub
                    pt = obj.pts(:,j);
                    [me_res, me_jac] = obj.static_methods.getMEResJac(A1,A2,k,pt);
                    aN_jac = me_jac(:,1:4); k_jac = me_jac(:,5);
                    
                    me_cov = reshape(obj.cov(:,j),2,2);
                    res_seg(2*(j-lb)+1:2*(j-lb+1)) = obj.InvMahalanobis(me_res,me_cov);
                    [I1,J1,V1] = obj.sparseFormat(2*(j-lb)+1:2*(j-lb+1),2*i-1:2*(i+1),obj.InvMahalanobis(aN_jac,me_cov));
                    [I2,J2,V2] = obj.sparseFormat(2*(j-lb)+1:2*(j-lb+1),2*size(arcNodes,2)+i,obj.InvMahalanobis(k_jac,me_cov));
                    I_seg(2*5*(j-lb)+1:2*5*(j-lb+1)) = [I1,I2];
                    J_seg(2*5*(j-lb)+1:2*5*(j-lb+1)) = [J1,J2];
                    V_seg(2*5*(j-lb)+1:2*5*(j-lb+1)) = [V1,V2];                    
                end
                jac_seg = sparse(I_seg,J_seg,V_seg,length(res_seg),2*size(arcNodes,2)+length(ks));
                res = [res; res_seg];
                jac = [jac; jac_seg];
            end
        end
        
        %% Constraint Function
        function [c,ceq,GC,GCeq] = nonlcon(obj,x)
            arcNodes = reshape(x(1:2*(size(obj.params.intv,1)+1)),2,size(obj.params.intv,1)+1);
            ks = reshape(x(2*(size(obj.params.intv,1)+1)+1:end),1,size(obj.params.intv,1));
            [ceq1,GCeq1] = obj.createCS1Block(arcNodes,ks);
            [c1,GC1] = obj.createCS2Block(arcNodes,ks);
            c = c1; GC = GC1';
            ceq = ceq1; GCeq = GCeq1';
            % GC = GC'; GCeq = GCeq'; % Need to transpose due to matlab format
        end

        %% Constraint Block 1
        function [res,jac] = createCS1Block(obj,arcNodes,ks)
            % G1 Continuity Constraint
            if length(ks) > 1
                % res = zeros(2*(size(arcNodes,2)-2),1);
                res = zeros(2*(size(arcNodes,2)-2),1);
                I = zeros(1,length(res)*8); J = I; V = I;
                for i=2:size(arcNodes,2)-1
                    A1 = arcNodes(:,i-1); A2 = arcNodes(:,i); A3 = arcNodes(:,i+1);
                    k1 = ks(i-1); k2 = ks(i);                    
                    res(2*(i-2)+1:2*(i-1)) = obj.static_methods.getCS1Res(A1,A2,A3,k1,k2);                    
                    cs1_jac = obj.static_methods.getCS1Jac(A1,A2,A3,k1,k2); 
                    aN_jac = cs1_jac(:,1:6); k_jac = cs1_jac(:,7:8);

                    [I1,J1,V1] = obj.sparseFormat(2*(i-2)+1:2*(i-1),2*(i-2)+1:2*(i+1),aN_jac);
                    [I2,J2,V2] = obj.sparseFormat(2*(i-2)+1:2*(i-1),2*size(arcNodes,2)+i-1:2*size(arcNodes,2)+i,k_jac);
                    I(2*8*(i-2)+1:2*8*(i-1)) = [I1,I2];
                    J(2*8*(i-2)+1:2*8*(i-1)) = [J1,J2];
                    V(2*8*(i-2)+1:2*8*(i-1)) = [V1,V2];
                end
                jac = sparse(I,J,V,length(res),2*size(arcNodes,2)+length(ks));
            else
                res = []; jac = [];
            end
        end

        %% Constraint Block 2
        function [res,jac] = createCS2Block(obj,arcNodes,ks)
            res = zeros(length(ks),1);
            I = zeros(1,5*length(ks)); J = I; V = I;
            for i=1:length(ks)
                A1 = arcNodes(:,i); A2 = arcNodes(:,i+1);
                k = ks(i);
                res(i) = obj.static_methods.getCS2Res(A1,A2,k,obj.L_min);
                cs2_jac = obj.static_methods.getCS2Jac(A1,A2,k,obj.L_min);
                aN_jac = cs2_jac(1,1:4); k_jac = cs2_jac(1,5);
                [I1,J1,V1] = obj.sparseFormat(i,2*i-1:2*(i+1),aN_jac);
                [I2,J2,V2] = obj.sparseFormat(i,2*size(arcNodes,2)+i,k_jac);
                I(5*(i-1)+1:5*i) = [I1,I2];
                J(5*(i-1)+1:5*i) = [J1,J2];
                V(5*(i-1)+1:5*i) = [V1,V2];
            end
            jac = sparse(I,J,V,length(res),2*size(arcNodes,2)+length(ks));
        end

        %% Data Association
        function intv = associate(obj,arcNodes)
            delayedPoints = [obj.pts(:,1),obj.pts(:,1:end-1)];
            diffPoints = obj.pts - delayedPoints;
            % search_radius = 10 * mean(vecnorm(diffPoints));
            % search_radius = 20;
            search_radius_base = 10 * mean(vecnorm(diffPoints)); 

            intv = zeros(size(obj.params.intv));
            intv(1,1) = 1; intv(end,2) = size(obj.pts,2);
            pts_ = [obj.pts(:,2:end);2:size(obj.pts,2)];
            
            for i=2:size(arcNodes,2)-1
                ax = arcNodes(1,i); ay = arcNodes(2,i);
                d = sqrt((pts_(1,:) - ax).^2 + (pts_(2,:) - ay).^2);
                
                % Modified version of data association.
                % In order to prevent data association error for data
                % points with overlapping region, index difference will
                % also be considered when choosing the "closest point"

                % Index of points that lie inside the search radius
                % Note that the indices are not the "absolute indices" but
                % they are "relative". This is because matched points are 
                % deleted automatically. Absolute index can be accessed by 
                % using the 3rd row of the 'pts_' object.
                
                % TrueMatchedIdxs = pts_(3,d < search_radius);
                % TrueMatchedIntvs = obj.getIntvs(TrueMatchedIdxs);
                
                RelMatchedIdxs = []; search_radius = search_radius_base;
                while isempty(RelMatchedIdxs)
                    RelMatchedIdxs = find(d < search_radius);
                    search_radius = search_radius * 2;
                end

                % RelMatchedIdxs = find(d < search_radius);
                RelMatchedIntvs = obj.getIntvs(RelMatchedIdxs);
                RelSearchIdxs = RelMatchedIntvs(1,1):RelMatchedIntvs(1,2);
                Ds = d(RelSearchIdxs);
                [~,minIdx] = min(Ds);
                RelRemIdx = RelSearchIdxs(minIdx);
                
                intv(i-1,2) = pts_(3,RelRemIdx);
                intv(i,1) = pts_(3,RelRemIdx);
                pts_ = pts_(:,RelRemIdx+1:end);
            end
        end

        %% Check Validity
        function [err,valid_flag,idx] = checkValidity(obj,arcNodes,ks,intv,pts,cov,disp_flag,maxN)
            err = cell(1,length(ks));
            
            % disp(['Validity check for ',num2str(length(ks)),' optimized segments'])
            invalid_idxs = [];
            invalid_N = [];

            for i=1:length(ks)
                lb = intv(i,1); ub = intv(i,2);
                err{i} = struct();
                err{i}.wrmse = 0; err{i}.rmse = 0;
                err{i}.full = zeros(1,ub-lb+1);
                err{i}.wfull = zeros(1,ub-lb+1);

                A1 = arcNodes(:,i); A2 = arcNodes(:,i+1); k = ks(i);
                
                for j=lb:ub
                    [res,~] = obj.static_methods.getMEResJac(A1,A2,k,pts(:,j));
                    err{i}.rmse = err{i}.rmse + res' * res;
                    err{i}.wrmse = err{i}.wrmse + res' / reshape(cov(:,j),2,2) * res;
                    err{i}.full(j-lb+1) = norm(res);
                    err{i}.wfull(j-lb+1) = sqrt(res' / reshape(cov(:,j),2,2) * res);
                end

                err{i}.invalidN = length(find(err{i}.wfull > sqrt(chi2inv(0.99,2))));
                err{i}.rmse = sqrt(err{i}.rmse/(ub-lb+1));
                err{i}.wrmse = sqrt(err{i}.wrmse/(ub-lb+1));
                err{i}.max = max(err{i}.full);
                err{i}.wmax = max(err{i}.wfull);
                
                % if disp_flag
                %     disp(['[Segment No. ',num2str(i),']'])
                %     disp(['Number of invalid approximations: ',num2str(err{i}.invalidN)])
                %     disp(['RMSE of fitting: ',num2str(err{i}.rmse)])
                %     disp(['Maximum fitting error: ',num2str(err{i}.max)])
                % end
                
                
                if maxN < err{i}.invalidN
                    invalid_idxs = [invalid_idxs, i];
                    invalid_N = [invalid_N, err{i}.invalidN];
                end
            end

            if isempty(invalid_idxs)
                idx = -1;
                valid_flag = true;
                % if disp_flag
                %     disp('[All segments are valid. Optimization finished]')
                % end
            else
                [~,rel_idx] = max(invalid_N);
                idx = invalid_idxs(rel_idx);
                % if disp_flag
                %     disp('Invalid Arc Segment Idx: ')
                %     disp(invalid_idxs)
                %     disp(['Number of invalid arc segments: ',num2str(length(invalid_idxs))])
                %     disp('---------------------------------------------------------------------------------')
                %     % disp(['[Segment no.',num2str(idx),' has ',num2str(maxN),' invalid approximations: Invalid]'])
                % end
                valid_flag = false;
            end    
        end
        
        %% Update Parameters
        function obj = updateParams(obj,idx)
            % Halve segment with the most number of invalid approximations
            A1 = obj.params.arcNodes(:,idx); A2 = obj.params.arcNodes(:,idx+1);
            k = obj.params.ks(:,idx);
            hsq = 1/2 * norm(A1 - A2);
            v = 1/(2*hsq)*[0,-1;1,0] * (A2 - A1);
            C = 1/2 * (A1 + A2) + k * v;
            Xc = 1/2 * (A1 + A2) - hsq^2/k * v;
            w = hsq/sqrt(hsq^2 + k^2);

            new_aN = 1/(2*w+2) * A1 + w/(w+1) * C + 1/(2*w+2) * A2;

            hsq1 = 1/2 * norm(A1 - new_aN); 
            v1 = 1/norm(A1 - new_aN) * [0,-1;1,0] * (new_aN - A1);
            d1 = 1/2 * (A1 + new_aN) - Xc;
            new_k1 = hsq1^2/(d1'*v1);

            hsq2 = 1/2 * norm(A2 - new_aN); 
            v2 = 1/norm(A2 - new_aN) * [0,-1;1,0] * (A2 - new_aN);
            d2 = 1/2 * (A2 + new_aN) - Xc;
            new_k2 = hsq2^2/(d2'*v2);

            front_arcNodes = obj.params.arcNodes(:,1:idx);
            back_arcNodes = obj.params.arcNodes(:,idx+1:end);
            obj.params.arcNodes = [front_arcNodes, new_aN, back_arcNodes];

            front_ks = obj.params.ks(:,1:idx-1);
            back_ks = obj.params.ks(:,idx+1:end);
            obj.params.ks = [front_ks, new_k1, new_k2, back_ks];
            
            d = (obj.pts(1,:) - new_aN(1)).^2 + (obj.pts(2,:) - new_aN(2)).^2;
            [~,min_idx] = min(d);

            front_intv = obj.params.intv(1:idx-1,:);
            back_intv = obj.params.intv(idx+1:end,:);
            obj.params.intv = [front_intv; obj.params.intv(idx,1), min_idx; min_idx, obj.params.intv(idx,2); back_intv];
        end
    end

    methods (Static)
        function ER = InvMahalanobis(M,cov)
        % INVMAHALANOBIS Square root weighting using covariance matrix
        % This function is mainly used for weighting in Non-linear Least Squares 
            n = size(cov,1);
            SIG = eye(n)/chol(cov);
            SIG = SIG';
            ER = SIG * M;
        end

        function [I, J, V] = sparseFormat(rows, cols, values)
        % sparseFormat returns the matrix indices and corresponding value array in  
        % sparse matrix format. This function is mainly used for constructing
        % sparse Jacobian matrix efficiently.
        %
        % [I, J, V] = sparseFormat(rows,cols,values)
        % A = sparse(I,J,V,m,n) --> sparse matrix can be constructed directly
        
            m = length(rows); n = length(cols);
            if ne(size(values), [m n])
                error('Value matrix format does not match row, column information')
            end
            
            I = zeros(1,m*n); J = zeros(1,m*n); V = zeros(1,m*n);
            for i=1:m
                for j=1:n
                    I((i-1)*n+j) = rows(i);
                    J((i-1)*n+j) = cols(j);
                    V((i-1)*n+j) = values(i,j);
                end
            end
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
end



