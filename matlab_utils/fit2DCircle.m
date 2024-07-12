classdef fit2DCircle < handle
    %FIT2DCIRCLE module approximates a given point dataset using single
    % arc, data point covariance data.
    %
    % [Input arguments]
    % pts: Data points (2*n matrix)
    % cov: Covariance of data points (4*n matrix) --> each column will be reshaped into 2*2 covariance matrix.
    % 
    % [Usage examples]
    % approx = fit2DCircle(pts,cov);
    % approx.optimize();
    % approx.visualize();
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
        pts
        cov
        params
        static_methods
        offset
    end

    methods (Access = public)
        %% Constructor
        function obj = fit2DCircle(pts,cov)
            obj.offset = mean(pts,2);
            obj.pts = pts - obj.offset;
            obj.cov = cov;
            obj.static_methods = StaticMethods();
            obj.params = struct();
        end

        %% Optimize
        function obj = optimize(obj)
            obj.initialize();
            x0 = [reshape(obj.params.arcNodes,4,1); obj.params.k];
            lb = []; ub = []; 
            options = optimoptions('lsqnonlin','Algorithm','trust-region-reflective','SpecifyObjectiveGradient',true,'Display','none','MaxFunctionEvaluations',inf);
            x = lsqnonlin(@obj.cost_func,x0,lb,ub,options);
            
            obj.pts = obj.pts + obj.offset;
            obj.params.arcNodes = [x(1:2),x(3:4)] + obj.offset;
            obj.params.k = x(5);
        end

        %% Visualize
        function visualize(obj)
            figure(1);
            p_data = plot(obj.pts(1,:),obj.pts(2,:),'r.'); hold on; axis equal;
            p_arcN = plot(obj.params.arcNodes(1,:),obj.params.arcNodes(2,:),'bo');
            A1 = obj.params.arcNodes(:,1);
            A2 = obj.params.arcNodes(:,2);
            k = obj.params.k;
            v = 1/norm(A2-A1) * [0,-1;1,0] * (A2-A1);
            C1 = 1/2 * (A1 + A2) + k * v;
            u = linspace(0,1,1e2);
            y = zeros(2,length(u));

            hsq = 1/2 * norm(A1 - A2);
            w = hsq/sqrt(hsq^2 + k^2);

            for s=1:length(u)
                y(:,s) = ((1-u(s))^2 * A1 + 2 * u(s) * (1-u(s)) * w * C1 + u(s)^2 * A2)/((1-u(s))^2 + 2*u(s)*(1-u(s))*w + u(s)^2);
            end
            p_approx = plot(y(1,:),y(2,:),'k--');
            xlabel('Global X'); ylabel('Global Y');
            % title('[Proposed] Multiple Arc Optimization','FontSize',fontsize1)
            title('Single Arc Approximation')
            legend([p_data,p_approx,p_arcN],'Data Points','Multiple Arc Approximation','Arc Node');
        end

    end
    methods (Access = private)
        %% Initialize
        function obj = initialize(obj)
            obj.params.arcNodes = [obj.pts(:,1), obj.pts(:,end)];
            p1 = obj.params.arcNodes(:,1);
            p3 = obj.params.arcNodes(:,2);
            pt_idx = floor((size(obj.pts,2)+1)/2);
            p2 = obj.pts(:,pt_idx);
            v1 = p2 - p1; v2 = p3 - p1;
            v11 = v1'*v1; v22 = v2'*v2; v12 = v1' * v2;
            b = 1/2 * 1/(v11*v22 - v12^2);
            k1 = b * v22 * (v11 - v12);
            k2 = b * v11 * (v22 - v12);
            Xc = p1 + k1*v1 + k2*v2;
            
            v = 1/norm(p3-p1) * [0,-1;1,0]*(p3-p1);
            vec = -Xc + 1/2 * (p1 + p3); % = hsq^2/k * v
            hsq = 1/2 * norm(p1 - p3);
            obj.params.k = hsq^2/(vec' * v);
        end

        %% Cost Function
        function [res,jac] = cost_func(obj,x)
            A1 = x(1:2); A2 = x(3:4); k = x(5);            
            [resA,jacA] = obj.createACBlock(A1,A2);
            [resB,jacB] = obj.createMEBlock(A1,A2,k);
            res = [resA;resB];
            jac = [jacA;jacB];
        end
        
        %% Anchor Block
        function [res,jac] = createACBlock(obj,A1,A2)
            ac_cov = 1e-4 * eye(4);
            res = InvMahalanobis([A1 - obj.pts(:,1);A2 - obj.pts(:,end)],ac_cov);
            jac = InvMahalanobis([eye(4),zeros(4,1)],ac_cov);
        end

        %% Measurement Block
        function [res,jac] = createMEBlock(obj,A1,A2,k)
            res = zeros(2*size(obj.pts,2),1);
            jac = zeros(2*size(obj.pts,2),5);
            for i=1:size(obj.pts,2)
                pt = obj.pts(1:2,i);
                [me_res, me_jac] = obj.static_methods.getMEResJac(A1,A2,k,pt);
                res(2*i-1:2*i) = InvMahalanobis(me_res,reshape(obj.cov(:,i),2,2));
                jac(2*i-1:2*i,:) = InvMahalanobis(me_jac,reshape(obj.cov(:,i),2,2));
            end
        end

    end
end