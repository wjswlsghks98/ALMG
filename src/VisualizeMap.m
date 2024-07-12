function VisualizeMap(args)
    file_path = args{1};
    arc_approx_flag = args{2};
    % Parse .osm (xml format) file
    map = loadosm(file_path);
    
    %% Generated OSM File
       
    f = figure(1); geobasemap satellite;
    %% Arc Spline Approximated Lanelet
    if strcmp(arc_approx_flag,'y')
        lla0 = [36.5222, 127.3032, 181.1957];
        for i=1:length(map.ways)
            type = map.ways(i).tags("type");
            
            if strcmp(type,'traffic_light')
                p_tf = geoplot(map.ways(i).points(1,:),map.ways(i).points(2,:),'r-x'); hold on;
            elseif strcmp(type,'stop_line')
                p_st = geoplot(map.ways(i).points(1,:),map.ways(i).points(2,:),'b-x'); hold on;
            else
                enu_ls = lla2enu([map.ways(i).points; zeros(1,size(map.ways(i).points,2))]',lla0,"ellipsoid")';

                n = fix(size(enu_ls,2)/2);
                arcNodes = enu_ls(1:2,1:2:2*n+1);
                midNodes = enu_ls(1:2,2:2:2*n+1);
                
                for k=1:n
                    A1 = arcNodes(:,k); A2 = arcNodes(:,k+1);
                    N1 = midNodes(:,k);
                    v1 = N1 - A1; v2 = A2 - A1;
                    v11 = v1'*v1; v22 = v2'*v2; v12 = v1' * v2;
                    b = 1/2 * 1/(v11*v22 - v12^2);
                    k1 = b * v22 * (v11 - v12);
                    k2 = b * v11 * (v22 - v12);
                    Xc = A1 + k1*v1 + k2*v2;
                    m = 1/2 * (A1 + A2);
                    hsq = 1/2 * norm(A1 - A2);
                    dist = norm(m - Xc);
                    
                    k_norm = hsq^2/dist;
                    C = m + (m-Xc)/dist * k_norm;
                    w = hsq/sqrt(hsq^2 + k_norm^2);
                    u = linspace(0,1,1e3);
                    y = zeros(2,length(u));
                    for s=1:length(u)
                        y(:,s) = ((1-u(s))^2 * A1 + 2 * u(s) * (1-u(s)) * w * C + u(s)^2 * A2)/((1-u(s))^2 + 2*u(s)*(1-u(s))*w + u(s)^2);
                    end
                    y = [y; zeros(1,size(y,2))];

                    lla_y = enu2lla(y',lla0,"ellipsoid")';
                    p_approx = geoplot(lla_y(1,:),lla_y(2,:),'k--'); hold on;
                    p_node = geoplot([lla_y(1,1),lla_y(1,end)],[lla_y(2,1),lla_y(2,end)],"go");
                end
            end
        end
        legend([p_tf,p_st,p_approx,p_node],"Traffic Lights","Stop Lines","Arc Spline Approximation","Arc Nodes");

    %% Point wise saved Lanelet
    elseif strcmp(arc_approx_flag,'n')
        for i=1:length(map.ways)
            for j=1:size(map.ways(i).tags,1)
                type = map.ways(i).tags("type");            
                if strcmp(type,'traffic_light')
                    p_tf = geoplot(map.ways(i).points(1,:),map.ways(i).points(2,:),'r-x'); hold on;
                elseif strcmp(type,'stop_line')
                    p_st = geoplot(map.ways(i).points(1,:),map.ways(i).points(2,:),'b-x'); hold on;
                else
                    p_approx = geoplot(map.ways(i).points(1,:),map.ways(i).points(2,:),'k--'); hold on;
                    geoplot([map.ways(i).points(1,1),map.ways(i).points(1,end)],[map.ways(i).points(2,1),map.ways(i).points(2,end)],'go');
                end
            end
        end
        legend([p_tf,p_st,p_approx],"Traffic Lights","Stop Lines","LineStrings");
    end
    
    %% Plot HD Map if exists
    if length(args) >= 3
        ref = readgeotable(args{3});
        ref_crs = ref.Shape.ProjectedCRS;
        for i=4:length(args)
            hdmap = readgeotable(args{i});
            hdmap.Shape.ProjectedCRS = ref_crs;
            geoplot(hdmap);
        end
    end
    saveas(f,file_path+".fig");
end