function VisualizeMap(file_path)
    % Parse .osm (xml format) file
    map = loadosm(file_path);
    
    %% Generated OSM File
    f = figure(1); geobasemap satellite;
    %% Arc Spline Approximated Lanelet
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
    saveas(f,file_path+".fig");
end