function eventIntvs = DMDC(date,tripNo)
    %% DMDC loads precomputed arc spline approximation and vehicle sensor fusion results 
    % to perform driving maneuver event detection and classification.
    % 
    % Arc spline approximation and vehicle sensor fusion process is skipped because 
    % it is not the main focus of the presented work.
    % 
    % Arc spline approximation framework implementation will be introduced in another work.
    
    rtc_file_path = strcat("data/",date,"/",num2str(tripNo),"/RTC.mat");
    RTC = load(rtc_file_path).rtc; % Load precomputed RoadTypeClassfier2 object (Arc spline approximated vehicle trajectory)
    eventIntvs = RTC.run('test'); % Run prediction
end