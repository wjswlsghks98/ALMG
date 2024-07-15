classdef RoadTypeClassifier2 < handle
% ROADTYPECLASSIFIER2 is a Road Type Classifier module that determines
% events during a single drive. Lane points can be divided into several
% segments by Real/Virtual lanes.
% Implemented by JinHwan Jeon and Chaeho Lim, 2023
    properties
        SingleTripFit
        EventLabelIntvs
        EventLabelIntvs_
        opt
        direc
    end
    
    methods(Access=public)
        %% Constructor
        function obj = RoadTypeClassifier2(opt)
            % Depending on string flag, different funcions are operated.
            % [flag]
            % 'test' : Run trajectory-based event classification based on
            % pre-trained data
            % 'train' : Train trajectory-based event classifier
            % 'extract' : Extract features from trip data
            obj.fit2DCirclesSingleTrip(opt);
            obj.opt = opt;
            
            % Fix directories for custom modification
            if ispc
                obj.direc = 'D:/SJ_Dataset/MatlabFiles/github/Mapping/EventDetection/TrajectoryBasedDetection/';
            elseif isunix
                obj.direc = '/mnt/hdd1/SJ_Dataset/MatlabFiles/github/Mapping/EventDetection/TrajectoryBasedDetection/';
            end
        end
        
        %% Run 
        function eventIntvs = run(obj,flag)
            % Fix directories for custom modification
            if ispc
                obj.direc = 'D:/SJ_Dataset/MatlabFiles/github/Mapping/EventDetection/TrajectoryBasedDetection/';
            elseif isunix
                obj.direc = '/mnt/hdd1/SJ_Dataset/MatlabFiles/github/Mapping/EventDetection/TrajectoryBasedDetection/';
            end

            if strcmp(flag,'extract')
                eventIntvs = [];
                obj.extract();
            elseif strcmp(flag,'extract2')
                eventIntvs = [];
                obj.extract2();
            elseif strcmp(flag,'train')
                eventIntvs = [];
                obj.train();
            elseif strcmp(flag,'test')
                eventIntvs = obj.test();
            end
        end

        %% Allocate Labels for Events (Arc Segments)
        function obj = labelEvents(obj)
            xyR = obj.SingleTripFit.getParams();
            
            % Events are detected with radius less than 150m
            EventRadiusIntvs = getIntvs(find(xyR(3,:) < 700)); % 150 ->700 by cho
            
            % Driving Maneuver Type Label 
            % 
            % 0: Lane Keeping Maneuver (Straight, Curvy Road)
            % 1: Right Turn
            % 2: Left Turn
            % 3: Right Lane Change
            % 4: Left Lane Change
            % 5: Roundabout
            %
            % Note that region with label 0 can further be sliced using
            % vision based methods
            EventIntvs = []; Labels = [];
            
            disp('[Event Labelling Process]')
            for i=1:size(EventRadiusIntvs,1)
                lb = EventRadiusIntvs(i,1); ub = EventRadiusIntvs(i,2);
                
                break_flag = false;
                while true
                    disp(['Enter lower and upper bounds of arc segment number for a particular event from ',num2str(lb),' to ',num2str(ub),' in list form: '])
                    Eventintv = input('Input format: [a, b]   ');
                    
                    if Eventintv(1) == lb && Eventintv(2) == ub
                        break_flag = true;
                    else
                        if Eventintv(1) ~= lb
                            error(['Invalid input. Event lower bound should be equal to ',num2str(lb)])
                        elseif Eventintv(2) > ub
                            error(['Invalid input. Event upper bound should be smaller or equal to ',num2str(ub)])
                        else
                            lb = Eventintv(2)+1;
                        end
                    end

                    disp('Enter event type for the particular event:')
                    disp('0: Lane Keeping Maneuver (Straight, Curvy Road)')
                    disp('1: Right Turn')
                    disp('2: Left Turn')
                    disp('3: Right Lane Change')
                    disp('4: Left Lane Change')
                    disp('5: Roundabout')
                    Label = input('Enter in integer form   ');

                    EventIntvs = [EventIntvs; Eventintv];
                    Labels = [Labels; Label];

                    if break_flag
                        break;
                    end
                end
                
            end
            
            % Using data association interval, sample features
            StateIntvs = obj.SingleTripFit.params.intv;

            EventStateIntvs = zeros(size(EventIntvs));
            for i=1:size(EventIntvs,1)
                lb = EventIntvs(i,1); ub = EventIntvs(i,2);
                % Access the data association interval
                EventStateIntvs(i,1) = StateIntvs(lb,1);
                EventStateIntvs(i,2) = StateIntvs(ub,2);
            end
            
            obj.EventLabelIntvs = [EventStateIntvs, Labels];
        end

        %% Allocate Labels for Events (Each Arc Segment)
        function obj = labelEventsEach(obj)
            xyR = obj.SingleTripFit.getParams();
            
            % Events are detected with radius less than 700m
            EventArcs = find(xyR(3,:) < 700); % 150 ->700 by cho
            
            % Driving Maneuver Type Label 
            % 
            % 0: Lane Keeping Maneuver (Straight, Curvy Road)
            % 1: Right Turn
            % 2: Left Turn
            % 3: Right Lane Change
            % 4: Left Lane Change
            % 5: Roundabout
            %
            % Note that region with label 0 can further be sliced using
            % vision based methods
            EventItvs = []; Labels = [];
            
            disp('[Event Labelling Process for Each Arc]')
            for i=1:length(EventArcs)
                idx = EventArcs(i);
                disp(['Enter event type of arc ',num2str(idx)])
                disp('0: Lane Keeping Maneuver (Straight, Curvy Road)')
                disp('1: Right Turn')
                disp('2: Left Turn')
                disp('3: Right Lane Change')
                disp('4: Left Lane Change')
                disp('5: Roundabout')
                Label = input('Enter in integer form   ');
                if Label == 6
                    continue;
                end

                Eventintv = [idx,idx];
                EventItvs = [EventItvs; Eventintv];
                Labels = [Labels; Label];
            end
            
            % Using data association interval, sample features
            StateIntvs = obj.SingleTripFit.params.intv;

            EventStateIntvs = zeros(size(EventItvs));
            for i=1:size(EventItvs,1)
                lb = EventItvs(i,1); ub = EventItvs(i,2);
                % Access the data association interval
                if Labels(i) > 10
                    EventStateIntvs(i,1) = StateIntvs(lb,1)+1;
                    Labels(i) = Labels(i)-10;
                else 
                    EventStateIntvs(i,1) = StateIntvs(lb,1);
                end
                EventStateIntvs(i,2) = StateIntvs(ub,2);
            end
            
            obj.EventLabelIntvs = [EventStateIntvs, Labels];
            obj.EventLabelIntvs_ = obj.conti(obj.EventLabelIntvs);
        end

    end

    methods(Access=private)
        %% Fit Full Single Trip Trajectory with Arcs
        function obj = fit2DCirclesSingleTrip(obj,opt)
            % Approximate vehicle trajectory using arc segments
            P = zeros(2,length(opt.states));
            for i=1:length(opt.states)
                P(:,i) = opt.states{i}.P(1:2);
            end
            covs = repmat(reshape(5e-4*eye(2),4,1),[1,length(opt.states)]);

            % fit2DCircles is not include in this Repository. 
            % Wait for our future work to be released! 
            obj.SingleTripFit = fit2DCircles(P,covs);
            obj.SingleTripFit.L_min = 10; % Set minimum possible arc length to be 5m
            obj.SingleTripFit.optimize(true);
            % obj.SingleTripFit.visualize();
        end
        
        %% Data extraction
        function obj = extract(obj)
            % playVid(obj.opt.t(1),obj.opt.t(2),[],1,[]);

            obj.labelEventsEach();
            itvs = obj.EventLabelIntvs;
            itvs_ = obj.EventLabelIntvs_;
            save(strcat(obj.direc,'src/Detection/itvs/',obj.opt.date,'-',num2str(obj.opt.tripNo),'.mat'),'itvs');
            save(strcat(obj.direc,'src/Classification/itvs/',obj.opt.date,'-',num2str(obj.opt.tripNo),'.mat'),'itvs_'); 

            IS = zeros(length(itvs(:,1)),length(obj.Interp_Smoothed(itvs(1,1),itvs(1,2),itvs(1,3))));
            IS_ = zeros(length(itvs_(:,1)),length(obj.Interp_Smoothed(itvs_(1,1),itvs_(1,2),itvs_(1,3))));
            EF = zeros(length(itvs(:,1)),length(obj.Statistical(itvs(1,1),itvs(1,2),itvs(1,3))));
            EF_ = zeros(length(itvs_(:,1)),length(obj.Statistical(itvs_(1,1),itvs_(1,2),itvs_(1,3))));

            for j = 1:length(itvs(:,1))
                IS(j,:) = obj.Interp_Smoothed(itvs(j,1),itvs(j,2),itvs(j,3));
                EF(j,:) = obj.Statistical(itvs(j,1),itvs(j,2),itvs(j,3));
            end

            for j = 1:length(itvs_(:,1))
                IS_(j,:) = obj.Interp_Smoothed(itvs_(j,1),itvs_(j,2),itvs_(j,3));
                EF_(j,:) = obj.Statistical(itvs_(j,1),itvs_(j,2),itvs_(j,3));
            end
            % if exist(strcat('D:/SJ_Dataset/MatlabFiles/EventDetection/TrajectoryBasedDetection/src/Detection/IS',obj.date,num2str(obj.trip),'.csv'),'file')
            %     delete(strcat('D:/SJ_Dataset/MatlabFiles/EventDetection/TrajectoryBasedDetection/src/Detection/IS',obj.date,num2str(obj.trip),'.csv'));
            %     disp(strcat("File 'IS",obj.date,num2str(obj.trip),".csv' already existed has been deleted"));
            % end

            writematrix(IS,strcat(obj.direc,'src/Detection/data/IS-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
            writematrix(EF,strcat(obj.direc,'src/Detection/data/EF-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
            writematrix(IS_,strcat(obj.direc,'src/Classification/data/IS-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
            writematrix(EF_,strcat(obj.direc,'src/Classification/data/EF-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
        end
        
        %% Data extraction
        function obj = extract2(obj)
            itvdir = strcat(obj.direc,'src/Detection/itvs/');
            filelist = dir(fullfile(itvdir, '*.mat'));
            for i = 1: length(filelist)
                filename = filelist(i).name;
                [date,tripNo] = obj.filenameReader(filename);
                if ispc
                    pathOpt = strcat("/SJ_Dataset/2023/",date,"/rlog/TripData/",num2str(tripNo),"/opts.mat");
                elseif isunix
                    pathOpt = strcat("/mnt/hdd1/SJ_Dataset/2023/",date,"/rlog/TripData/",num2str(tripNo),"/opts.mat");
                end
                obj.opt = load(pathOpt).opts;
                itvs = load(strcat(obj.direc,'src/Detection/itvs/',obj.opt.date,'-',num2str(obj.opt.tripNo),'.mat')).itvs;
                itvs_ = load(strcat(obj.direc,'src/Classification/itvs/',obj.opt.date,'-',num2str(obj.opt.tripNo),'.mat')).itvs_;
                IS = zeros(length(itvs(:,1)),length(obj.Interp_Smoothed(itvs(1,1),itvs(1,2),itvs(1,3))));
                IS_ = zeros(length(itvs_(:,1)),length(obj.Interp_Smoothed(itvs_(1,1),itvs_(1,2),itvs_(1,3))));
                EF = zeros(length(itvs(:,1)),length(obj.Statistical(itvs(1,1),itvs(1,2),itvs(1,3))));
                EF_ = zeros(length(itvs_(:,1)),length(obj.Statistical(itvs_(1,1),itvs_(1,2),itvs_(1,3))));

                for j = 1:length(itvs(:,1))
                    IS(j,:) = obj.Interp_Smoothed(itvs(j,1),itvs(j,2),itvs(j,3));
                    EF(j,:) = obj.Statistical(itvs(j,1),itvs(j,2),itvs(j,3));
                end
    
                for j = 1:length(itvs_(:,1))
                    IS_(j,:) = obj.Interp_Smoothed(itvs_(j,1),itvs_(j,2),itvs_(j,3));
                    EF_(j,:) = obj.Statistical(itvs_(j,1),itvs_(j,2),itvs_(j,3));
                end
                
                % 
                writematrix(IS,strcat(obj.direc,'src/Detection/data/IS-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
                writematrix(EF,strcat(obj.direc,'src/Detection/data/EF-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
                writematrix(IS_,strcat(obj.direc,'src/Classification/data/IS-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
                writematrix(EF_,strcat(obj.direc,'src/Classification/data/EF-',obj.opt.date,'-',num2str(obj.opt.tripNo),'.csv'));
            end
        end

        %% Train trajectory-based event classifier
        function obj = train(obj)
            % python script
            python_file = strcat(obj.direc,'RTC.py');

            key = input('Do you want to train the model with data except current trip? (Y/N):  ',"s");
                if strcmpi(key,"n")
                    system(['python',python_file]);
                else
                    testTrip = strcat(obj.opt.date,'-',num2str(obj.opt.tripNo));
                    command = sprintf('python %s --testTrip %s', python_file, testTrip);
                    system(command);
                end
        end

        %% Perform trajectory-based event classification
        function labeled_itvs = test(obj)
            [idxs,itvs] = obj.getArcsIntvs();
            type = "EF" ; % EF or IS
            if type == "IS"
                n = 100;
                features = zeros(length(idxs),length(obj.Interp_Smoothed(itvs(1,1),itvs(1,2),idxs(1))));
                for i = 1:length(idxs)
                    features(i,:) = obj.Interp_Smoothed(itvs(i,1),itvs(i,2),idxs(i));
                end
            elseif type == "EF"
                n = 10;
                features = zeros(length(idxs),length(obj.Statistical(itvs(1,1),itvs(1,2),idxs(1))));
                for i = 1:length(idxs)
                    features(i,:) = obj.Statistical(itvs(i,1),itvs(i,2),idxs(i));
                end
            end
            % Step 2 : Classification of Lane Keeping Manuever and Others
            warning('off','all');
            
            model1_path = '../model/train/Detection/model_CNN';
            model_CNN = importTensorFlowNetwork(model1_path,'OutputLayerType','classification','Classes',{'LKM','Event'},'Verbose',false);
            arc_idx = features(:,end);

            feature = features(:,1:end-1);
            LKM = [];
            for i = 1:length(feature(:,1))
                feature_ = reshape(feature(i,:),n,[]);
                if type == "IS"
                    Y = classify(model_CNN,feature_);
                elseif type == "EF"
                    Y = classify(model_CNN,feature_');
                    S = predict(model_CNN,feature_');
                    % disp(S)

                end
                if Y == "LKM"
                    LKM = [LKM,arc_idx(i)];
                end
            end
            comb = obj.possibleComb2(LKM);
            if ~isempty(comb)
                combIntvs = getCombIntvs(comb);
                combStateIntvs = obj.getCombStateIntvs(combIntvs);
                
                features = zeros(length(combStateIntvs(:,1)),length(obj.Interp_Smoothed(combStateIntvs(1,1),combStateIntvs(1,2),combStateIntvs(1,3:4))));
                for i = 1:length(combStateIntvs(:,1))
                    features(i,:) = obj.Interp_Smoothed(combStateIntvs(i,1),combStateIntvs(i,2),combStateIntvs(i,3:4));
                end
    
                % Step 3 : Classification of Others into {Right Turn, Left Turn, Right Lane Change, Left Lane Change, Roundabout}
                comb_label = features(:,end-1);
                
                section_label = features(:,end);
                feature = features(:,1:end-2);

                model2_path = '../model/train/Classification/model_CNN';
    
                model2_CNN = importTensorFlowNetwork(model2_path,'OutputLayerType','classification','Classes',{'RT','LT','RLC','LLC','RA'},'Verbose',false);
                table = [];
                for i = 1:length(feature(:,1))
                    feature_ = reshape(feature(i,:),100,[]);
                    score = predict(model2_CNN,feature_);
                    [M,I] = max(score);
                    table = [table;section_label(i),comb_label(i),M,I];
                end
                
                % Step 4 : Finding the best combination of arc segment
                % classification labels
                score = table(:,3);
                label = table(:,4);
                predicts = zeros(max(section_label),max(comb_label));
                for i = 1:length(section_label)
                    a = section_label(i); b = comb_label(i);
                    if predicts(a,b) == 0
                        predicts(a,b) = predicts(a,b)+score(i);
                    else
                        predicts(a,b) = predicts(a,b)*score(i);
                    end
                end
                [~, idx] = max(predicts, [], 2);
                eventIntvs = [];
                for i = 1:max(section_label)
                    for j = 1:length(combIntvs(:,1))
                        if combIntvs(j,3) == idx(i) && combIntvs(j,4) == i
                            eventIntvs = [eventIntvs;combIntvs(j,1),combIntvs(j,2),label(j)];
                        end
                    end
                end
            else
                eventIntvs = [min(idxs), max(idxs), 0];
            end
            
            % obj.SingleTripFit.visualize_predict(eventIntvs)
            
            labeled_itvs = obj.getLabeledItvs(itvs,eventIntvs);
            
        end

        %% Get all possible combination of events
        function possi_comb = possibleComb(obj)

            xyR = obj.SingleTripFit.getParams();
            EventRadiusIntvs = getIntvs(find(xyR(3,:) < 700));
            EventRadiusIntvs = EventRadiusIntvs';
            possi_comb = cell(1,length(EventRadiusIntvs(1,:)));

            for i = 1:length(EventRadiusIntvs(1,:))
                lb = EventRadiusIntvs(1,i);
                ub = EventRadiusIntvs(2,i);
                comb_n = 2^(ub-lb);
                comb = zeros(comb_n,ub-lb+1);
                comb(:,1) = lb;
                numbers = lb+1:ub;
                k = 2;

                for j = 1:ub-lb
                    nck = nchoosek(numbers,j);
                    shape = size(nck);
                    comb(k:k+shape(1)-1,2:2+shape(2)-1) = nck;
                    k = k+shape(1);
                end

                possi_comb{i} = comb;
            end
        end

        %% Get all possible combination of events
        function possi_comb = possibleComb2(obj,LKM)

            xyR = obj.SingleTripFit.getParams();
            % arcs = find(xyR(3,:) < 700);
            arcs = 1:size(xyR,2);
            arcs_e = arcs(~ismember(arcs,LKM));
            if ~isempty(arcs_e)
                EventRadiusIntvs = getIntvs(arcs_e);
    
                EventRadiusIntvs = EventRadiusIntvs';
                possi_comb = cell(1,length(EventRadiusIntvs(1,:)));
    
                for i = 1:length(EventRadiusIntvs(1,:))
                    lb = EventRadiusIntvs(1,i);
                    ub = EventRadiusIntvs(2,i);
                    comb_n = 2^(ub-lb);
                    comb = zeros(comb_n,ub-lb+1);
                    comb(:,1) = lb;
                    numbers = lb+1:ub;
                    k = 2;
    
                    for j = 1:ub-lb
                        nck = nchoosek(numbers,j);
                        shape = size(nck);
                        comb(k:k+shape(1)-1,2:2+shape(2)-1) = nck;
                        k = k+shape(1);
                    end
    
                    possi_comb{i} = comb;
                end
            else
                possi_comb = {};
            end
        end

        %% Get State Intervals for all possible combination
        function combStateIntvs = getCombStateIntvs(obj,intvs)
            StateIntvs = obj.SingleTripFit.params.intv;
            combStateIntvs = zeros(size(intvs));
            for i = 1:size(combStateIntvs,1)
                lb = intvs(i,1); ub = intvs(i,2);
                combStateIntvs(i,1) = StateIntvs(lb,1);
                combStateIntvs(i,2) = StateIntvs(ub,2);
                combStateIntvs(i,3:4) = intvs(i,3:4);
            end
        end

        %% Get all arcs
        function [idx,intvs] = getArcsIntvs(obj)
            xyR = obj.SingleTripFit.getParams();
            % idx = find(xyR(3,:) < 700);
            StateIntvs = obj.SingleTripFit.params.intv;
            % intvs = StateIntvs(idx,1:2);
            idx = 1:size(xyR,2);
            intvs = StateIntvs(:,1:2);
        end

        %% features / time series
        function oneline = Interp_Smoothed(obj,lb,ub,type)
            ha = obj.heading_angle(obj.opt,lb,ub);
            ha_ = obj.interpSmoothed(ha,100);
            yr = obj.yawrate(obj.opt,lb,ub);
            yr_ = obj.interpSmoothed(yr,100);
            [x,y] = obj.position(obj.opt,lb,ub);
            x_ = obj.interpSmoothed(x,100);
            y_ = obj.interpSmoothed(y,100);
            [ly,ry] = obj.lane(obj.opt,lb,ub);
            ly_ = obj.interpSmoothed(ly,100);
            ry_ = obj.interpSmoothed(ry,100);
            yy_ = ly_ - ry_;
            yyy_ = ly_ + ry_;
            [lystd,rystd] = obj.lanestd(obj.opt,lb,ub);
            lystd_ = obj.interpSmoothed(lystd,100);
            rystd_ = obj.interpSmoothed(rystd,100);
            oneline = [x_,y_,ha_,yr_,ly_,-ry_,lystd_,rystd_,type];  %%val
        end

        %% features / statistical
        function oneline = Statistical(obj,lb,ub,type)
            ha = obj.heading_angle(obj.opt,lb,ub);
            ha_ = obj.statistical(ha);
            yr = obj.yawrate(obj.opt,lb,ub);
            yr_ = obj.statistical(yr);
            [x,y] = obj.position(obj.opt,lb,ub);
            x_ = obj.statistical(x);
            y_ = obj.statistical(y);
            [ly,ry] = obj.lane(obj.opt,lb,ub);
            ly_ = obj.statistical(ly);
            ry_ = obj.statistical(ry);
            yy_ = ly_ - ry_;
            yyy_ = ly_ + ry_;
            [lystd,rystd] = obj.lanestd(obj.opt,lb,ub);
            lystd_ = obj.statistical(lystd);
            rystd_ = obj.statistical(rystd);
            oneline = [x_,y_,ha_,yr_,ly_,-ry_,lystd_,rystd_,type]; %%val     
        end

    end

    methods (Static)
        %% Merge intervals using detection/classification results
        function labeled_itvs = getLabeledItvs(state_itvs,event_itvs)
            
            labeled_itvs = [state_itvs, zeros(size(state_itvs,1),1)];
            for i=1:size(event_itvs,1)
                lb = event_itvs(i,1); ub = event_itvs(i,2);
                for j=lb:ub
                    labeled_itvs(j,3) = event_itvs(i,3);
                end 
            end
            cnt = 1;
            while true
                if cnt == size(labeled_itvs,1)
                    break;
                end

                if labeled_itvs(cnt,3) == labeled_itvs(cnt+1,3)
                    labeled_itvs(cnt,2) = labeled_itvs(cnt+1,2);
                    labeled_itvs(cnt+1,:) = [];
                else
                    cnt = cnt + 1;
                end
            end
        end
        
        %% Conti
        function itvs_ = conti(itvs)
            j = 1;
            itvs_(j,:) = itvs(1,:);
            for i = 2:length(itvs(:,1))
                if itvs(i-1,2) == itvs(i,1) && itvs(i-1,3) == itvs(i,3)
                    itvs_(j,2) = itvs(i,2);
                else
                    j = j+1;
                    itvs_(j,:) = itvs(i,:);
                end
            end
        end
        
        %% Linear Interpolation & Savitzky-Golay filter
        function data_out = interpSmoothed(data,m)
            n = length(data);
            if n>5
                smoothed_data = sgolayfilt(data,2,5);
            else
                smoothed_data = data;
            end
            data_out = interp1(linspace(0,1,n),smoothed_data,linspace(0,1,m));
        end
        
        %% heading angle
        function pts = heading_angle(opt,lb,ub)
            heading_angle = zeros(1,ub-lb);
            angle = 0;
            for i = lb+1:ub
                R_b = opt.states{i-1}.R;
                R = opt.states{i}.R;
                rpy = dcm2rpy(R'*R_b);
                dyaw = rpy(3)*180/pi;
                angle = angle+dyaw;
                heading_angle(i-lb) = angle;
            end
            pts = heading_angle;
        end

        %% yaw rate
        function pts = yawrate(opt,lb,ub)
            yawrate = zeros(1,ub-lb);
            for i = lb+1:ub
                R_b = opt.states{i-1}.R;
   
                t_b = opt.lane.t(opt.lane.state_idxs(i-1));
                R = opt.states{i}.R;
                t = opt.lane.t(opt.lane.state_idxs(i));
                rpy = dcm2rpy(R'*R_b);
                dyaw = rpy(3)*180/pi;
                dt = t-t_b;
                yawrate(i-lb) = dyaw/dt;
            end
            pts = yawrate;
        end

        %% position x,y
        function [x,y] = position(opt,lb,ub)
            position = zeros(2,ub-lb);
            pos = [0;0];
            for i = lb+1:ub
                P_b = opt.states{i-1}.P;
                P = opt.states{i}.P;
                dpos = P-P_b;
                dxy = dpos(1:2);
                pos = pos+dxy;
                position(:,i-lb) = pos;
            end
            data_pts = position./position(:,end);
            x = data_pts(1,:);
            y = data_pts(2,:);
        end

        %% lane distance
        function [ly,ry] = lane(opt,lb,ub)
            lane = zeros(2,ub-lb+1);
            for i = lb:ub
                ly = mean(opt.states{i}.left(2,1:3));
                ry = mean(opt.states{i}.right(2,1:3));
                lane(1,i-lb+1) = ly;
                lane(2,i-lb+1) = ry;
            end
            ly = lane(1,:); ry = lane(2,:);
        end

        %% lane distance std
        function [lystd,rystd] = lanestd(opt,lb,ub)
            lanestd = zeros(2,ub-lb+1);
            for i = lb:ub
                lanestd(1,i-lb+1) = opt.lane.lystd(opt.lane.state_idxs(i),1);
                lanestd(2,i-lb+1) = opt.lane.rystd(opt.lane.state_idxs(i),1);
            end
            lystd = lanestd(1,:); rystd = lanestd(2,:);
        end

        %% statistical features
        function features = statistical(data_pts)
            features = zeros(1,10);

            Mean = mean(data_pts);
            features(1) = Mean;
            
            Var = var(data_pts);
            features(2) = Var;
            
            peak = max(data_pts);
            features(3) = peak;
            
            rms = sqrt(mean(data_pts.^2));
            features(4) = rms;
            
            crest_factor = peak/rms;
            features(5) = crest_factor;
            
            shape_factor = rms/mean(abs(data_pts));
            features(6) = shape_factor;
            
            skewness = sum((data_pts-Mean).^3)/length(data_pts)/Var^(3/2);
            features(7) = skewness;
            
            smr = (sum(sqrt(abs(data_pts)))/length(data_pts))^2;
            features(8) = smr;
            
            pp = max(data_pts)-min(data_pts);
            features(9) = pp;

            % md = median(data_pts);
            % features(10) = md;

            sd = std(data_pts);
            features(10) = sd;
        end

        %% filename reader
        function [date,tripNo] = filenameReader(filename)
            matches1 = regexp(filename,'(\d{4}-\d{2}-\d{2})-(\d{1})\.mat', 'tokens');
            matches2 = regexp(filename,'(\d{4}-\d{2}-\d{2})-(\d{2})\.mat', 'tokens');
            if ~isempty(matches1)
                date = matches1{1}{1};
                date = string(date);
                tripNo = matches1{1}{2};
                tripNo = str2num(tripNo);
            elseif ~isempty(matches2)
                date = matches2{1}{1};
                date = string(date);
                tripNo = matches2{1}{2};
                tripNo = str2num(tripNo);
            end
        end
    end
end
