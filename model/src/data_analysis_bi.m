clc; close all; clear;
data = load("output_bi.csv");

label = data(:,end);
features = data(:,1:end-1);

LKM = features(label==0,:);
Event = features(label==1,:);

flabels = {'Mean','Var','Peak','RMS','Kurtosis','Crest Fatcor', 'Impulse Factor', 'Shape Factor', 'Skewness', 'SMR', 'Peak-Peak'};
feature = 1;
dylabels = {'y_L', 'y_E', 'ha_L', 'ha_E', 'r_L', 'r_E', 'ly_L', 'ly_E', 'ry_L','ry_E', 'lstd_L', 'lstd_E', 'rstd_L', 'rstd_E'};
% for i = 1:11
%     figure(i);
%     datacomb = [LKM(:,i)' Event(:,i)' LKM(:,11+i)' Event(:,11+i)' LKM(:,22+i)' Event(:,22+i)' LKM(:,33+i)' Event(:,33+i)' ...
%         LKM(:,44+i)' Event(:,44+i)' LKM(:,55+i)' Event(:,55+i)' LKM(:,66+i)' Event(:,66+i)'];
%     groupIdx = [repmat(dylabels(1),length(LKM(:,1)),1);repmat(dylabels(1),length(Event(:,1)),1); ...
%         repmat(dylabels(1),length(LKM(:,1)),1);repmat(dylabels(2),length(Event(:,1)),1); ...
%         repmat(dylabels(3),length(LKM(:,1)),1);repmat(dylabels(4),length(Event(:,1)),1); ...
%         repmat(dylabels(5),length(LKM(:,1)),1);repmat(dylabels(6),length(Event(:,1)),1); ...
%         repmat(dylabels(7),length(LKM(:,1)),1);repmat(dylabels(8),length(Event(:,1)),1); ...
%         repmat(dylabels(9),length(LKM(:,1)),1);repmat(dylabels(10),length(Event(:,1)),1); ...
%         repmat(dylabels(11),length(LKM(:,1)),1);repmat(dylabels(12),length(Event(:,1)),1);];
%     boxplot(datacomb,groupIdx);
%     title(flabels{i});
%     drawnow;
% end
for i = 1:11
    for j = 2
        figure(i*10+j);
        dataComb = [LKM(:,11*(j-1)+i)' Event(:,11*(j-1)+i)'];
        groupIdx = [repmat(dylabels(2*j-1),length(LKM(:,11*(j-1)+i)),1);repmat(dylabels(2*j),length(Event(:,11*(j-1)+i)),1)];
        boxplot(dataComb,groupIdx);
        title(flabels{i});
        drawnow;
    end
end