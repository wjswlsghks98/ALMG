clc; close all; clear;
data = load("output.csv");

label = data(:,end);
features = data(:,1:end-1);

LKM = features(label==0,:);
RT = features(label==1,:);
LT = features(label==2,:);
RLC = features(label==3,:);
LLC = features(label==4,:);
RA = features(label==5,:);

xlabels = {'Mean','Var','Peak','rms','Kurtosis','Crest fatcor', 'Impulse factor', 'Shape factor', 'Skewness', 'smr', 'pp'};
pldt = RT;
figure(1);
boxplot(pldt(:,1:11),'Labels',xlabels);hold on;
boxplot(LKM(:,1:11),'Labels',xlabels,'PlotStyle','compact')
title('position y');
drawnow;

figure(2);
boxplot(pldt(:,12:22),'Labels',xlabels);
title('heading angle');
drawnow;

figure(3);
boxplot(pldt(:,23:33),'Labels',xlabels);
title('yaw rate');

figure(4);
boxplot(pldt(:,34:44),'Labels',xlabels);
title('left lane distance');

figure(5);
boxplot(pldt(:,45:55),'Labels',xlabels);
title('right lane distance');

figure(6);
boxplot(pldt(:,56:66),'Labels',xlabels);
title('left lane std');

figure(7);
boxplot(pldt(:,67:77),'Labels',xlabels);
title('right lane std');

figure(8)
boxplot(LKM(:,1:11))