% Write all the code necessary to run and test your controller(s) in this file
% Code should be in ready-to-execute format

% All the code related to generating plots and verifying your requirement
% should also be included here

% Sample code
launch_angle = 10; % launch angle in degrees
elevator_len = 0.1; % length of elevator. Must be between 0 and 0.2
DesignProblem03('Controller', 'launchangle', deg2rad(launch_angle), 'elevatorlen', elevator_len, 'datafile', 'data.mat')

syms theta phi xdot ydot thetadot phidot
load('DesignProblem03_EOMs.mat') % load equations of motion

f = symEOM.f;
state = [theta phi xdot ydot thetadot]; % define state




launch_angle = 30; % launch angle in degrees
elevator_len = 0.11; % length of elevator. Must be between 0 and 0.2

n=1000;
x=zeros(n,1);
t=zeros(n,1);

for i=1:n
    DesignProblem03('Controller','launchangle', deg2rad(launch_angle), 'elevatorlen', elevator_len,'datafile','data.mat','display',false);
    load('data.mat');
    x(i)= processdata.x(end);
    t(i)= processdata.t(end);
    fprintf('%d out of %d tests done.\n', i, n);
    figure(2)
    plot(processdata.x,processdata.y)
    grid on
    hold on
    xlabel('Range')
    ylabel('Height')
    title('Flight Paths')
end

figure(3)
histogram(x,30)
edges = linspace(0,30,61)
min_d = min(x)
max_d = max(x)
mean_d = mean(x)
med_d = median(x)
std_d = std(x)
minlabel=sprintf('Min -- %3.2d', min_d);
title('Histogram of 1000 Controlled Flight Simulations');
xlabel('Distance (m)') 
ylabel('Number of Flights') 
maxlabel=sprintf('Max -- %3.2d', max_d);
mnlabel=sprintf('Mean -- %3.2d', mean_d);
mdlabel=sprintf('Median -- %3.2d', med_d);
stdlabel=sprintf('Std Deviation -- %3.2d', std_d);

h=annotation('textbox',[0.58 0.75 0.1 0.1]);
set(h,'String',{minlabel, maxlabel,mnlabel, mdlabel, stdlabel});


X=sum(x);
T=sum(t);
t_avg= T/length(t);

x_max=max(x)
x_avg= mean(x)
x_med= median(x)
s=std(x)








  








