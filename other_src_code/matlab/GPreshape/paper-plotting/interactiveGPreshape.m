function [  ] = interactiveGPreshape(varargin)

global originalGMM originalDynamics;

% parse arguments
opt = {};
opt.figNb = 1;
if(size(varargin,2)>0)
    opt = parseArguments(varargin);
else
    opt = parseArguments({'figure',1,'dynamics','linear'});
end

% define original dynamics
if(strcmp(opt.dynamics,'linear'));
    originalDynamics = @(x) originalDynamicsLINEAR(x);
else
    load(opt.dynamics);
    %load GShape;
    originalGMM.Mu = Mu;
    originalGMM.Priors = Priors;
    originalGMM.Sigma = Sigma;
    originalDynamics = @(x) originalDynamicsGMR(x);
end

% Check namespace for GP-ML and Kalman filter
if(~exist('gp'))
    disp 'Setting up GPML settings';
    run('gpml-matlab-v3.4-2013-11-11/startup.m')
end
if(~exist('EstimateVA_P'))
    disp 'Adding KalmanTools/ to path';
    addpath('KalmanTools/')
end
   

close all
figure(opt.figNb)
set(gcf,'UserData',[])
set(gcf,'WindowButtonDownFcn',[])
set(gcf,'WindowButtonUpFcn',[]);
set(gcf,'WindowButtonMotionFcn',[]);

nX = 100;
x = linspace(-200,200,nX);
y = linspace(-200,200,nX);
[xM, yM] = meshgrid(x,y);
X = [xM(:)';yM(:)'];
Xd = originalDynamics(X);
figure(opt.figNb);clf;hold on
hs = streamslice(xM,yM,reshape(Xd(1,:),nX,nX),reshape(Xd(2,:),nX,nX),0.5);
hi=plot(0,0);
%prepare a place where the functions can put data 
s = {};
gridData.xM = xM;
gridData.yM = yM;
gridData.X = X;
gridData.nX = nX;
s.gridData = gridData;
s.streamHandle = hs;
s.inflHandle = hi;
s.allData = [];
s.gpData = [];
s.newPosData = [];
s.drawingDt = 0.01;
s.breakSimulation = 0;
s.opt = opt;


% parameters for GP regression
s.gprStruct.meanfunc = {@meanZero};
s.gprStruct.covfunc = {@covSEiso}; 
ell = 30;   % INTERESTING PARAMTER TO PLAY WITH
sf = 1;     % INTERESTING PARAMTER TO PLAY WITH 
s.gprStruct.hyp.cov = log([ell; sf]);
s.gprStruct.likfunc = @likGauss; 
sn = 01.0;    % INTERESTING PARAMTER TO PLAY WITH
s.gprStruct.hyp.lik = log(sn);
% store a handle to the regressionfunction for use in various functions
s.gprStruct.regressionFunction = @(x_train,y_train,x_query) gp(s.gprStruct.hyp, @infExact, s.gprStruct.meanfunc, s.gprStruct.covfunc, s.gprStruct.likfunc, x_train, y_train, x_query);

% the s structure is just an ugly hack for conveniently sharing some data
% between different functions. s is stores as "UserData" in the figure
set(gcf,'UserData',s)

% prepare for interaction
set(gcf, 'WindowButtonDownFcn', @(h,e)buttonClicked(h,e,varargin));

% Secondary figure is the training data. ** Not operational. ** (due to the
% way data is stored in the figure, we cannot have two figures open).
if(0 && s.opt.global_demonstrations == 1)
    demonstrations_fig = figure('name', 'Demonstrations');
    set(demonstrations_fig, 'WindowButtonDownFcn', @(h,e)demonstrationsCallback(h,e,varargin));
    axes()
    setup_global_demo_figure(demonstrations_fig, originalDynamics)
end
end

function setup_global_demo_figure(figureHandle, dynamics)

    ax = findall(figureHandle,'type','axes');

    nX = 100;
    x = linspace(-200,200,nX);
    y = linspace(-200,200,nX);
    [xM, yM] = meshgrid(x,y);
    X = [xM(:)';yM(:)'];
    Xd = dynamics(X);
    hs = streamslice(ax, xM,yM,reshape(Xd(1,:),nX,nX),reshape(Xd(2,:),nX,nX),0.5);
end

function [xd jac] = originalDynamicsGMR(x)
global originalGMM;
xd = GMR(originalGMM.Priors,originalGMM.Mu,originalGMM.Sigma,x,1:2,3:4 );
jac = eye(2);
end


function Xd = reshapedDynamics(x)
global originalDynamics;
s = get(gcf,'UserData');

if(size(s.gpData, 1) < 4)
    Xd = originalDynamics(x);
    return
end

angleHat = s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(3,:)',x');
speedHat = s.gprStruct.regressionFunction(s.gpData(1:2,:)', s.gpData(4,:)',x');
speedHat = max(speedHat, -0.9);
%speedHat = customLogistic(speedHat,-1,20);
Xd = originalDynamics(x);
Xd = locallyRotateV(Xd,angleHat,speedHat);
end


function ret = buttonClicked(h, e, args)
disp(['Click callback on main figure: ', get(gcf,'selectiontype')])

s=get(gcf,'UserData');
s.breakSimulation = 1;
set(gcf,'UserData',s);
if(strcmp(get(gcf,'SelectionType'),'normal'))
    x = get(gca,'Currentpoint');
    x = x(1,1:2)';
    s=get(gcf,'UserData');
    s.breakSimulation =0;
    set(gcf,'UserData',s);
    startSimulation(x);
elseif(strcmp(get(gcf,'SelectionType'),'alt'))
    startDemonstration();
elseif(strcmp(get(gcf,'SelectionType'),'extend'))
    interactiveGPreshape(args{:});
elseif(strcmp(get(gcf, 'SelectionType'), 'open'))
    x = get(gca,'Currentpoint');
    x = x(1,1:2)';
    loadStoredPoints(h, x)
end
end

function ret = startDemonstration()
% disp('started demonstration')
set(gcf,'WindowButtonUpFcn',@stopDemonstration);
set(gcf,'WindowButtonMotionFcn',@recordPoint);
tic
end

function ret = stopDemonstration(h,e)
% disp('stopped demonstration')
set(gcf,'WindowButtonMotionFcn',[]);
processNewData();
updateStreamlines(h);
set(gcf,'WindowButtonUpFcn',[]);
end

function ret = loadStoredPoints(figureHandle, offset)
% Reproduces the stored training data around the double-clicked location
% (from user input). The stored data is in gpData.mat (nominally these are
% the last-added demonstration points). This calls processNewData() and
% updateStreamLines().

stored_data = load('gpData.mat');
tmp = stored_data.saved_data;
points = tmp(1:2, :);  % Select out the x/y coordinates, remove velocity.
num_pts = size(tmp, 2);
fprintf(1, 'Loading %d points and applying offset: %f %f\n', num_pts, offset)

% Take the offset and add it to the poses.
offset_mat = repmat(offset, 1, num_pts);
new_points = points + offset_mat;

% Set new position data
s = get(gcf,'UserData');
s.newPosData = new_points;
set(gcf,'UserData',s);
processNewData();  
updateStreamlines(figureHandle);
end

function ret = recordPoint(h,e)
s =get(gcf,'UserData');
x = get(gca,'Currentpoint');
x = x(1,1:2)';
s.newPosData = [s.newPosData,x];
set(gcf,'UserData',s);
plot(x(1),x(2),'r.')
end

function ret = updateStreamlines(figureHandle)
s = get(gcf,'UserData');

% If not enough data, skip updating stream lines.
if(size(s.gpData, 1) < 4)
    return
end

Xd = reshapedDynamics(s.gridData.X);
% plot
clf; hold on;

% plot the shaded influence region
infl =  s.gprStruct.regressionFunction(s.gpData(1:2,:)',ones(size(s.gpData(3,:)')),s.gridData.X');
load whiteCopperColorMap;

axisHandle = findall(figureHandle,'type','axes');

s.inflHandle=pcolor(axisHandle, s.gridData.xM,s.gridData.yM,reshape(infl,s.gridData.nX,s.gridData.nX));
set(s.inflHandle,'linestyle','none');
colormap(cm);


s.streamHandle = streamslice(axisHandle, s.gridData.xM,s.gridData.yM,reshape(Xd(1,:),s.gridData.nX,s.gridData.nX),reshape(Xd(2,:),s.gridData.nX,s.gridData.nX),0.5);
% plot collected points

%plot(axisHandle, s.allData(1,:),s.allData(2,:),'r.');
% plot training points
plot(axisHandle, s.gpData(1,:),s.gpData(2,:),'k', 'Linewidth', 5);
plot(axisHandle, s.gpData(1,end), s.gpData(2,end), 'k^-', 'Markersize', 10, ...
'MarkerFaceColor', 'black') 

set(gcf,'UserData',s);
%x=[0;0];
%[orgVel orgJac]=originalDynamics(x);
%J=compute2dGPLMDSjacobian(s.gprStruct,s.gpData,orgVel,orgJac,x);
%eig(J)

end

function ret= processNewData()
global originalDynamics;
s = get(gcf,'UserData');

if(size(s.newPosData,2)<10)
    return
end
disp('processing the new data')

% kalman filter initial guesstimate covariance matrix
P0 = zeros(6,6);
P0(1,1) = 1;
P0(2,2) = 1;
P0(3,3) = 100;
P0(4,4) = 100;
P0(5,5) = 100;
P0(6,6) = 100;

% Kalman smoothing of the demonstration: the resulting poses are not
% exactly the same as the demonstration points.
X = s.newPosData(1:2,:);
dt = s.drawingDt;
Q = 1e2*ones(2,1);
R = 0.001*ones(2,1);
xi0 = [s.newPosData(1:2,1);zeros(4,1)];
[demPos,demVel, demAcc] = EstimateVA_P(X, dt, Q, R, xi0, P0);
% Compute the angle and speed factor. newData is a 4xN matrix with each
% column: [x; y; theta; velocity].
newData = computeLMDSdata2D(demPos,demVel,originalDynamics(demPos));

% make sure there is at least one datapoint for the gp
if(size(s.gpData,2)==0)
    s.gpData=newData(:,1);
    % need to write it back here for reshapedDynamics function
    set(gcf,'UserData',s);
end
%plot(demPos(1,:),demPos(2,:),'ro')
% select data
nNewData = size(newData,2);
newData(3,:);

% Figure out which of the 'new' demonstrated data points to store as
% training data. Predict the velocity and angle using the existing data,
% and only store those where the predicted velocity or angle are different
% enough from the training data. This subsamples the data points in an
% online fashion (sequential inspection).
for n=1:nNewData
    % Two independent GP regression problems: angle and velocity at the new
    % data point (nth point), given the training data seen so far.
    %   Input to regressionFunction: x_train,y_train,x_query
    [angleHat s2]=s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(3,:)',newData(1:2,n)');
    [speedHat s2]=s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(4,:)',newData(1:2,n)');
    %xd = reshapedDynamics(demPos(:,n));
    
    % Velocity and angle (predicted vs actual) comparison.
    if ( abs((speedHat - newData(4,n))/newData(4,n)) > 0.3 || abs((angleHat - newData(3,n))) > 0.3    )%( norm(xd - demVel(:,n))/norm(demVel(:,n)) > 1.5)%abs((a-newData(3,n))/newData(3,n))>0.3)%s2>0.8)
        s.gpData = [s.gpData,newData(:,n)];  % Append data
        % need to write it back here for reshapedDynamics function
        set(gcf,'UserData',s);
    else
        kuk=1;  % No-op.
    end 
end

% Save all new data (from the current demonstration) to file.
saved_data = newData;
fname ='gpData.mat';
save(fname, 'saved_data');
fprintf(1, 'Saved %d data points in %s\n', size(saved_data, 2), fname);

% Now we have both gpData (training data points) and allData (all data
% points).
s.allData = [s.allData,newData];
s.newPosData = [];
set(gcf,'UserData',s);

%figure(3);clf; plot(s.gpData(3,:),'k.')
end

function r = startSimulation(x)
s = get(gcf,'UserData');
disp('starting simulation')
xd = 0;
dt = s.drawingDt;
t=0;
breakSimulation = 0;
hm=plot(0,0);
while ~breakSimulation
   xd = reshapedDynamics(x);
   x = x+dt*xd;
   t=t+dt;
   %h = findobj(gcf,'color',[1 0 1]);
   %delete(h);
   %hm=plot(x(1),x(2),'mx','linewidth',3,'markersize',10);
   plot(x(1),x(2),'ro', 'MarkerFacecolor', 'red');
   pause(dt)
   if(norm(x)<5)
       break;
   end
   b=get(gcf,'UserData');
   breakSimulation = b.breakSimulation;
end
end

function opt = parseArguments(args)
    opt = {};
    opt.figNb = 1;  % default values
    opt.dynamics = 'linear';
    opt.global_demonstrations = 0;
    for i=1:size(args,2)
       if strcmp(args{i},'figure')
           opt.figNb = args{i+1};
       elseif strcmp(args{i},'dynamics')
           opt.dynamics = args{i+1};
       elseif strcmp(args{i},'global_demonstrations')
           opt.global_demonstrations = str2num(args{i+1});
       end
    end
end


function ret = demonstrationsCallback(h, e, args)
    disp(['Click callback on global demonstrations: ', get(gcf, 'selectiontype')])
    updateStreamlines(h)
end
