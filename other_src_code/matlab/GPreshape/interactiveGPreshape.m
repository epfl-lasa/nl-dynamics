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
s.trianglesData = [];
s.spline = [];
s.smooth = 1/1; %cut every 1/smoothparam
s.smooth2 = 5; %check that the distance between point is bigger than smooth2
s.xs = 0;
s.ys = 0;
s.allAngle = [];
s.allAngleHat = [];
s.allVelocity=[];
s.allt = [];
s.allX=[];

% parameters for GP regression
s.gprStruct.meanfunc = {@meanZero};
s.gprStruct.covfunc = {@covSEiso}; 
%ell = 5;   % INTERESTING PARAMTER TO PLAY WITH
ell = 1;
sf = 1;     % INTERESTING PARAMTER TO PLAY WITH 
s.gprStruct.hyp.cov = log([ell; sf]);
s.gprStruct.likfunc = @likGauss; 
sn = 0.0002;    % INTERESTING PARAMTER TO PLAY WITH
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
xd = GMR(originalGMM.Priors,originalGMM.Mu,originalGMM.Sigma,x,1:2,3:4 )
jac = eye(2);
end

function Xd = reshapedDynamics(x)
    global originalDynamics;
    s = get(gcf,'UserData');

    if(size(s.gpData, 1) < 4 || size(x,2) > 1)
        Xd = originalDynamics(x);
        return
    end

    % angleHat = s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(3,:)',x');
    % speedHat = s.gprStruct.regressionFunction(s.gpData(1:2,:)', s.gpData(4,:)',x');
    
    t_close = minimumDistance2Spline(x(1,1), x(2,1), s.spline(1,:));
    i_close = 1;
    for i=2:size(s.spline,1)
        t_new = minimumDistance2Spline(x(1,1), x(2,1), s.spline(i,:));
        
        d1 = dist(getPointsSplineNO(s.spline(i_close,1), t_close), getPointsSplineNO(s.spline(i_close,2), t_close), x(1), x(2));
        d2 = dist(getPointsSplineNO(s.spline(i,1), t_new), getPointsSplineNO(s.spline(i,2), t_new), x(1), x(2));
        
        if d2<d1
            t_close = t_new;
            i_close = i;
        end
    end
    
    if t_close ~= -1 %-1 means that the closest point is not on the curve
        
        %add comment everywhere
        xy_close = [getPointsSplineNO(s.spline(i_close,1), t_close); getPointsSplineNO(s.spline(i_close,2), t_close)];
        xy_vel_close = [getVelSplineNO(s.spline(i_close,1), t_close); getVelSplineNO(s.spline(i_close,2), t_close)];
        xy_org_close = originalDynamics(xy_close);
        training_angle = atan2 (xy_vel_close(2), xy_vel_close(1)) - atan2(xy_org_close(2), xy_org_close(1) );

        if(training_angle > pi)
            training_angle = training_angle-2*pi;
        elseif(training_angle < -pi)
            training_angle = training_angle+2*pi;
        end

        training_speed = norm(xy_vel_close)/norm(xy_org_close)-1;
        
        s.allAngle = [s.allAngle, training_angle]; 
        s.allVelocity = [s.allVelocity, training_speed];
        s.allt = [s.allt, t_close];
        
        angleHat = s.gprStruct.regressionFunction(xy_close, training_angle, x);
        speedHat = s.gprStruct.regressionFunction(xy_close, training_speed, x);

        s.allAngleHat = [s.allAngleHat, angleHat];
        
        set(gcf, 'UserData', s);
        
        speedHat = max(speedHat, -0.9);
        %speedHat = customLogistic(speedHat,-1,20);
        Xd = originalDynamics(x);
        Xd = locallyRotateV(Xd,angleHat,speedHat);
        
        
    else
        Xd = originalDynamics(x);
        return
    end
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
%     x = get(gca,'Currentpoint');
%     x = x(1,1:2)';
%     loadStoredPoints(h, x)
    x = get(gca,'Currentpoint');
    s.xs = x(1,1);
    s.ys = x(1,2);
    set(gcf, 'UserData', s);
    updateStreamlines(h);
end
end

%only for debugging, to be deleated...
function plotall()
     s = get(gcf, 'UserData');
     
     t = linspace (0, 1, 1000);
     angle = 180/pi*atan2( getVelSplineNO(s.spline(end,2), t), getVelSplineNO(s.spline(end,1), t));
     
     figure
     plot( t, angle);
%     
%     t = linspace ( 0,1,1000);
%     i = 1:1000;
%     
%     splineVel = [getVelSplineNO(s.spline(1,1), t); getVelSplineNO(s.spline(1,2), t)];
%     
%     a = s.spline.points;
%     tx = [a(1,:), 1];
%     ix = 1:size(tx,2);
%     
%     pointvel = [getVelSplineNO(s.spline(1,1), tx); getVelSplineNO(s.spline(1,2), tx)];
%     
%     figure
%     plot (i, 180/pi.*atan2(splineVel(2,i), splineVel(1,i)));
    %hold on
    %plot (ix, 180/pi.*atan2(pointvel(2,ix), pointvel(1,ix)), 'ro');

%     t = 1:size(s.allAngle, 2);
%     
%     figure
%     plot(t, s.allAngle, 'b', t, s.allVelocity, 'r');
%     legend('angle', 'anglehat');
%     
%     figure
%     plot(t, s.allVelocity);
%     legend('speed')
    
%     figure
%     plot(t, s.allt);
%     legend('t_close')

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

% % plot collected points (desactivated because spline are plot now)
% plot(axisHandle, s.allData(1,:),s.allData(2,:),'r.');
% % plot training points
% plot(axisHandle, s.gpData(1,:),s.gpData(2,:),'go');

%drawing triangles
% for i=1:2:size(s.trianglesData, 1)
%     drawTriangle(s.trianglesData(i,:), 'm');
%     drawTriangle(s.trianglesData(i+1,:), 'k');
% end

%drawing points in green
plot(s.allData(1,:), s.allData(2,:), 'go')
% t=smoothData(size(s.allData, 2), s.smooth);
% plot(s.allData(1,t), s.allData(2,t), 'bo','LineWidth', 1.3);

%drawing spline
for i=1:size(s.spline)

%     t=linspace(-0.003,1.003,s.spline(i,1).nbPoint*100);
%     plot(getPointsSplineNO(s.spline(i,1), t), getPointsSplineNO(s.spline(i,2), t), 'r-', 'LineWidth', 1.5);

    for j=1:s.spline(1).nbPoint
        t=linspace(s.spline(1).points(1,j),    s.spline(1).points(2,j)    ,100);
        plot(getPointsSplineNO(s.spline(i,1), t), getPointsSplineNO(s.spline(i,2), t), 'r-', 'LineWidth', 1.5);
    end

end

% 
%     t = linspace(0,1,s.spline(i,1).nbPoint);
%     splineVel = [getVelSplineNO(s.spline(1,1), t); getVelSplineNO(s.spline(1,2), t)];
%     splinePos = [getPointsSplineNO(s.spline(1,1), t); getPointsSplineNO(s.spline(1,2), t)];
%     angle = [];
    
% for i=1:size(t,2)
%     norma = norm(splineVel(:,i));
%     
%    splineVel(1,i) = splineVel(1,i)/norma * 20;
%    splineVel(2,i) = splineVel(2,i)/norma * 20;
%    
%    %plot( [splinePos(1,i), splinePos(1,i)+splineVel(1,i)], [splinePos(2,i), splinePos(2,i)+splineVel(2,i)], 'k')
%    
%    a = atan2(splineVel(2,i), splineVel(1,i));
%    angle(i) = a;
%    a = a-pi/2;
%    
%    %plot( [splinePos(1,i), splinePos(1,i)+20*cos(a)], [splinePos(2,i), splinePos(2,i)+20*sin(a)], 'k')
%    
% end

% show triangle on the testing points and closest point
% for i=1:size(s.spline, 1)
%     %draw test points
%     drawTriangle([s.xs,s.ys],'g');
%     
%     %get closest t_point of the spline
%     tic
%     t_close = minimumDistance2Spline(s.xs,s.ys, s.spline(i,:))
%     toc
%     
%     %draw closest point
%     if (t_close ~= -1)
%         if t_close > 1
%             t = linspace(1,t_close, 100);
%             plot(getPointsSplineNO(s.spline(i,1), t), getPointsSplineNO(s.spline(i,2), t), 'r:', 'lineWidth', 2)
%         elseif t_close < 0
%             t = linspace(0,t_close, 100);
%             plot(getPointsSplineNO(s.spline(i,1), t), getPointsSplineNO(s.spline(i,2), t), 'r:', 'lineWidth', 2)
%         end
%         
%         drawTriangle([ getPointsSplineNO(s.spline(i,1), t_close ), getPointsSplineNO(s.spline(i,2), t_close ) ], 'k');
%         
%         %x = getPointsSplineNO(s.spline(i,1), t_close );
%         %y = getPointsSplineNO(s.spline(i,2), t_close );
%         dxytemp= [getVelSplineNO(s.spline(1,1), t_close); getVelSplineNO(s.spline(1,2), t_close)];
%         a = atan2 (dxytemp(2), dxytemp(1));
%             
%         if(a > pi)
%             a = a-2*pi;
%         elseif(a < -pi)
%             a = a+2*pi;
%         end
%         180 * a / pi;
%         %plot([x+10, x+10+dxytemp(1)], [y, y+dxytemp(2)], 'k')
%               
%     end
%     
% end


set(gcf,'UserData',s);
%x=[0;0];
%[orgVel orgJac]=originalDynamics(x);
%J=compute2dGPLMDSjacobian(s.gprStruct,s.gpData,orgVel,orgJac,x);
%eig(J)'

% plotall();
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

fig = get(groot,'CurrentFigure');
% analyzeError(demPos);
% set(groot,'CurrentFigure', fig);
% s.trianglesData = [s.trianglesData; analyzeData(demPos, demVel)];
s.spline = [s.spline ; computeTrajectory(demPos, demVel, s.smooth, s.smooth2)];

% % % % xx = [150.542531883059,148.674515182304,146.765175268026,144.788973945182,142.723948171580,140.553421122646,138.267318733695,135.862865842712,133.344562620784,130.723689056222,128.017468726738,125.247680523913,122.438866739243,119.616662542623,116.806478592317,114.032499348411,111.316715800404,108.677723033327,106.129739957343,103.682365872497,101.340972992802,99.1077239086036];
% % % % yy = [134.684560314771,120.084961801160,105.406630901772,90.6076621522382,75.6629923754485,60.5703012506297,45.3529158678522,30.0591015034953,14.7587336286611,-0.460310308876327,-15.4958185986607,-30.2365081859360,-44.5685792094079,-58.3816097815843,-71.5730061752909,-84.0512777941971,-95.7384740170925,-106.572378340245,-116.508053104726,-125.517248556069,-133.585387795046,-140.706991904418];
% % % % dx = [0.714386710353913,0.763413503212703,0.817227574364949,0.876878688112854,0.943492441844645,1.01810688309406,1.10143977861684,1.19359636466952,1.29376494043671,1.40001454056964,1.50933909379580,1.61803642348203,1.72235721108707,1.81914965767884,1.90624575987235,1.98250584726052,2.04758700470593,2.10161599106964,2.14491746858475,2.17775242785712,2.20006171657981,2.21121336357146];
% % % % dy = [0.818072071765488,0.929138805740384,1.05548448981490,1.19611749290643,1.34761589479246,1.50356940409661,1.65430654520888,1.78728281424246,1.88814951040139,1.94273797089204,1.94042322385556,1.87758952808783,1.75887234421553,1.59534168620222,1.40109163451429,1.18989323853515,0.973199164907620,0.759585844934236,0.554825501647990,0.362136229923608,0.182705666031401,0.0161953355580271];
% % % % 
% % % % s.allData = [xx;yy;dx;dy];
% % % % 
% % % % s.spline = [s.spline ; computeTrajectory([xx;yy], [dx;dy], s.smooth, s.smooth2)];

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

function ret = smoothData(nbPoint, alpha)
% return a vector of the index of the new point 
    if (nbPoint*alpha<2)
        ret=[1,nbPoint];
    else
        ret=round(linspace(1,nbPoint,nbPoint*alpha));
    end
end

function ret=advancedSmoothData(x,y,alpha2)
    xx=[x(1)];
    yy=[y(1)];
    k = 1;
    ret = [1];
    
    for i=1:size(x,1)-1
        goodpoint = 0;
        
        for j= ( (i+1):(size(x,1)-1) )
            if dist(xx(k), yy(k), x(j), y(j)) < alpha2
                goodpoint = j;
            end
        end
        
        if goodpoint > 0
            k=k+1;
            xx(k) = x(goodpoint);
            yy(k) = y(goodpoint);
            ret = [ret, goodpoint];
        end
    end
    
	ret = [ret, size(x, 1)];
    
end

function splinesData = computeTrajectory(pos, vel, alpha1, alpha2)
    %put originalDynamic velocity on first and last points
    %vel(:,1)  = originalDynamicsLINEAR(pos(:,1));
    %vel(:,end)= originalDynamicsLINEAR(pos(:,end));

    %smooth data
    values=smoothData(size(pos,2), alpha1);
    
    %values = [1, round(size(pos,2)/2), size(pos,2)];
    
    x =transpose(pos(1,values));
    y =transpose(pos(2,values));
    xp=transpose(vel(1,values));
    yp=transpose(vel(2,values));
    
    %2nd smoothing
    %values=advancedSmoothData(x,y,alpha2);
    
    x =transpose(pos(1,values));
    y =transpose(pos(2,values));
    xp=transpose(vel(1,values));
    yp=transpose(vel(2,values));
    
    % INSERTED BY KLAS: %
    % less confusing parameterization
    t = transpose(linspace(0,1,size(x,1)));
    % replace tx AND ty by t below and replace ttx and tty in plot function
    % by linspace(0,1,nbPoints)
    % END KLAS%
    
    %spline data calculation
    splinesData=[ownSpline(t,x,xp), ownSpline(t,y,yp)];

%     figure
%     t = linspace(0,1,1000);
%     plot(t,getPointsSplineNO(splinesData(1), t), 'r')
%     figure
%     plot(t,getPointsSplineNO(splinesData(2), t), 'b');
%     
%     pause
end

function start_stop=analyzeData (pos, vel)
    precisionFactor = 0.15; %percent
    s = get(gcf,'UserData');
    
    %vectors from point to the attractor following dynamics direction
    originalDS=[];
    for i=1:size(pos,2)
        [temp1, temp2] = originalDynamicsLINEAR(pos(:,i));
        originalDS=[originalDS, temp1];
    end
    
    %normalization of the vectors in the matrix
    %this could be done in only two lines with matlab 2015a and normc
    for i = 1:size(pos,2)-1
        vel(:,i) = vel(:,i) / norm(vel(:,i));
        originalDS(:,i) = originalDS(:,i) / norm(originalDS(:,i));
    end
    
    %comparison with dotproduct
    dotProd = dot(originalDS, vel);
    
    %verifying all the starts-and-stops
    isIn=false;
    count=0;
    start_stop=[];
    for i = 1:size(pos,2)-1
        
        if ~isIn && dotProd(i)<1-precisionFactor
            start_stop = [start_stop; [pos(1,i), pos(2,i)] ];
            isIn = true;
            count=count+1;
            
        elseif isIn && dotProd(i)>=1-precisionFactor
            start_stop = [start_stop; [pos(1,i), pos(2,i)] ];
            isIn = false;
            
            %for only one start-and-stop uncomment the next line
            %break
        end
    end
    
    %in case of there weren't the last stop
    if isIn
        start_stop= [start_stop; [pos(1,size(pos,2)-1), pos(2, size(pos,2)-1)]];
    end
end

function drawTriangle(pos, color)
    %drawing a triangle around the point
    halfLength=5;
    a=[pos(1)-halfLength,pos(1)+halfLength,pos(1),pos(1)-halfLength];
    b=[pos(2)-halfLength,pos(2)-halfLength,pos(2)+halfLength,pos(2)-halfLength];
    plot(a,b,color, 'LineWidth', 2)

    %having the color's point changed
    plot(pos(1),pos(2),strcat(color, '.'))

end

function r = startSimulation(x)
s = get(gcf,'UserData');
disp('starting simulation')
xd = 0;
dt = s.drawingDt/10;
t=0;
breakSimulation = 0;
hm=plot(0,0);

nb = 0;

while ~breakSimulation
   xd = reshapedDynamics(x);
   x = x+dt*xd;
   t=t+dt;
   h = findobj(gcf,'color',[1 0 1]);
   delete(h);
%    hm=plot(x(1),x(2),'mx','linewidth',3,'markersize',10);
   g=get(gcf,'UserData'); 
   g.allX=[g.allX, x];
   set(gcf, 'UserData', g);
   plot(x(1),x(2),'k.');hold on
   
   if size(g.allAngleHat,2) > nb
       nb = size(g.allAngleHat,2);
       
       v=originalDynamicsLINEAR(x);
       angle = atan2(v(2), v(1)) + g.allAngleHat(end);
       plot([x(1), x(1)+cos(angle)*10], [x(2), x(2)+sin(angle)*10], 'r');
       hold on

       angle = atan2(v(2), v(1)) + g.allAngle(end);
       plot([x(1), x(1)+cos(angle)*10], [x(2), x(2)+sin(angle)*10], 'm');
       
   end
   
   pause(dt)
   if(norm(x)<5)
       break;
   end
   b=get(gcf,'UserData');
   breakSimulation = b.breakSimulation;
end

    %plotall();
    s.allAngle = [];
    s.allVelocity = [];
    s.allt=[];
    set(gcf, 'UserData', s);

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

% ------- myOwnSpline Functions

function splineData = ownSpline_o(x,y, dev)
    splineData.points = [];
    splineData.coef   = [];
    splineData.nbPoint= size(x,1);
    
    for i=1:size(x,1)-1
        splineData.coef   = [splineData.coef, systemSolver(x(i), x(i+1), y(i), y(i+1), dev(i), dev(i+1))];
        splineData.points = [splineData.points, [x(i); x(i+1)]];
    end
end

function coef = systemSolver(t1, t2, f1, f2, fp1, fp2)
% at^3+bt^2+ct+d=ft(1,2)
% 3at^2+2bt+c=ftp(1,2)
% ( t1^3    t1^2    t1     1) ( a )   ( ft1)
% ( t2^3    t2^2    t2     1) ( b )   ( ft2)
% (3t1^2    2t1     1      0) ( c ) = (fpt1)
% (3t2^2    2t2     1      0) ( d )   (fpt2)
    
    A = [[t1.^3   t1.^2  t1 1];
         [t2.^3   t2.^2  t2 1];
         [3*t1.^2 2*t1   1  0];
         [3*t2.^2 2*t2   1  0]];
     
    b=[f1;f2;fp1;fp2];
    
    %A*coef = b
    coef = A\b;
    %coef = inv(A)*b;

end

function yy=getPointsSplineNO_o(splineData, xx)
%this function coulp be optimized a bit, so NO for Not optimized
    yy=[];
    for i=1:size(xx,2)
        count=2;
        while (count<splineData.nbPoint && xx(i) > splineData.points(1,count))
            count = count+1;
        end
        yy=[yy getValCoef(splineData.coef(:,count-1), xx(i))];
    end
end

function dyy=getVelSplineNO_o(splineData, xx)
%this function coulp be optimized a bit, so NO for Not optimized
    dyy=[];
    for i=1:size(xx,2)
        count=2;
        while (count<splineData.nbPoint && xx(i) > splineData.points(1,count))
            count = count+1;
        end
        dyy=[dyy getDerCoef(splineData.coef(:,count-1), xx(i))];
    end
end

function y=getValCoef(coef, x)
    y=coef(1).*x.^3+coef(2).*x.^2+coef(3).*x+coef(4);
end

function dy=getDerCoef(coef, x)
    dy=3.*coef(1).*x.^2+2.*coef(2).*x+coef(3);
end

% -------- matlab spline adaptation

function splineData = ownSpline(x,y, dev)
    sp = spline(x,y);
    
    addpath('slm')
    
    splineData.points = [sp.breaks(1:end-1);sp.breaks(2:end)];
    splineData.nbPoint= sp.pieces;
    
    coeff = slmpar(sp,'symabs');
    splineData.coef = [];
    
    for i=1:splineData.nbPoint
        splineData.coef = [splineData.coef, sym2poly(coeff{2,i})'];
    end
    a = sp.coefs';
end

function yy=getPointsSplineNO(splineData, xx)
%this function coulp be optimized a bit, so NO for Not optimized
    yy=[];
    for i=1:size(xx,2)
        count=2;
        while (count<splineData.nbPoint && xx(i) > splineData.points(1,count))
            count = count+1;
        end
        yy=[yy getValCoef(splineData.coef(:,count-1), xx(i))];
    end
end

function dyy=getVelSplineNO(splineData, xx)
%this function coulp be optimized a bit, so NO for Not optimized
    dyy=[];
    for i=1:size(xx,2)
        count=2;
        while (count<splineData.nbPoint && xx(i) > splineData.points(1,count))
            count = count+1;
        end
        dyy=[dyy getDerCoef(splineData.coef(:,count-1), xx(i))];
    end
end

% -------- distance func

function d = dist(x1,y1,x2,y2)
    d = sqrt((x1-x2).^2 + (y1-y2).^2);
end

function t_min = minimumDistance2Spline(xs,ys,spline2D)
    t = distance2Spline2D(xs, ys, spline2D);
    dist(getPointsSplineNO(spline2D(1), t')', getPointsSplineNO(spline2D(2), t')', t./t.*xs, t./t.*ys);

    if size(t,1) == 1 && t == -1
        t_min =-1;
        return
    end
    
    d = dist(getPointsSplineNO(spline2D(1), t(1) ), getPointsSplineNO(spline2D(2), t(1)), xs,ys);
    t_min = t(1);
    for j=2:size(t, 1)
        dlocal = dist(getPointsSplineNO(spline2D(1), t(j) ), getPointsSplineNO(spline2D(2), t(j)), xs,ys);
        if dlocal < d
            d = dlocal;
            t_min = t(j);
        end
    end
end

function t = distance2Spline2D(xs, ys, spline2D)
    nbpts = min([spline2D(1).nbPoint, spline2D(2).nbPoint]);
    t = [];
    
    for i=1:nbpts-1
        if (i==1 || i == nbpts-1)
            new_t = distance2Spline(spline2D(1).points(:,i), spline2D(1).coef(:,i), spline2D(2).coef(:,i), xs, ys, 1);
        else
            new_t = distance2Spline(spline2D(1).points(:,i), spline2D(1).coef(:,i), spline2D(2).coef(:,i), xs, ys, 1);
        end
        
        if new_t == -1 %error
            continue
        end
        t = [t;new_t];
    end
    
    if (size(t,1) == 0)
        t=-1;
    end
end

function t = distance2Spline(t_lim, coefx, coefy, xs, ys, sharpParam)

    %the spline is parametrize by t form t_lim(1) to t_lim(2), check
    %distance from all the spline to (xs,ys)
    %if sharpParam = 0, check on aaaall the curve (-inf to inf)
    %if sharpParam = 1, only check on t_lim interval

    % equ to resolve : a.t^5 + b.t^4+c.t^3+d.t^2+e.t+f = 0
    
    AA=[coefx(1), coefy(1)];
    BB=[coefx(2), coefy(2)];
    CC=[coefx(3), coefy(3)];
    DD=[coefx(4), coefy(4)];
    x =[xs, ys];
    
    a=0; b=0; c=0; d=0; e=0; f=0;

    for i=1:2
        A = AA(i);
        B = BB(i);
        C = CC(i);
        D = DD(i);
        mx = x(i);
        
        a = a + 6*A^2;
        b = b + 10*A*B;
        c = c + 8*A*C + 4*B^2;
        d = d + 6*A*D + 6*B*C - 6*A*mx;
        e = e + 4*B*D + 2*C^2 - 4*mx*B;
        f = f + 2*C*D - 2*mx*C;
    end

    value = roots([a,b,c,d,e,f]);
    rvalue = [];

    for i = 1:size(value,1)
        if sharpParam == 0 && imag(value(i)) == 0    
            rvalue = [rvalue; real(value(i))];
        elseif sharpParam == 1 && imag(value(i)) == 0 && real(value(i))>= t_lim(1) && real(value(i))<= t_lim(2)
                rvalue = [rvalue; real(value(i))];
        end
    end

    if size(rvalue, 1) == 0
        t = -1; %error, no value found
    elseif size(rvalue, 1) == 1
        t = rvalue(1);
    else
       fx = @(t) coefx(1).*t.^3+coefx(2).*t.^2+coefx(3).*t+coefx(4);
       fy = @(t) coefy(1).*t.^3+coefy(2).*t.^2+coefy(3).*t+coefy(4);
       
       t = rvalue(1);
       dmin = dist(fx(t), fy(t), xs, ys);
      
       for i=2:size(rvalue, 1)
           d = dist(fx(rvalue(i)), fy(rvalue(i)), xs,ys);
           if (d<dmin)
               dmin=d;
               t=rvalue(i);
           end
       end
    end
end




