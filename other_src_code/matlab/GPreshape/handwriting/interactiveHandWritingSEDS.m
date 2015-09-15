function [  ] = interactiveHandwritingSEDS(dd, Priors, Mu ,Sigma,ell)

global originalGMM originalDynamics args breakSimulation;

breakSimulation = 0;

args = {dd,Priors,Mu,Sigma,ell};
originalGMM.Mu = Mu;
originalGMM.Priors = Priors;
originalGMM.Sigma = Sigma;
originalDynamics = @(x) originalDynamicsGMR(x);

set(gcf,'UserData',[])
set(gcf,'WindowButtonDownFcn',[])
set(gcf,'WindowButtonUpFcn',[]);
set(gcf,'WindowButtonMotionFcn',[]);
drawnow()

clf;
hold on
nDemos = size(dd,2);
Data = [];
for n=1:nDemos
    Data = [Data,dd{n}.pos];
    ss = 10;
   h=plot(dd{n}.pos(1,1:ss:end),dd{n}.pos(2,1:ss:end),'.','color',[1 0 0]);
   %plot(0,0,'k.','markersize',50)
end
b = 10;
dataLimits = [min(Data(1,:))-b,max(Data(1,:))+b,min(Data(2,:))-b,max(Data(2,:))+b];
nX = 20;
nY = 30;
[X Y ] = meshgrid(linspace(dataLimits(1),dataLimits(2),nX), linspace(dataLimits(3),dataLimits(4),nY));
xx = [X(:)';Y(:)'];
xxd = GMR(Priors,Mu,Sigma,xx,1:2,3:4);
Xd = reshape(xxd(1,:),nY,nX);
Yd = reshape(xxd(2,:),nY,nX);
hs = streamslice(X,Y,Xd,Yd,0.5);
axis tight 
box on

hi = plot(0,0);
%prepare a place where the functions can put data 
s = {};
gridData.xM = X;
gridData.yM = Y;
gridData.X = xx;
gridData.nX = nX;
gridData.nY = nY;
s.gridData = gridData;
s.streamHandle = hs;
s.inflHandle = hi;
s.allData = [];
s.gpData = [];
s.newPosData = [];
s.drawingDt = 0.01;
s.breakSimulation = 0;

% parameters for GP regression
s.gprStruct.meanfunc = {@meanZero};
s.gprStruct.covfunc = {@covSEiso}; 
%ell = 2; 
sf = 1; 
s.gprStruct.hyp.cov = log([ell; sf]);
s.gprStruct.likfunc = @likGauss; 
sn = 0.4; 
s.gprStruct.hyp.lik = log(sn);
% store a handle to the regressionfunction for use in various functions
s.gprStruct.regressionFunction = @(x_train,y_train,x_query) gp(s.gprStruct.hyp, @infExact, s.gprStruct.meanfunc, s.gprStruct.covfunc, s.gprStruct.likfunc, x_train, y_train, x_query);

set(gcf,'UserData',s)

% prepare for interaction
set(gcf,'WindowButtonDownFcn',@(h,e)buttonClicked(h,e,args));

end



function [xd jac] = originalDynamicsGMR(x)
global originalGMM;
xd = GMR(originalGMM.Priors,originalGMM.Mu,originalGMM.Sigma,x,1:2,3:4 );
jac = eye(2);
end

function [xd jacobian] = originalDynamicsLINEAR(x)
p = 4;
A = -p*eye(2);
xd =A*x; 
%xd(2,:) =xd(2,:) + 2*p*x(1,:).*cos(2*pi/150*x(1,:));
jacobian = A;
end

function Xd = reshapedDynamics(x)
global originalDynamics;
s = get(gcf,'UserData');
angleHat = s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(3,:)',x');
speedHat = s.gprStruct.regressionFunction(s.gpData(1:2,:)', s.gpData(4,:)',x');
th = 0.2;
speedHat(speedHat<= -1+th) = -1+th;
speedHat = 0*speedHat;
%speedHat = customLogistic(speedHat,-1,20);
Xd = originalDynamics(x);
Xd = locallyRotateV(Xd,angleHat,speedHat);
end


function ret = buttonClicked(h,e,args)
global breakSimulation;
breakSimulation = 1;
disp(get(gcf,'selectiontype'))
s=get(gcf,'UserData');
s.breakSimulation =1;
set(gcf,'UserData',s);


if(strcmp(get(gcf,'SelectionType'),'normal'))
    x = get(gca,'Currentpoint');
    x = x(1,1:2)';
    startSimulation(x);
elseif(strcmp(get(gcf,'SelectionType'),'alt'))
    startDemonstration();
elseif(strcmp(get(gcf,'SelectionType'),'extend'))
    set(gcf,'WindowButtonUpFcn',[]);
    s.breakSimulation =1;
set(gcf,'UserData',s);
    interactiveHandWritingSEDS(args{1},args{2},args{3},args{4},args{5});
end
breakSimulation = 1;

end

function ret = startDemonstration()

disp('started demonstration')
set(gcf,'WindowButtonUpFcn',@stopDemonstration);
set(gcf,'WindowButtonMotionFcn',@recordPoint);
tic
end

function ret = stopDemonstration(h,e)
disp('stopped demonstration')
set(gcf,'WindowButtonMotionFcn',[]);
processNewData();
updateStreamlines();
set(gcf,'WindowButtonUpFcn',[]);
end



function ret = recordPoint(h,e)
disp('recording point');
s =get(gcf,'UserData');
x = get(gca,'Currentpoint');
x = x(1,1:2)';
s.newPosData = [s.newPosData,x];
set(gcf,'UserData',s);
plot(x(1),x(2),'go')
%toc
%tic
%pause(s.drawingDt);
end

function ret = updateStreamlines()
global originalDynamics;
s = get(gcf,'UserData');
% 
% angleHat = s.regressionFunction(s.gpData(1:2,:)',s.gpData(3,:)',s.gridData.X');
% speedHat = s.regressionFunction(s.gpData(1:2,:)', s.gpData(4,:)', s.gridData.X');
% Xd = originalDynamics(s.gridData.X);
% Xd = locallyRotateV(Xd,angleHat,speedHat);
Xd = reshapedDynamics(s.gridData.X);
% plot
delete(s.streamHandle);
s.streamHandle = streamslice(s.gridData.xM,s.gridData.yM,reshape(Xd(1,:),s.gridData.nY,s.gridData.nX),reshape(Xd(2,:),s.gridData.nY,s.gridData.nX),0.5);
infl =  s.gprStruct.regressionFunction(s.gpData(1:2,:)',ones(size(s.gpData(3,:)')),s.gridData.X');
load whiteCopperColorMap;
delete(s.inflHandle);
s.inflHandle=pcolor(s.gridData.xM,s.gridData.yM,reshape(infl,s.gridData.nY,s.gridData.nX));
shading interp
set(s.inflHandle,'linestyle','none');
colormap(cm);

x=[0;0];
[orgVel orgJac]=originalDynamics(x);
J=compute2dGPLMDSjacobian(s.gprStruct,s.gpData,orgVel,orgJac,x);
eig(J)
set(gcf,'UserData',s);
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
% kalman smoothing of the demonstration
[demPos,demVel, demAcc] = EstimateVA_P(s.newPosData(1:2,:),s.drawingDt,1e2*ones(2,1),0.001*ones(2,1),[s.newPosData(1:2,1);zeros(4,1)],P0);
% get the angle and speed factor 
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
newData(3,:)
for n=1:nNewData
    [angleHat s2]=s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(3,:)',newData(1:2,n)');
     [speedHat s2]=s.gprStruct.regressionFunction(s.gpData(1:2,:)',s.gpData(4,:)',newData(1:2,n)');
    %xd = reshapedDynamics(demPos(:,n));
    if ( abs((speedHat - newData(4,n))/newData(4,n)) > 100 | abs((angleHat - newData(3,n))) > 0.1    )%( norm(xd - demVel(:,n))/norm(demVel(:,n)) > 1.5)%abs((a-newData(3,n))/newData(3,n))>0.3)%s2>0.8)
        s.gpData = [s.gpData,newData(:,n)];
        % need to write it back here for reshapedDynamics function
        set(gcf,'UserData',s);
    else
        kuk=1;
    end 
end
s.allData = [s.allData,newData];
s.newPosData = [];
hgp = findobj(gcf,'color','m');
delete(hgp);
plot(s.gpData(1,:),s.gpData(2,:),'mo');
set(gcf,'UserData',s);
end

function r = startSimulation(x)

global originalDynamics  breakSimulation;
s = get(gcf,'UserData');

bHaveGPData = 0;
if(size(s.gpData,2)>1)
    bHaveGPData = 1;
end

disp('starting simulation')
xd = 0;
dt = s.drawingDt;
t=0;
breakSimulation = 0;
hm=plot(0,0)
while 1
    if(bHaveGPData)
        xd = reshapedDynamics(x);
    else
        xd = originalDynamics(x);
    end
   x = x+dt*xd;
   t=t+dt;
   %h = findobj(gcf,'color',[1 0 1]);
   %delete(h);
   %hm=plot(x(1),x(2),'mx','linewidth',3,'markersize',10);
   plot(x(1),x(2),'k.');
   pause(dt)
   if(norm(x)<5)
       break;
   end
   x;
   xd;
   drawnow
   b=get(gcf,'UserData');
   %breakSimulation = b.breakSimulation
   breakSimulation
   if(breakSimulation)
       disp 'leacing'
       break;
   end
end
end
