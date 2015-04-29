function ret = threeDimEx()

nX = 10;
nY = 10;
nZ = 10;
dataLimits = getDataLimits();
x = linspace(dataLimits(1),dataLimits(2),nX);
y = linspace(dataLimits(1),dataLimits(2),nY);
z = linspace(dataLimits(1),dataLimits(2),nZ);
[X,Y,Z] = meshgrid(x,y,z);
XYZ = grid2Vector(X,Y,Z);

% set up trainingdata and gp
data = getData();
gpData = getGPData();

[M S2] = gpRegress(gpData(1:3,:),gpData);
%gpData = selectGPdata(data);
XYZd=reshapedDynamics(XYZ,gpData);
[Xd Yd Zd] = vector2grid(XYZd,nX,nY,nZ);

figure(1);clf;hold on;
colormap jet
plot3(0,0,0,'k.','markersize',20)
tubePlotForDemos2(data,gpData);
plotISOsurface(gpData,0.6,dataLimits);
xlabel('X')
ylabel('Y')
zlabel('Z')
%quiverPlot(X,Y,Z,Xd,Yd,Zd);
%plot3(gpData(1,:),gpData(2,:),gpData(3,:),'m+','markersize',10,'linewidth',2)
sx = 0; sy=0;sz=0;
%plot3dInfluenceSlice(gpData,plotLimitsXYZ,sx,2,2)
%return

figure(1)
x0 = gpData(1:3,1);
%x0(3) = x0(3)-0.5
[xSim xdSim] = simulateTrajectory(x0,gpData);
%plotStreamtape(xSim,xdSim);
%plot3(xSim(1,end),xSim(2,end),xSim(3,end),'mo','markersize',10,'linewidth',2)
nSim = 3;
i=0;
X0 = repmat(gpData(1:3,1),1,5);
X0(2,1) = X0(2,1)+1;
X0(3,1) = X0(3,1)-2;

X0(2,2) = X0(2,2);
X0(3,2) = X0(3,2)-2;


X0(1,3) = X0(1,3)-1;
X0(2,3) = X0(2,3)+2;
X0(3,3) = X0(3,3)-1;

col = {'r','b','y'}

while i<nSim
% simulation
x0 = [gpData(1,1);gpData(2:3,1)+0.5*randn(2,1)+[1;-3]];
x0 = X0(:,i+1);
[xSim xdSim] = simulateTrajectory(x0,gpData)

plotStreamtape(xSim,xdSim,col{i+1},0.2);
%plot3(xSim(1,end),xSim(2,end),xSim(3,end),'ro','markersize',10,'linewidth',2)
i=i+1;
end
axis tight
camlight 
lighting gouraud
box on
view(3)
camorbit(140,-20)
end


function h=plotStreamtape(xSim,xdSim,col,w)

spd = sqrt(sum(xdSim.^2,1));
h =streamribbon({xSim'},{zeros(size(xSim,2),1)},w);
set(h,'cdata',[spd',spd'],'linestyle','none');
set(h,'facecolor',col)
%plot3(xSim(1,end),xSim(2,end),xSim(3,end),'mo','markersize',10,'linewidth',2);

end

function dataLimits = getDataLimits()
dataLimits = zeros(1,6);
dataLimits(1)=-10;
dataLimits(2)=2;
dataLimits(3)=-10;
dataLimits(4)=10;
dataLimits(5)=-10;
dataLimits(6)=10;
end

function data = getData()
data = generateTrainingData('escargot',[2 4 -6],1);
end

function gpData = getGPData()
data = getData();
ss = 2;
data = data(:,1:ss:end);
gpData = computeLMDSdata3D(data(1:3,:),data(4:6,:),originalDynamics(data(1:3,:)));

end


function s=getGP()
% GP hyper-params
ell = 1; 
sf = 1; 
s.hyp.cov = log([ell; sf]); 
sn = 0.4; 
s.hyp.lik = log(sn);
s.infMethod = @infExact;
s.meanFunc = @meanZero;
s.covFunc = @covSEiso;
s.likFunc = @likGauss;
end

function plot2dInfluence(gpData,inds,plotLimitsXYZ)
nX2 = 30;
nY2 = 30;
[X2,Y2]=meshgrid(linspace(plotLimitsXYZ(1+2*(inds(1)-1)),plotLimitsXYZ(2+2*(inds(1)-1)),nX2),linspace(plotLimitsXYZ(1+2*(inds(2)-1)),plotLimitsXYZ(2+2*(inds(2)-1)),nY2));
XY2 = [X2(:)';Y2(:)'];
s = getGP();
oneHat = gp(s.hyp,s.infMethod,s.meanFunc,s.covFunc,s.likFunc,gpData([inds(1),inds(2)],:)',ones(size(gpData,2),1),XY2');
oneHat(oneHat>1)=1;
inflHandle=pcolor(X2,Y2,reshape(oneHat,nX2,nY2));
set(inflHandle,'linestyle','none');
load whiteCopperColorMap;
colormap(cm);
shading interp
colorbar;
lab = ['X','Y','Z'];
xlabel(lab(inds(1)));
ylabel(lab(inds(2)))
end

function plot3dInfluenceSlice(gpData,plotLimitsXYZ,sx,sy,sz)
nX = 20;
nY = 20;
nZ = 20;

x = linspace(plotLimitsXYZ(1),plotLimitsXYZ(2),nX);
y = linspace(plotLimitsXYZ(3),plotLimitsXYZ(4),nY);
z = linspace(plotLimitsXYZ(5),plotLimitsXYZ(6),nZ);
[X,Y,Z] = meshgrid(x,y,z);
XYZ = grid2Vector(X,Y,Z);
s = getGP();
oneHat = gp(s.hyp,s.infMethod, s.meanFunc,s.covFunc, s.likFunc, gpData(1:3,:)',ones(size(gpData,2),1),XYZ');
oneHat = reshape(oneHat, nY,nX,nZ);

h=slice(x,y,z,oneHat,sx,sy,sz);
load whiteCopperColorMap;
colormap(cm)
view(3)
end


function [xSim, xdSim] = simulateTrajectory(x0,gpData)
dt = 0.001;
t=0;
xSim = [];
xdSim = [];
x = x0;
T = 10;
[T xSim]=ode45(@(t,x) reshapedDynamics(x,gpData),[0 T],x0)
xSim = xSim';
xdSim = reshapedDynamics(xSim,gpData);
end


function [M S2] = gpRegress(x_query,gpData)
s = getGP();
% predict reshaping parameters at requested inputs
nParams = size(gpData,1)-3;
nQuery = size(x_query,2);
M = zeros(nParams,nQuery);
S2 = M;
for i=1:nParams
[m,s2]=gp(s.hyp,s.infMethod,s.meanFunc,s.covFunc,s.likFunc,gpData(1:3,:)',gpData(3+i,:)',x_query');
M(i,:)=m';
S2(i,:)=s2';
end
end

function XYZ = grid2Vector(X,Y,Z)
XYZ = [X(:)';Y(:)';Z(:)'];
end

function [X,Y,Z]=vector2grid(XYZ,nX,nY,nZ)
X = reshape(XYZ(1,:),nY,nX,nZ);
Y = reshape(XYZ(2,:),nY,nX,nZ);
Z = reshape(XYZ(3,:),nY,nX,nZ);
end

function xd=originalDynamics(x)
A = -0.4*eye(3);
xd = A*x;
end



function xd = reshapedDynamics(x,gpData)
xd = originalDynamics(x);
P = gpRegress(x,gpData);
xd = locallyRotate3dV(xd,P);
end

function quiverPlot(X,Y,Z,U,V,W)
%quiver3(X,Y,Z,U,V,W,1,'k','linewidth',2)
ss = 3;
coneplot(X,Y,Z,U,V,W,X(1:ss:end,1:ss:end),Y(1:ss:end,1:ss:end),Z(1:ss:end,1:ss:end))
end

function tubePlotForDemos(data,gpData)
vertices = {[data(1:3,:)']};
ss = 2;
vertices2 = {vertices{1}(1:ss:end,:)};


% define the width of the tube
nVert = size(vertices{1},1);
queryPoints = vertices2{1}';
r=findGPRadius(gpData,queryPoints);
r=0.1
w=r;
w2 = w(1:ss:end);
w = {w};
w2 = {r};

%vertices2{1}(20:30,:)=[];
%w2{1}(20:30)=[];
%h=streamtube(vertices,w);
%set(h,'facecolor',[1 0 0],'facealpha',0.2,'linestyle','none');
h2 = streamtube(vertices2,w2,[1,12]);
set(h2,'facecolor',1*[0 1 0],'facealpha',1);
set(h2,'linestyle','none')

%set(h2,'Xdata',X,'YData',Y,'ZData',Z);
axis equal
%quiverPlot(X,Y,Z,Xd,Yd,Zd)
end

function tubePlotForDemos2(data,gpData)
% find indices of training data


vertices = {[data(1:3,:)']};

% define the width of the tube
nVert = size(vertices{1},1);
r=0.2
h2 = streamtube(vertices,r,[1,20]);
set(h2,'facecolor',1*[0 1 0],'facealpha',1);
set(h2,'linestyle','none')
%put spheres on gpData

rr=1.1*r/2;
[x,y,z]=sphere;
x = rr*x; y = rr*y; z = rr*z;
nTrain = size(gpData,2);
for i =1:nTrain
   h=surf(x+gpData(1,i),y+gpData(2,i),z+gpData(3,i)) 
   set(h,'linestyle','none','facecolor','m')
end
end

function plotISOsurface(gpData,isoval,dataLimits)
% grid:
nX = 80;
nY = 80;
nZ = 80;
x = linspace(dataLimits(1),dataLimits(2),nX);
%x = linspace(dataLimits(1),dataLimits(2),nX);

y = linspace(dataLimits(3),dataLimits(4),nY);
z = linspace(dataLimits(5),dataLimits(6),nZ);
[X,Y,Z]=meshgrid(x,y,z);
XYZ = grid2Vector(X,Y,Z);
nTrain  = size(gpData,2);
oneHat = gpRegress(XYZ,[gpData(1:3,:);ones(1,nTrain)]);
oneHat = reshape(oneHat,nY,nX,nZ);
s = isosurface(X,Y,Z,oneHat,isoval);
h=patch(s)
set(h,'facecolor',0.4*[1 1 1],'facealpha',0.5,'linestyle','none')
end


function r=findGPRadius(gpData,x_query)


nTrain = size(gpData,2);
nX = size(x_query,2);
r = zeros(1,nX);
s = getGP();
[dum s2] = gpRegress(x_query,gpData);


% method 1, assume that influence at single data point is good
% approximation
%sn = exp(s.hyp.lik)
sn2 = exp(s.hyp.lik*2);
sf2 = exp(s.hyp.cov(2)*2);
ell = exp(s.hyp.cov(1));
rad = sqrt(-2*ell*log(0.3*(1+sn2/sf2)))
r(:)=rad;
%return

%method 3 search in perpendicular direction to find specific value of
%regression. 
xd = reshapedDynamics(x_query, gpData);
oneHat = gpRegress(x_query,[gpData(1:3,:);ones(1,nTrain)]);
for n=1:nX
    % vector perpendicular to direction of motion
    v=randn(1,3);
    searchDir = cross(xd(:,n)/norm(xd(:,n)),v/norm(v))';
    searchDir = searchDir/norm(searchDir);
    xq = x_query(:,n);
    
    objFun = @(rad) (gpRegress(xq+rad*searchDir,[gpData(1:3,:);ones(1,nTrain)])-0.7)^2;
    options = optimset('Display','none');
    rOpt = fmincon(objFun, 0, [],[],[],[],0,[],[],options);
    
    r(n)=rOpt;
    %plot3([xq(1),xq(1)+10*searchDir(1)],[xq(2),xq(2)+10*searchDir(2)],[xq(3),xq(3)+10*searchDir(3)],'r')
    
end


end