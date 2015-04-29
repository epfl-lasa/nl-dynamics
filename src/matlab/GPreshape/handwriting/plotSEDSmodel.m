function [X, Y, Xd, Yd] = plotSEDSmodel(dd,Priors,Mu,Sigma)
hold on
nDemos = size(dd,2);
Data = [];
for n=1:nDemos
    Data = [Data,dd{n}.pos];
   plot(dd{n}.pos(1,:),dd{n}.pos(2,:),'g.','linewidth',2) 
   plot(0,0,'k.','markersize',50)
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
h = streamslice(X,Y,Xd,Yd);
set(h,'linewidth',2)
axis tight 
box on
end
