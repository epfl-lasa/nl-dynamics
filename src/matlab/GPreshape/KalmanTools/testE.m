

clear;
dt = 0.01;
Tf = 5;
t = 0:dt:Tf;

X = zeros(2,length(t));
Xd = zeros(2,length(t));
Xdd=zeros(2,length(t));
Y=X;
f=1/4;
R = 2*[0.1;0.1];

for i=1:length(t)
    r=0.5*i*dt;
    
    X(1,i)=r*sin(2*pi*f*t(i));
    X(2,i)=r*cos(2*pi*f*t(i));
    Y(1,i)=X(1,i)+R(1)*randn;
    Y(2,i)=X(2,i)+R(2)*randn;
    
    Xd(1,i)=0.5*sin(2*pi*f*t(i))+r*2*pi*f*cos(2*pi*f*t(i));
    Xd(2,i)=0.5*cos(2*pi*f*t(i))-r*2*pi*f*sin(2*pi*f*t(i));
    
    
    Xdd(1,i)=0.5*2*pi*f*cos(2*pi*f*t(i))-r*(2*pi*f)^2*sin(2*pi*f*t(i));
    Xdd(2,i)=-0.5*2*pi*f*sin(2*pi*f*t(i))-r*(2*pi*f)^2*cos(2*pi*f*t(i));
end

ms=5;
lw=2;

figure(1);clf;
hold on
plot(Y(1,:),Y(2,:),'.','markersize',ms)


Q = 0.1*[1;1];


[X_hat,Xd_hat,Xdd_hat]=EstimateVA_P(Y,dt,Q,R,zeros(6,1),eye(6));
plot(X(1,:),X(2,:),'k--','linewidth',lw)
plot(X_hat(1,:),X_hat(2,:),'r','linewidth',lw)

figure(2);clf;
subplot(2,1,1)
hold on
plot(t,Y(1,:),'.','markersize',ms)
plot(t,X(1,:),'k--','linewidth',lw)
plot(t,X_hat(1,:),'r')
subplot(2,1,2)
hold on
plot(t,Y(2,:),'.','markersize',ms)
plot(t,X(2,:),'k--','linewidth',lw)
plot(t,X_hat(2,:),'r')

figure(3);clf;
subplot(2,1,1)
hold on
%plot(t,Y(1,:),'.','markersize',ms)
plot(t,Xd(1,:),'k--','linewidth',lw)
plot(t,Xd_hat(1,:),'r')
subplot(2,1,2)
hold on
%plot(t,Y(2,:),'.','markersize',ms)
plot(t,Xd(2,:),'k--','linewidth',lw)
plot(t,Xd_hat(2,:),'r')


figure(4);clf;
subplot(2,1,1)
hold on
%plot(t,Y(1,:),'.','markersize',ms)
plot(t,Xdd(1,:),'k--','linewidth',lw)
plot(t,Xdd_hat(1,:),'r')
subplot(2,1,2)
hold on
%plot(t,Y(2,:),'.','markersize',ms)
plot(t,Xdd(2,:),'k--','linewidth',lw)
plot(t,Xdd_hat(2,:),'r')

PlotEstimatedVA_P(Y,X_hat,Xd_hat,Xdd_hat,dt)