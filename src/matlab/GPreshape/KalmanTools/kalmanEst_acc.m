%% kalman estimator of position, velocity and acceleration based on position measurements and accelerometer

% 1 generate some data
dt = 0.001;
Tf = 3;
t = 0:dt:Tf;

x = sin(2*pi*t);
xd = 2*pi*cos(2*pi*t);
xdd = -4*pi^2*sin(2*pi*t)

R_p=0.4; % measurement noise on position
R_a = 30; % measurement noise on acceleration
R = [R_p 0;0 R_a];
x_m = x +R_p*randn(size(x))
%xd_m = diff(x_m)./dt;
%xd_m = [xd_m, xd_m(end)];
%xdd_m = diff(xd_m)./dt;
%xdd_m  = [xdd_m, xdd_m(end)];
xdd_m = xdd + R_a*randn(size(xdd));

A = [1 dt dt^2/2; 0 1 dt; 0 0 1 ];
C = [1 0 0; 0 0 1];
Q = 0.1;
Gv = [0;0;1];
%R = 0.3;

x0=[0;10;0];
P_hat = [1 0 0; 0 100 0;0 0 100];
P_pred = P_hat;
x_pred = x0;
X_pred = [];
X_hat = [];
P_hat_ = zeros(3,3,length(t));
P_pred_ =P_hat_; 

%filtering
for i=1:length(t)
    
    %[x_pred P_pred x_hat P_hat]=KalmanPredict(x_pred,P_pred,x_m(i),A,C,Q,R)
    H = inv(C*P_pred*C'+R);
   
    L = P_pred*C'*H;
    [x_pred, P_pred, x_hat,P_hat]=KalmanPredict(x_pred,P_pred,[x_m(i);xdd_m(i)],A,C,Gv*Q*Gv',R);
    %[x_hat P_hat] = KalmanMeasurementUpdate(x_pred,P_pred,x_m(i),C,R)
    %x_hat = x_pred+L*(x_m(i)-C*x_pred);
    %[x_pred P_pred] = KalmanTimeUpdate(x_hat,P_hat,A,Gv*Q*Gv')
    %x_pred = A*x_hat;
    
    X_pred = [X_pred,x_pred];
    X_hat = [X_hat,x_hat];
%     
%     P_hat = P_pred - P_pred*C'*H*C*P_pred;
%     P_pred = A*P_hat*A'+Gv*Q*Gv';
    
    P_hat_(:,:,i)=P_hat;
    
    P_pred_(:,:,i)=P_pred;
    
end

figure(1);clf;
subplot(3,1,1)
title('estimated position trajectory, kalman forward filtering')
hold on
plot(t,x_m);
plot(t,X_hat(1,:),'r','linewidth',2)
plot(t,x,'y--','linewidth',2)

legend('measured pos.','est. pos.','true pos.','location','eastoutside')

subplot(3,1,2)
title('estimated velocity, kalman forward filtering')

hold on
%plot(t,xd_m);
plot(t,X_hat(2,:),'r','linewidth',2)

plot(t,xd,'k--','linewidth',2)
legend('est. vel.','true vel.','location','eastoutside')
subplot(3,1,3)
title('estimated acceleration, kalman forward filtering')

hold on

plot(t,xdd_m)
plot(t,X_hat(3,:),'r','linewidth',2)
plot(t,xdd,'k--','linewidth',2)
legend('meas acc','est. acc.','true acc.','location','eastoutside')


%%
X_hat_2=X_hat;
P_b_=P_hat_;
%smoothing
for i=1:length(t)-1
    
   j=length(t)-i;
   %pause
   %X_hat_2(:,j)= X_hat(:,j)+Pt_(:,:,j)*A'*inv(P_pred_(:,:,j))*(X_hat_2(:,j+1)-X_pred(:,j));
   X_hat_2(:,j)= KalmanBackwardStep(X_hat(:,j),P_hat_(:,:,j),X_pred(:,j),P_pred_(:,:,j),X_hat_2(:,j+1),P_b_(:,:,j+1),A);
   
   
    
end

% integrate the estimated acceleration
X_int = X_hat_2;
for i=2:length(t)
   xdd_est = X_hat_2(3,i);
   X_int(2,i) = X_int(2,i-1) + dt*xdd_est;
   X_int(1,i) =X_int(1,i-1) + dt*X_int(2,i-1)+dt^2/2*xdd_est;
   %x_int = x_int + xd_int*dt + dt^2/2*xdd_est;
    
    
end
figure(2);clf;

subplot(3,1,1)
title('estimated position trajectory, forward-backward kalman')
hold on
plot(t,x,'k--','linewidth',2)
plot(t,x_m);
plot(t,X_hat_2(1,:),'r','linewidth',2)
plot(t,X_int(1,:),'g-.','linewidth',2)
legend('true pos.','measured pos','est. pos.','integrated pos. from est. acc.','location','eastoutside')
subplot(3,1,2)
title('estimated velocity, forward-backward kalman')
hold on
plot(t,xd,'k--','linewidth',2)
%plot(t,xd_m);
plot(t,X_hat_2(2,:),'r','linewidth',2)
plot(t,X_int(2,:),'g-.','linewidth',2)
legend('true vel.','est. vel.','int. vel. from est. acc.','location','eastoutside')
subplot(3,1,3)
title('estimated acceleration, forward-backward kalman')
hold on
plot(t,xdd,'k--','linewidth',2)
%plot(t,xdd_m)
plot(t,X_hat_2(3,:),'r','linewidth',2)
legend('true acc.','est. acc.','location','eastoutside')



%%
[est_p, est_v,est_a]=EstimateVA_P(x_m,dt,Q,R,[0;10;0],[1 0 0;0 100 0;0 0 100]);

figure(2);
subplot(3,1,1);
plot(t,est_p,'k')
subplot(3,1,2);
plot(t,est_v,'k')

subplot(3,1,3);
plot(t,est_a,'k')

