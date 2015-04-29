function h=PlotEstimatedVA_P(Y,X_hat,Xd_hat,Xdd_hat,dt)
%h=PlotEstimatedVA_P(Y,X_hat,Xd_Hat,Xdd_hat,dt)
% a function that plots raw position measurements and estimated
% trajectories of position, velocity and acceleration. This function is
% useful for visual inspection of estimates for tuning the measurement
% noise variance R and process noise Q for Kalman smoothing.
[dim,Ndata]=size(Y);
ms=6;
lw=2;
t=linspace(0,Ndata*dt,Ndata);

% integration
X_int = zeros(dim,Ndata);
X_int(:,1)=X_hat(:,1);
Xd_int = zeros(dim,Ndata);
Xd_int(:,1)=Xd_hat(:,1);
for i=2:Ndata
   Xd_int(:,i)=Xd_int(:,i-1)+Xdd_hat(:,i-1)*dt;
   X_int(:,i)=X_int(:,i-1)+Xd_int(:,i-1)*dt+Xdd_hat(:,i-1)*dt^2/2;  
end



figure(45);clf;
subplot(dim,1,1)
title('Estimated Position')
for i=1:dim
    subplot(dim,1,i);
    hold on;
    plot(t,Y(i,:),'.','markersize',ms);
    plot(t,X_hat(i,:),'r','linewidth',lw);
    plot(t,X_int(i,:),'g--','linewidth',lw);
    xlabel('time');
    ylabel('position');
    legend('Position measurements','estimated position','integrated position');
end

figure(46);clf;
subplot(dim,1,1)
title('Estimated Velocity')
for i=1:dim
    subplot(dim,1,i);
    hold on;
    %plot(Y(i,:),'.','markersize',ms);
    plot(t,Xd_hat(i,:),'r','linewidth',lw);
    plot(t,Xd_int(i,:),'g--','linewidth',lw)
    legend('estimated velocity','integrated velocity')
    xlabel('time');
    ylabel('velocity');

end

figure(47);clf;
subplot(dim,1,1)
title('Estimated Acceleration')
for i=1:dim
    subplot(dim,1,i);
    hold on;
    %plot(Y(i,:),'.','markersize',ms);
    plot(t,Xdd_hat(i,:),'r','linewidth',lw);
    xlabel('time');
    ylabel('acceleration');

end


