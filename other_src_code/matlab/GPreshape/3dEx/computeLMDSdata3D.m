function [ data ] = computeLMDSdata3D( demPos,demVel,orgVel )
% A function that computes velocity scaling and rotation by comparing
% demVel and orgVel at the input points demPos

nbData = size(demVel,2);
data = zeros(7,nbData);
for n =1:nbData
    speedDes = norm(demVel(:,n));
    normDesDir = demVel(:,n)/speedDes;
    speedOrg = norm(orgVel(:,n));
    normOrgDir = orgVel(:,n)/speedOrg;
    %angle = atan2(normDesDir(2),normDesDir(1))-atan2(normOrgDir(2),normOrgDir(1));
    u = getAngleAxis(normOrgDir,normDesDir);
    %angle = norm(u);
    %norm(u)
    %angle
    % put in a good range
%     if(angle > pi)
%         angle = -(2*pi-angle);
%     elseif(angle < -pi)
%         angle = 2*pi+angle;
%     end
    % compute logistic latent function value
    %s = -log(2*pi/angle-1)
    
   % angle = acos(dot(demVel(:,n),orgVel(:,n))/norm(demVel(:,n))/norm(orgVel(:,n)));

    speed_fact = speedDes/speedOrg - 1;
    data(:,n) = [demPos(:,n);u;speed_fact];
end

