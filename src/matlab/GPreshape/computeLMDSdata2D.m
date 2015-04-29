function [ data ] = computeLMDSdata2D( demPos,demVel,orgVel )
% A function that computes velocity scaling and rotation by comparing
% demVel and orgVel at the input points demPos

nbData = size(demVel,2);
data = zeros(4,nbData);
prevAngle = 0;
for n =1:nbData
    speedDes = norm(demVel(:,n));
    normDesDir = demVel(:,n)/speedDes;
    speedOrg = norm(orgVel(:,n));
    normOrgDir = orgVel(:,n)/speedOrg;
    angle = atan2(normDesDir(2),normDesDir(1))-atan2(normOrgDir(2),normOrgDir(1));
    
    % put in a good range
    if(angle > pi)
        angle = -(2*pi-angle);
    elseif(angle < -pi)
        angle = 2*pi+angle;
    end
    angle = findConsistentAngle(angle,prevAngle);
    prevAngle = angle;
    % compute logistic latent function value
    %s = -log(2*pi/angle-1)
    
   % angle = acos(dot(demVel(:,n),orgVel(:,n))/norm(demVel(:,n))/norm(orgVel(:,n)));

    speed_fact = speedDes/speedOrg - 1;
    data(:,n) = [demPos(:,n);angle;speed_fact];
end

end

function a = findConsistentAngle(angle,prevAngle)
a = angle;
if(prevAngle > pi/2 && angle < - pi/2)
    a = angle + 2*pi;
elseif (prevAngle < -pi/2 && angle > pi/2)
    a = angle - 2*pi;
end
end

