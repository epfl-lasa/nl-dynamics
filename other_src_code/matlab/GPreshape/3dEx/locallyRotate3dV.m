function xd = locallyRotate3dV(xd,P)
angLimit = 0.001;

angAxis = P(1:3,:);
speed = P(4,:);

nX = size(xd,2);
for j=1:nX
    ang = norm(angAxis(:,j));
    if(abs(ang) > angLimit)
        ax = angAxis(:,j)/ang;
        R = angvec2r(ang,ax);
    else
        R = eye(3);
    end
    
    %xd(:,j) = R(1:2,1:2)*xd(:,j);
    xd(:,j) = (1+speed(j))*R*xd(:,j);
end

end
