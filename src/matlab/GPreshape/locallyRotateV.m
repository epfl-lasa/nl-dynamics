function xd = locallyRotateV(xd,angle,speed)

nX = size(xd,2);
for j=1:nX
  
   R = [cos(angle(j)) -sin(angle(j));sin(angle(j)) cos(angle(j))];
    %xd(:,j) = R(1:2,1:2)*xd(:,j);
    xd(:,j) = (1+speed(j))*R(1:2,1:2)*xd(:,j);
end
