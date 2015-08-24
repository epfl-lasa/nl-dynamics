function [xd jacobian] = originalDynamicsLINEAR(x)
    % stable linear isotropic dynamics. adjust speed of convergence with p. 
    p = 4;
    A = -p*eye(2);
    xd =A*x; 
    %xd(2,:) =xd(2,:) + 2*p*x(1,:).*cos(2*pi/150*x(1,:));
    jacobian = A;
end