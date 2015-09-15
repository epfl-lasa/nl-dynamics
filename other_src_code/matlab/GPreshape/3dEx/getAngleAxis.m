
function uu = getAngleAxis(v1,v2)
% u = getAngleAxis(v1,v2) computes the angle axis representation of the
% rotation from v1 to v2. axis = u/norm(u) angle in radians = norm(u)
 % function computes the axis of rotation and angle required to rotate
 % v1 to align with v2.

w = cross(v1,v2);
u = w/norm(w);
ang = acos(dot(v1,v2)/norm(v1)/norm(v2));

%R = angvec2r(ang,u);

% target direction
%v2/norm(v2);
% resulting direction
%v3 = R*v1;
%v3/norm(v3);

uu = u*ang;
