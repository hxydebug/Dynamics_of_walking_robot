function T = revolute(theta,r,u)

ux = u(1); uy = u(2); uz = u(3);
cth = cos(theta);
sth = sin(theta);
vth = 1-cth;
R(1,1) = ux^2*vth + cth;
R(1,2) = ux*uy*vth-uz*sth;
R(1,3) = ux*uz*vth+uy*sth;
R(2,1) = ux*uy*vth + uz*sth;
R(2,2) = uy^2*vth + cth;
R(2,3) = uy*uz*vth - ux*sth;
R(3,1) = ux*uz*vth - uy*sth;
R(3,2) = uy*uz*vth + ux*sth;
R(3,3) = uz^2*vth + cth;
I = eye(3,3);


T = [R, (I-R)*r'; 
     zeros(1,3), 1];