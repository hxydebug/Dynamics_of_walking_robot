function [zdot,A,b,P_LA,P_RA,tau] = single_stanceMEX(t,zz,parms)

B = [0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0; 
     1 0 0 0 0 0 0 0; 
     0 1 0 0 0 0 0 0; 
     0 0 1 0 0 0 0 0; 
     0 0 0 1 0 0 0 0; 
     0 0 0 0 1 0 0 0; 
     0 0 0 0 0 1 0 0; 
     0 0 0 0 0 0 1 0; 
     0 0 0 0 0 0 0 1; 
     0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0; 
     0 0 0 0 0 0 0 0]; 

tau = controller(t,zz,parms);

[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ... ; 
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ... ; 
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(zz); 

[mb,mt,mc,Ibx,Iby,Ibz,Itx,Ity,Itz,Icx,Icy,Icz,l0,l1,l2,w,g] = getparms(parms); 

params = [mb mt mc Ibx Iby Ibz Itx Ity Itz Icx Icy Icz l0 l1 l2 w g]; 

[~,nn] = size(zz);
if (nn==1) 
    zz = zz';
end

[A,b,J_l,J_r,Jdot_l,Jdot_r] = gateway_dynamicsMEX(t,zz,params); 

A = A'; 
J_l = J_l'; J_r = J_r'; 
Jdot_l = Jdot_l'; Jdot_r = Jdot_r'; 
qdot = [xd yd zd phid thetad psid phi_lhd theta_lhd psi_lhd theta_lkd phi_rhd theta_rhd psi_rhd theta_rkd]; 

if (strcmp(parms.stance_foot,'r'))
    AA = [A, -J_r'; J_r, zeros(3,3)];
    bb = [b; -Jdot_r*qdot'] + B*tau; 
   alpha=AA\bb; 

   P_RA = alpha(15:17)'; 
   P_LA = zeros(1,3); 
elseif (strcmp(parms.stance_foot,'l'))
    AA = [A, -J_l'; J_l, zeros(3,3)];
    bb = [b; -Jdot_l*qdot'] + B*tau; 
   alpha=AA\bb; 

   P_LA = alpha(15:17)'; 
   P_RA = zeros(1,3); 
else
   error('parms.stance_foot needs to be set to l or r');
end

xdd = alpha(1); ydd = alpha(2); zdd = alpha(3); phidd = alpha(4); thetadd = alpha(5); psidd = alpha(6); 

phi_lhdd = alpha(7); theta_lhdd = alpha(8); psi_lhdd = alpha(9); theta_lkdd = alpha(10);  

phi_rhdd = alpha(11); theta_rhdd = alpha(12); psi_rhdd = alpha(13); theta_rkdd = alpha(14);  

zdot=[xd xdd yd ydd zd zdd phid phidd thetad thetadd psid psidd phi_lhd phi_lhdd theta_lhd theta_lhdd psi_lhd ...  
psi_lhdd theta_lkd theta_lkdd phi_rhd phi_rhdd theta_rhd theta_rhdd psi_rhd psi_rhdd theta_rkd theta_rkdd]';  

