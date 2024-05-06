%===================================================================
function [zplus,P_LA,P_RA] =footstrike(t,Z,parms)     
%===================================================================

[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ....
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(Z);

P = parms.control.P;

[mb,mt,mc,Ibx,Iby,Ibz,Itx,Ity,Itz,Icx,Icy,Icz,l0,l1,l2,w,g] = getparms(parms);

params = [mb mt mc Ibx Iby Ibz Itx Ity Itz Icx Icy Icz l0 l1 l2 w g]; 

[~,nn] = size(Z);
if (nn==1) 
    Z = Z';
end

[A,~,J_l,J_r] = gateway_dynamicsMEX(t,Z,params);
A = A';
J_l = J_l'; J_r = J_r'; 

qdot_minus = [xd yd zd phid thetad psid ...
    phi_lhd, theta_lhd,psi_lhd, theta_lkd ...
    phi_rhd, theta_rhd,psi_rhd, theta_rkd]';

[I_LA,I_RA] = foot_impulse(P,l2,phi,phi_lh,phi_rh,psi_lh,psi_rh,psi,theta,theta_lh,theta_lk,theta_rh,theta_rk);

if (strcmp(parms.stance_foot,'r'))
    P_RA = I_RA;
    b_hs = [A*qdot_minus+J_r'*P_RA'; zeros(3,1)];
    A_hs = [A -J_l' ; J_l zeros(3,3)];
    X_hs = A_hs\b_hs;
    P_LA = X_hs(15:17)';
elseif (strcmp(parms.stance_foot,'l'))
    P_LA = I_LA;
    b_hs = [A*qdot_minus+J_l'*P_LA'; zeros(3,1)];
    A_hs = [A -J_r' ; J_r zeros(3,3)];
    X_hs = A_hs\b_hs;
    P_RA = X_hs(15:17)';
else
   error('parms.stance_foot needs to be set to l or r');
end

i = 1;
xd = X_hs(i); i=i+1;
yd = X_hs(i); i=i+1;
zd = X_hs(i); i=i+1;
phid = X_hs(i); i=i+1;
thetad = X_hs(i); i=i+1;
psid= X_hs(i); i=i+1;
phi_lhd= X_hs(i); i=i+1;
theta_lhd= X_hs(i); i=i+1;
psi_lhd= X_hs(i); i=i+1;
theta_lkd= X_hs(i); i=i+1;
phi_rhd= X_hs(i); i=i+1;
theta_rhd= X_hs(i); i=i+1;
psi_rhd= X_hs(i); i=i+1;
theta_rkd= X_hs(i); 

zplus = [x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ....
        phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...
        phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd];                     
