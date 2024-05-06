function [x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ....
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(Z)

[m,n] = size(Z);
if (n==1)
    Z = Z';
end
x = Z(:,1); xd = Z(:,2); 
y = Z(:,3); yd = Z(:,4); 
z = Z(:,5); zd = Z(:,6); 
phi = Z(:,7); phid = Z(:,8); 
theta = Z(:,9); thetad = Z(:,10); 
psi = Z(:,11); psid = Z(:,12); 
phi_lh = Z(:,13); phi_lhd = Z(:,14); 
theta_lh = Z(:,15); theta_lhd = Z(:,16); 
psi_lh = Z(:,17); psi_lhd = Z(:,18); 
theta_lk = Z(:,19); theta_lkd = Z(:,20); 
phi_rh = Z(:,21); phi_rhd = Z(:,22); 
theta_rh = Z(:,23); theta_rhd = Z(:,24); 
psi_rh = Z(:,25); psi_rhd = Z(:,26); 
theta_rk = Z(:,27); theta_rkd = Z(:,28); 
