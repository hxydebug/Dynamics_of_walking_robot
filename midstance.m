%===================================================================
function [gstop, isterminal,direction]=midstance(t,Z,parms)
%===================================================================

l0 = parms.l0;
l1 = parms.l1;
l2 = parms.l2;
w = parms.w;

[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ....
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd] = getstate(Z);


%%% for straight line walking 
% [B,H,LH,LK,LA,RH,RK,RA,b,rt,rc,lt,lc] = joint_locations(l0,l1,l2,phi,phi_lh,phi_rh,psi_lh,psi_rh,psi,theta,theta_lh,theta_lk,theta_rh,theta_rk,w,x,y,z);
% x_lhip = LH(1);
% x_rhip = RH(1);
% x_lfoot = LA(1);
% x_rfoot = RA(1);
% 
% if (strcmp(parms.stance_foot,'r'))
%     gstop = x_rhip - x_rfoot;
% elseif (strcmp(parms.stance_foot,'l'))
%     gstop = x_lhip - x_lfoot;
% end

gstop = zd; % l1*cos(theta0 + theta1) - l1*cos(theta0 + theta2) + l2*cos(theta0 + theta1 + theta3) - l2*cos(theta0 + theta2 + theta4);
isterminal = 1; 
% if (theta01>-0.05) %allow legs to pass through for small hip angles (taken care in real walker using stepping stones)
%     isterminal = 0;
% else
%     isterminal=1; %ode should terminate is conveyed by 1, if you put 0 it goes till the final time u specify
% end
direction=[]; % The t_final can be approached by any direction is indicated by the direction
