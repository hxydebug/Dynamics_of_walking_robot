%===================================================================
function [z,t,P_LA_all,P_RA_all,Torque]=onestep(z0,parms,steps)
%===================================================================

tol = 1e-13;
%stop_sim_due_to_error = 1;
% M= humanoid.m0+2*humanoid.m1+2*humanoid.m2;
% L = humanoid.l1+humanoid.l2;
% g = humanoid.g;
flag = 1;
if nargin<2
    error('need more inputs to onestep');
elseif nargin<3
    flag = 0; %send only last state, for root finder and jacobian
    steps = 1;
end

%%%% convention theta0 stays 0; + is backward torso
%%%%%% swing leg 
%%%%% theta2 goes from - to positive
%%%%% theta4 goes from - to 0.
%%%% stance leg %%%
%%%%% theta1 goes from + to -
%%%%% theta3 always stays 0.

%%%% get hip position and velocities %%%
l1 = parms.l1;
l2 = parms.l2;
w = parms.w;
z_temp = [0 0 0 0 0 0 z0]; %initialize x, xd, y, yd, z, zd to zero
[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ....
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(z_temp);
[pos_hip_l_stance,pos_hip_r_stance] = hip_positions(l1,l2,phi,phi_lh,phi_rh,psi_lh,psi_rh,psi,theta,theta_lh,theta_lk,theta_rh,theta_rk,w);
[vel_hip_l_stance,vel_hip_r_stance] = hip_velocities(l1,l2,phi,phid,phi_lh,phi_rh,phi_lhd,phi_rhd,psid,psi_lh,psi_rh,psi,psi_lhd,psi_rhd,theta,thetad,theta_lh,theta_lk,theta_rh,theta_rk,theta_lhd,theta_lkd,theta_rhd,theta_rkd,w);

if (strcmp(parms.stance_foot,'r'))
    x = pos_hip_r_stance(1); 
    xd = vel_hip_r_stance(1);
    y = pos_hip_r_stance(2);  
    yd = vel_hip_r_stance(2); 
    z = pos_hip_r_stance(3);  
    zd = vel_hip_r_stance(3);
elseif (strcmp(parms.stance_foot,'l'))
    x = pos_hip__lstance(1); 
    xd = vel_hip_l_stance(1);
    y = pos_hip_l_stance(2);  
    yd = vel_hip_l_stance(2); 
    z = pos_hip_l_stance(3);  
    zd = vel_hip_l_stance(3);
else
    error('parms.stance_foot needs to be set to l or r');
end
z0 = [x xd y yd z zd z0];  %initialize x, xd, y, yd, z, zd to zero

t_ode = 0;
z_ode = z0;
tf = 0;

P_LA_all = [];
P_RA_all = [];
Torque = [];
for i=1:steps
    %%%%%%%%%% mid stance to before foot strike %%%%%
    options=odeset('abstol',tol,'reltol',tol,'events',@collision);
    t0 = 0;
    t1 = 2;
    [x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ... 
    phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...  
    phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(z0); 
           
    tspan = linspace(t0,t1);
    parms.control.t0 = 0; 
    parms.control.tf = 0.2;
     if (strcmp(parms.stance_foot,'r'))
        parms.control.s0 = [phi, theta, psi, phi_lh, theta_lh, psi_lh, theta_lk, theta_rk];
        parms.control.v0 = [phid, thetad, psid, phi_lhd, theta_lhd, psi_lhd, theta_lkd, theta_rkd];  
    elseif   (strcmp(parms.stance_foot,'l'))
        parms.control.s0 = [phi, theta, psi, phi_rh, theta_rh, psi_rh, theta_lk, theta_rk];
        parms.control.v0 = [phid, thetad, psid, phi_rhd, theta_rhd, psi_rhd, theta_lkd, theta_rkd];
    end 
    parms.control.sf = [0 0 0 0 parms.control.stepAngle 0 0 0];
    parms.control.vf = [0 0 0 0 0 0 0 0];
    parms.control.a0 = [0 0 0 0 0 0 0 0]; 
    parms.control.af = [0 0 0 0 0 0 0 0];
    
    [t_temp1, z_temp1] = ode113(@single_stanceMEX,tspan,z0,options,parms);
%      disp(['time mid to bhs = ', num2str(t_temp1(end))]);
%      disp(['predicted = ',num2str(t1_pred)]);
    
    %%%%%%%%%%%%% reaction forces %%%%%%%%%%%%%%
    %tau = [];
    if (i==1) %only for the first step take the first index
        j = 1;
        [~,~,~,P_LA,P_RA,tau] = single_stanceMEX(t_temp1(j),z_temp1(j,:),parms);
        P_LA_all = [P_LA_all; P_LA];
        P_RA_all = [P_RA_all; P_RA];
        Torque = [Torque; tau'];
    end
    for j=2:length(t_temp1)
        [~,~,~,P_LA,P_RA,tau] = single_stanceMEX(t_temp1(j),z_temp1(j,:),parms);
        P_LA_all = [P_LA_all; P_LA];
        P_RA_all = [P_RA_all; P_RA];
        Torque = [Torque; tau'];
        %P_LA_all(j,:,i) = P_LA; %jth item at step i
        %P_RA_all(j,:,i) = P_RA;
        %Torque(j,:,i) = tau;
    end
    t_temp1 = tf+t_temp1;
    tf = t_temp1(end);
   

   
    %%%%%% foot strike: before to after foot strike %%%%%%%%%%
    parms.control.P = parms.control.Impulse;
    [zplus,I_LA,I_RA]=footstrike(t_temp1(end),z_temp1(end,:),parms);
    
%     %%%%% swap legs %%%
    if (strcmp(parms.stance_foot,'r'))
        parms.stance_foot = 'l';
    elseif (strcmp(parms.stance_foot,'l'))
        parms.stance_foot = 'r';
    end

    %%%%%%%%%%  after foot strike to midstance %%%%%
    z0 = zplus;
    %options=odeset('abstol',1e-13,'reltol',1e-13);
    options=odeset('abstol',tol,'reltol',tol,'events',@midstance);
    t0 = 0;
    t1 = 2;
    tspan = linspace(t0,t1);

    [x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ...  
    phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...  
    phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(z0); 
       
    parms.control.t0 = 0; 
    parms.control.tf = 0.2;
    if (strcmp(parms.stance_foot,'r'))
        parms.control.s0 = [phi, theta, psi, phi_lh, theta_lh, psi_lh, theta_lk, theta_rk];
        parms.control.v0 = [phid, thetad, psid, phi_lhd, theta_lhd, psi_lhd, theta_lkd, theta_rkd];  
        parms.control.sf = [0 0 0 0 0 0 parms.control.kneeAngle 0];
    elseif   (strcmp(parms.stance_foot,'l'))
        parms.control.s0 = [phi, theta, psi, phi_rh, theta_rh, psi_rh, theta_lk, theta_rk];
        parms.control.v0 = [phid, thetad, psid, phi_rhd, theta_rhd, psi_rhd, theta_lkd, theta_rkd];
         parms.control.sf = [0 0 0 0 0 0 0 parms.control.kneeAngle];
    end 
    parms.control.vf = [0 0 0 0 0 0 0 0];
    parms.control.a0 = [0 0 0 0 0 0 0 0]; 
    parms.control.af = [0 0 0 0 0 0 0 0];
    [t_temp2, z_temp2] = ode113(@single_stanceMEX,tspan,z0,options,parms);
%     disp(['time ahs to mid = ', num2str(t_temp2(end))]);
%     disp(['predicted = ',num2str(t2_pred)]);

     
    %%%%%%%%%%%%% reaction forces %%%%%%%%%%%%%%
    for j=2:length(t_temp2)
        [~,~,~,P_LA,P_RA,tau] = single_stanceMEX(t_temp2(j),z_temp2(j,:),parms);
%         P_LA_all(j,:,i) = P_LA; %jth item at step i
%         P_RA_all(j,:,i) = P_RA;
%         Torque(j,:,i) = tau;

        
        P_LA_all = [P_LA_all; P_LA];
        P_RA_all = [P_RA_all; P_RA];
        Torque = [Torque; tau'];
    end

    t_temp2 = tf+t_temp2;
    tf = t_temp2(end);
    
    z0 = z_temp2(end,:);
    
    t_ode = [t_ode; t_temp1(2:end); t_temp2(2:end)];
    z_ode = [z_ode; z_temp1(2:end,:); z_temp2(2:end,:)];
end

z = z0(7:end);

if flag==1
   z = z_ode;
   t = t_ode;
end
