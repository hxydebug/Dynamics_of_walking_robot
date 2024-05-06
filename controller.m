function tau = controller(t,Z,parms) 

[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ...  
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ... 
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(Z); 

[mb,mt,mc,Ibx,Iby,Ibz,Itx,Ity,Itz,Icx,Icy,Icz,l0,l1,l2,w,g] = getparms(parms); 

params = [mb mt mc Ibx Iby Ibz Itx Ity Itz Icx Icy Icz l0 l1 l2 w g]; 

[~,nn] = size(Z);
if (nn==1) 
    Z = Z';
end

t0 = parms.control.t0;
tf = parms.control.tf;
s0 = parms.control.s0;
sf = parms.control.sf;
v0 = parms.control.v0;
vf = parms.control.vf;
a0 = parms.control.a0;
af = parms.control.af;

for i=1:8
    if (t>tf)
        t = tf;
    end
    [theta_temp(i),omega_temp(i),alpha_temp(i)] = traj(t,t0,tf,s0(i),sf(i),v0(i),vf(i),a0(i),af(i));
end
Xdd_des = alpha_temp';
Xd_des = omega_temp';
X_des = theta_temp';

B = [0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0; %3 for linear pos global
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0; %3 for ang pos global
     1 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0; %6
     0 0 0 1 0 0 0 0;
     0 0 0 0 1 0 0 0;
     0 0 0 0 0 1 0 0; %9
     0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 1;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0;
     0 0 0 0 0 0 0 0]; %3 for P's
S_L = [0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0; %phi
       0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0; %theta
       0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0; %psi
       0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0; %phi_rh
       0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0; %theta_rh
       0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0; %psi_rh
       0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0; %theta_lk
       0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0]; %theta_rk;
S_R = [0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0; %phi
       0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0; %theta
       0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0; %psi
       0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0; %phi_lh
       0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0; %theta_lh
       0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0; %psi_lh
       0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0; %theta_lh
       0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0]; %theta_rk;
   
[M,N,J_l,J_r,Jdot_l,Jdot_r] = gateway_dynamicsMEX(t,Z,params); 

M = M'; 
J_l = J_l'; J_r = J_r'; 
Jdot_l = Jdot_l'; Jdot_r = Jdot_r'; 
qdot = [xd yd zd phid thetad psid phi_lhd theta_lhd psi_lhd theta_lkd phi_rhd theta_rhd psi_rhd theta_rkd]; 

%%%%% 1) get the equation %%%%
%%%%% A X = b; %without control
if (strcmp(parms.stance_foot,'r'))
    A = [M, -J_r'; J_r, zeros(3,3)];
    b = [N; -Jdot_r*qdot'];
elseif (strcmp(parms.stance_foot,'l'))
    A = [M, -J_l'; J_l, zeros(3,3)];
    b = [N; -Jdot_l*qdot'];
else
   error('parms.stance_foot needs to be set to l or r');
end

%%%%% Now AX = b + B*tau %with control
%%%%% Reduced X_c = S X = S*inv(A) (b+B*tau) = v
%%%%%% Solving for tau gives, tau = inv(S*inv(A)*B)*(v - S*inv(A)*b)
Kp = parms.control.Kp;
Kd = sqrt(Kp);
Ainv = A \ eye(length(A));
if (strcmp(parms.stance_foot,'r'))
    X = [phi, theta, psi, phi_lh, theta_lh, psi_lh, theta_lk, theta_rk]';
    Xd = [phid, thetad, psid, phi_lhd, theta_lhd, psi_lhd, theta_lkd, theta_rkd]';
    v = Xdd_des + Kd*(Xd_des-Xd) + Kp*(X_des-X);
    SAinvB = S_R*Ainv*B;
    SAinvB_inv = S_R*Ainv*B \ eye(length(SAinvB));
    tau = SAinvB_inv*(v-S_R*Ainv*b);
elseif (strcmp(parms.stance_foot,'l'))
    X = [phi, theta, psi, phi_rh, theta_rh, psi_rh, theta_lk, theta_rk]';
    Xd = [phid, thetad, psid, phi_rhd, theta_rhd, psi_rhd, theta_lkd, theta_rkd]';
    v = Xdd_des + Kd*(Xd_des-Xd) + Kp*(X_des-X);
    SAinvB = S_L*Ainv*B;
    SAinvB_inv = S_L*Ainv*B \ eye(length(SAinvB));
    
    tau = SAinvB_inv*(v-S_L*Ainv*b);
else
   error('parms.stance_foot needs to be set to l or r');
end


