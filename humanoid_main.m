clc
clear all
close all
format long

%%%%%%%%%%% 0) initialization %%%%%%%%
l0 = 1;
l1 = 0.5;
l2 = 0.5;
w = 0.1;

mb = 70; mt = 10; mc = 5;
Ibx = 5; Iby = 3; Ibz = 2;
Itx = 1; Ity = 0.3; Itz = 2;%1
Icx = 0.5; Icy = 0.15; Icz = 1; %0.5

g = 9.8;
parms.mb = mb; parms.mt = mt; parms.mc = mc;
parms.Ibx = Ibx; parms.Iby = Iby; parms.Ibz = Ibz;
parms.Itx = Itx; parms.Ity = Ity; parms.Itz = Itz;
parms.Icx = Icx; parms.Icy = Icy; parms.Icz = Icz;
parms.g = g;
parms.l0 = l0; parms.l1 = l1; parms.l2 = l2; parms.w = w;


M = parms.mb+2*parms.mt+2*parms.mc;
L = parms.l1+parms.l2;
parms.control.stepAngle = 0.375; %0.375; %step length
parms.control.Impulse = 0.18*M*sqrt(g*L); %push-off impulse
parms.control.kneeAngle = -1; %Knee bending to avoid scuffing
parms.control.Kp = 100; %gain for partial feedback linearization

parms.stance_foot_init = 'r';

phi = 0;
theta = 0;
psi = 0; 
psid = 0;  
thetad = 0.5;  
phid = 0; 

psi_lh = 0; 
phi_lh = 0; 
theta_lh = 0;
psi_rh = 0; 
phi_rh = 0;
theta_rh = 0; 
theta_lk = 0; 
theta_rk = 0; 
psi_lhd = 0;
phi_lhd = 0;  
theta_lhd = 0;  
psi_rhd = 0;  
phi_rhd = 0;  
theta_rhd =0; 
theta_lkd = 0; 
theta_rkd = 0;

if (strcmp(parms.stance_foot_init,'r')) %right foot is stance
    theta_lk = parms.control.kneeAngle;
elseif (strcmp(parms.stance_foot_init,'l'))
    theta_rk = parms.control.kneeAngle;
else
    error(' parms.stance_foot should be l or r');
end

z0 = [  phi phid theta thetad psi psid ...
        phi_lh phi_lhd theta_lh theta_lhd psi_lh psi_lhd theta_lk theta_lkd ...
        phi_rh phi_rhd theta_rh theta_rhd psi_rh psi_rhd theta_rk theta_rkd];
 
%%%%%%%%%%% initialization done %%%%%%%%
% 
% %%%%% 1) find fixed points %%%%%%%
% %%% Root finding, Period one gait %%%%
% options = optimset('TolFun',1e-8,'TolX',1e-8,'Display','iter');
% [zstar,fval,exitflag] = fsolve(@fixedpt,z0,options,parms);
% if exitflag == 1
%     disp('Fixed point:');
%     %disp(zstar);
% else
%     error('Root finder not converged, change guess or change system parameters')
% end
% zstar'


%%%%%%% straight line with r leg as stance leg %%%
zstar = [ 0.000000000000000
   0.000000000000000
  -0.000000000000000
   0.000000000000001
  -0.000000000000006
  -0.000000000000003
   0.000000000000006
   0.000000000000043
  -0.000000000000002
   0.000000000000070
  -0.000000000000013
  -0.000000000000057
  -0.999999999999981
   0.000000000000061
  -0.029247773468329
   0.054577886084082
  -0.001551040824746
  -1.029460778006675
   0.054690088280553
   0.559306810759302
  -0.000000000000001
  -0.000000000000031]';
% 
% %%%%%% 2) find linearized stability %%%%%%
% disp('Computing J of the map and eigenvalues');
% J=partialder(@onestep,zstar,parms);
% eigJ = eig(J);
% disp('The norm of eigenvalues are:')
% for i=1:length(eigJ)
%     disp(norm(eigJ(i)))
% end


%%%%%%%%%%% 3) forward simulation  %%%%%%%%   
steps = 8;
parms.stance_foot = parms.stance_foot_init;
[Z,t,P_LA_all,P_RA_all,Torque]=onestep(zstar,parms,steps);

disp('----- start state --------- end state ----');
disp([zstar' Z(end,7:end)']);

%%%%%%%%%%% 4) animation and plotting  %%%%%%%%  
fps = 50;
figure(1)
title('animation');
% view([0 0]);
% view([71 17]);
view([120 54]);
animate(t,Z,parms,fps,view)


[x, xd, y, yd, z, zd, phi, phid, theta, thetad, psi, psid, ....
phi_lh, phi_lhd, theta_lh, theta_lhd, psi_lh, psi_lhd, theta_lk, theta_lkd, ...
phi_rh, phi_rhd, theta_rh, theta_rhd, psi_rh, psi_rhd, theta_rk, theta_rkd]= getstate(Z);

figure(2)
subplot(2,1,1)
plot(t,x,'r'); hold on;
plot(t,y,'b');
plot(t,z,'g');
title('body absolute positions');
subplot(2,1,2)
plot(t,xd,'r'); hold on;
plot(t,yd,'b');
plot(t,zd,'g');
title('body absolute rates');
xlabel('time');
legend('x','y','z');

figure(3)
subplot(2,1,1)
plot(t,phi,'r'); hold on;
plot(t,theta,'b');
plot(t,psi,'g');
title('body absolute angles');
subplot(2,1,2)
plot(t,phid,'r'); hold on;
plot(t,thetad,'b');
plot(t,psid,'g');
legend('phi','theta','psi');
xlabel('time');
title('body absolute rates');

figure(4)
subplot(2,1,1)
plot(t,phi_lh,'r'); hold on;
plot(t,theta_lh,'b');
plot(t,psi_lh,'g');
plot(t,theta_lk,'k');
title('left angles');
subplot(2,1,2)
plot(t,phi_lhd,'r'); hold on;
plot(t,theta_lhd,'b');
plot(t,psi_lhd,'g');
plot(t,theta_lkd,'k');
xlabel('time');
legend('phi-hip','theta-hip','psi-hip','theta-knee');
title('left rates');

figure(5)
subplot(2,1,1)
plot(t,phi_rh,'r'); hold on;
plot(t,theta_rh,'b');
plot(t,psi_rh,'g');
plot(t,theta_rk,'k');
title('right angles');
subplot(2,1,2)
plot(t,phi_rhd,'r'); hold on;
plot(t,theta_rhd,'b');
plot(t,psi_rhd,'g');
plot(t,theta_rkd,'k');
legend('phi-hip','theta-hip','psi-hip','theta-knee');
title('right rates');
xlabel('time');

figure(6)
subplot(2,1,2)
plot(t,P_RA_all(:,1),'r'); hold on
plot(t,P_RA_all(:,2),'g'); 
plot(t,P_RA_all(:,3),'b');
title('right leg')
ylabel('reaction force');
subplot(2,1,1)
plot(t,P_LA_all(:,1),'r'); hold on
plot(t,P_LA_all(:,2),'g'); 
plot(t,P_LA_all(:,3),'b'); 
ylabel('reaction force');
title('left leg')
legend('x','y','z');
xlabel('time');

figure(7)
subplot(2,1,1);
plot(t,Torque(:,1),'r'); hold on;
plot(t,Torque(:,2),'g'); 
plot(t,Torque(:,3),'b'); 
plot(t,Torque(:,4),'k'); 
title('left leg')
ylabel('Torque');
subplot(2,1,2);
plot(t,Torque(:,5),'r'); hold on;
plot(t,Torque(:,6),'g'); 
plot(t,Torque(:,7),'b'); 
plot(t,Torque(:,8),'k'); 
title('right leg')
ylabel('Torque');
legend('hip-phi','hip-theta','hip-psi','knee-theta');
xlabel('time');
