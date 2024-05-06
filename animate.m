%===================================================================
function animate(t_all,z_all,parms,fps,view)
%===================================================================


% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

l0=parms.l0; l1=parms.l1; l2=parms.l2; w=parms.w;

if (nargin<5)
    view([0 0]);
end
if (nargin<4)
    fps = 50;
end

[m,n] = size(z_all); 
%%%% Interpolate linearly using fps %%%%%
% z = [x xd y yd z zd phi phid theta thetad psi psid ...
%         phi_lh phi_lhd theta_lh theta_lhd psi_lh psi_lhd theta_lk theta_lkd ...
%         phi_rh phi_rhd theta_rh theta_rhd psi_rh psi_rhd theta_rk theta_rkd];


z_all_plot = [];
for i=1:2:n
    z_all_plot = [z_all_plot, z_all(:,i)];
end

if (m~=1)
    nn = size(z_all_plot,2);
    total_frames = round(t_all(end)*fps);
    t = linspace(0,t_all(end),total_frames);
    zz = zeros(total_frames,nn);
    for i=1:nn
        zz(:,i) = interp1(t_all,z_all_plot(:,i),t);
    end
else
     zz = z_all_plot;
end

%%%%% Now animate the results %%%%%%%  
mm = size(zz,1);

% axis('equal')
% axis([window_xmin window_xmax window_ymin window_ymax])
% axis off
% set(gcf,'Color',[1,1,1])

 
for i=1:mm
    j = 1;
    x = zz(i,j); j=j+1;
    y = zz(i,j);j=j+1;
    z = zz(i,j);j=j+1;
    phi = zz(i,j); j=j+1;
    theta = zz(i,j);j=j+1;
    psi = zz(i,j);j=j+1;
    phi_lh = zz(i,j); j=j+1;
    theta_lh = zz(i,j);j=j+1;
    psi_lh = zz(i,j);j=j+1;
    theta_lk = zz(i,j);j=j+1;
    phi_rh = zz(i,j); j=j+1;
    theta_rh = zz(i,j);j=j+1;
    psi_rh = zz(i,j);j=j+1;
    theta_rk = zz(i,j);%j=j+1;

    [B,H,LH,LK,LA,RH,RK,RA,b,rt,rc,lt,lc] = joint_locations(l0,l1,l2,phi,phi_lh,phi_rh,psi_lh,psi_rh,psi,theta,theta_lh,theta_lk,theta_rh,theta_rk,w,x,y,z);


    
    loc1 = B; loc2 = H;
    k1=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','r'); hold on;

    loc1 = B; loc2 = LH;
    k2=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','b'); hold on;

    loc1 = B; loc2 = RH;
    k3=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','b'); hold on;

    loc1 = LH; loc2 = LK;
    k4=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','c'); hold on;

    loc1 = RH; loc2 = RK;
    k5=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','c'); hold on;

    loc1 = LK; loc2 = LA;
    k6=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','m'); hold on;

    loc1 = RK; loc2 = RA;
    k7=line([loc1(1) loc2(1)],[loc1(2) loc2(2)],[loc1(3) loc2(3)],'Linewidth',5,'Color','m'); hold on;

    pt = b;
    k8=plot3(pt(1),pt(2),pt(3),'ko','Markersize',20,'MarkerFaceColor','k');

    pt = rt;
    k9=plot3(pt(1),pt(2),pt(3),'ko','Markersize',10,'MarkerFaceColor','k');

    pt = rc;
    k10=plot3(pt(1),pt(2),pt(3),'ko','Markersize',10,'MarkerFaceColor','k');

    pt = lt;
    k11=plot3(pt(1),pt(2),pt(3),'ko','Markersize',10,'MarkerFaceColor','k');

    pt = lc;
    k12=plot3(pt(1),pt(2),pt(3),'ko','Markersize',10,'MarkerFaceColor','k');

    xlabel('x'); ylabel('y'); zlabel('z');
    %axis('equal');
    axis(2*[0 2 -1 1 0 1]) 
    %axis([-XY_max XY_max -XY_max XY_max 0 2])
    %axis('equal')
    

   %delay to create animation
   pause(0.01);
   
   if (i~=mm)     %delete all but not the last entry
       delete(k1);
       delete(k2);
       delete(k3);
       delete(k4);
       delete(k5);
       delete(k6);
       delete(k7);
       delete(k8);
       delete(k9);
       delete(k10);
       delete(k11);
       delete(k12);
   end
   
end

