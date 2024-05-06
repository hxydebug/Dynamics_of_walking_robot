%===================================================================
function J=partialder(FUN,z,parms)
%===================================================================
pert=1e-5;
n = length(z);
J = zeros(n,n);

%%%% Using forward difference, accuracy linear %%%
% y0=feval(FUN,z,walker); 
% for i=1:n
%     ztemp=z;
%     ztemp(i)=ztemp(i)+pert; 
%     J(:,i)=(feval(FUN,ztemp,walker)-y0) ;
% end
% J=(J/pert);

%%% Using central difference, accuracy quadratic %%%
for i=1:n
    ztemp1=z; ztemp2=z;
    ztemp1(i)=ztemp1(i)+pert; 
    ztemp2(i)=ztemp2(i)-pert; 
    parms.stance_foot = parms.stance_foot_init;
    Ztemp1 = feval(FUN,ztemp1,parms);
    parms.stance_foot = parms.stance_foot_init;
    Ztemp2 = feval(FUN,ztemp2,parms);
    J(:,i)=Ztemp2-Ztemp1;
end
J=J/(2*pert);