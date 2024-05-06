clc
clear all

syms s0 sf real
syms v0 vf real
syms a0 af real
syms t t0 tf real
syms c0 c1 c2 c3 c4 c5 real

s = [c0 c1 c2 c3 c4 c5]*[1 t t^2 t^3 t^4 t^5]';
v = diff(s,t);
a = diff(v,t);

eq(1) = subs(s,t,t0)-s0;
eq(2) = subs(s,t,tf)-sf;
eq(3) = subs(v,t,t0)-v0;
eq(4) = subs(v,t,tf)-vf;
eq(5) = subs(a,t,t0)-a0;
eq(6) = subs(a,t,tf)-af;

A = jacobian(eq,[c0 c1 c2 c3 c4 c5]);
for i=1:6
    b(i,1) = -subs(eq(i),[c0 c1 c2 c3 c4 c5],[0 0 0 0 0 0]);
end

disp('function [s,v,a] = traj(t,t0,tf,s0,sf,v0,vf,a0,af)');
disp(' ');
for i=1:6
    for j=1:6
        string = ['A(',num2str(i),',',num2str(j),')=',char(simplify(A(i,j))),';'];
        disp(string);
    end
end
disp(' ');
for i=1:6
    for j=1:1
        string = ['b(',num2str(i),',',num2str(j),')=',char(simplify(b(i,j))),';'];
        disp(string);
    end
end

disp(' ');
disp('X = A\b;');
disp(' ');
disp('c0 = X(1); c1 = X(2); c2 = X(3); c3 = X(4); c4 = X(5); c5 = X(6);');
disp(' ');
disp(['s = ',char(s),';']);
disp(['v = ',char(v),';']);
disp(['a = ',char(a),';']);
disp(' ');
