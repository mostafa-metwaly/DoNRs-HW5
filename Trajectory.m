% Trajectory
% define the initial and final conditions
clear all
close all

t0 = 0; tf = 2;
q0 = [0 0 0]; qf = [2 3 4]; 
v0 = 0; vf = 0;
acc0 = 0; accf = 0;
tf = 2;
q_all=zeros(21,3);
v_all= zeros(21,3);
acc_all=zeros(21,3); 

for i=1:3
%Given:
A = [1 t0 t0^2 t0^3 t0^4 t0^5
     0 1 2*t0 3*t0^2 4*t0^3 5*t0^4
     0 0 2 6*t0 12*t0^2 20*t0^3
     1 tf tf^2 tf^3 tf^4 tf^5
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4
     0 0 2 6*tf 12*tf^2 20*tf^3];
c = [q0(i);v0;acc0;qf(i);vf;accf]
b = A\c
% assign the results to the coefficients 
a0 = b(1); a1 = b(2); a2 = b(3); a3 = b(4); a4 = b(5); a5 = b(6);


t = 0:0.1:2;
q = a0+a1.*t+a2.*t.^2+a3.*t.^3+a4.*t.^4+a5.*t.^5
v = a1+2*a2.*t+3*a3.*t.^2+4*a4.*t.^3+5*a5.*t.^4;
acc = 2*a2+6*a3.*t+12*a4.*t.^2+20*a5.*t.^3;

q_all(:,i) = q
v_all(:,i) = v;
acc_all(:,i)  = acc ;
end

figure
plot(t,q_all(:,1),'r-',t,q_all(:,2),'b--',t,q_all(:,3),'g-')
title('position vs time')
grid on
figure
plot(t,v_all(:,1),'r-',t,v_all(:,2),'b--',t,v_all(:,3),'g-')
title('velocity vs time')
grid on
figure
plot(t,acc_all(:,1),'r-',t,acc_all(:,2),'b--',t,acc_all(:,3),'g-')
title('acceleration vs time')
grid on
 