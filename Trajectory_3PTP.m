%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mostafa Osama M.othman@innopolis.university
% DoNRS course Home Assignment 4, task 3
% Solution for 3-dof robot:
% In this example, compute the trajectory for 3 joints of 3-dof robotic arm.   
% the controller frequency is 100 hz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all 
close all;clc;
q0 = [0 0 0]; qf = [2 3 4]; v_max = [1 1 1]; a_max = [10 10 10];
v0= [0 0 0];
dt = 0.01;
n = 0;

while (floor(dt*10^n)~=dt*10^n)
    n=n+1;
end
E = 1*10^-n;
ta = v_max./a_max;

delta_q = (qf(:)-q0(:));
tf = (delta_q(:))./v_max + ta;
taw=delta_q ./v_max;

if rem(ta,dt)~=0
    ta_new = round(ta,n)+E;
else
    ta_new = round(ta,n);
end
 tf = (qf(:)-q0(:))./v_max(:)  + ta_new(:);

if rem(tf,dt)~=0
    tf_new = round(tf,n)+E;
else
    tf_new = round(tf,n);
end

ta_new=max(ta_new)
tf_new=max(tf_new)

v_old =((qf(:)-q0(:))./(tf-ta));
a_old = v_old/ta;

v = ((qf(:)-q0(:))./(tf_new-ta_new))
a = v/ta_new


% all joints - coefficients:
for i =1:3

% t0 --> ta:
a10 = q0(i);
a11 = v0(i);
a12 = 0.5*a(i);

% ta --> tf-ta:
a20 = q0(i) + 0.5*a(i)*ta_new^2 - v(i)*ta_new;
a21 = v(i);

% tf-ta --> tf:
a30 = qf(i) - 0.5*a(i)*tf_new^2;
a31 = a(i)*tf_new;
a32 = -0.5*a(i);

 b(i,:) = [a10; a11; a12; a20; a21; a30; a31; a32];
end

t = 0:dt:tf_new;
v=zeros(3,length(t)); 

for i=1:3
    
q(i,:) = (b(i,1)+b(i,2).*t+b(i,3).*t.^2).*(t<=ta_new)...
    +(b(i,4)+b(i,5).*t).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(b(i,6)+b(i,7).*t+b(i,8).*t.^2).*(t>(tf_new-ta_new)).*(t<=tf_new);

v(i,:) = (b(i,2)+2*b(i,3).*t).*(t<=ta_new)...
    +(b(i,5)).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(b(i,7)+2*b(i,8).*t) .*(t>(tf_new-ta_new)).*(t<=tf_new);

acc(i,:) = (2*b(i,3)).*(t<=ta_new)...
    +(0).*(t>ta_new).*(t<=(tf_new-ta_new))...
    +(2*b(i,8)).*(t>(tf_new-ta_new)).*(t<=tf_new);

end
figure;

hold on
for i=1:2:length(q)
   draw_myrobot([1 1 1],q(:,i)')
   pause(0.01)
   cla
  end   
 draw_myrobot([1 1 1],q(:,i)')

figure
subplot(1,3,1)
plot(t,q(1,:),'b-','linewidth',2)
hold on
plot(t,q(2,:),'g.','linewidth',2)
hold on
plot(t,q(3,:),'r--','linewidth',2)
grid on
title('position vs time')
legend('joint_1','joint_2','joint_3')
axis([0 tf_new -inf inf])

subplot(1,3,2)
plot(t,v(1,:),'b-','linewidth',2)
hold on
plot(t,v(2,:),'g.','linewidth',2)
hold on
plot(t,v(3,:),'r--','linewidth',2)
title('velocity vs time')
legend('joint_1','joint_2','joint_3')
grid on
axis([0 tf_new -inf inf])

subplot(1,3,3)
plot(t,acc(1,:),'b-','linewidth',2)
hold on
plot(t,acc(2,:),'g.','linewidth',2)
hold on
plot(t,acc(3,:),'r--','linewidth',2)
title('acceleration vs time')
legend('joint_1','joint_2','joint_3')
grid on
axis([0 tf_new -inf inf])

figure;
subplot(3,3,1);plot(t,q(1,:),'b-','linewidth',2);title('position vs time');legend('J_1');
subplot(3,3,2);plot(t,q(2,:),'g-','linewidth',2);title('position vs time');legend('J_2');
subplot(3,3,3);plot(t,q(3,:),'r-','linewidth',2 );title('position vs time');legend('J_3');

subplot(3,3,4);plot(t,v(1,:),'b-','linewidth',2);title('velocity vs time');legend('J_1');
subplot(3,3,5);plot(t,v(2,:),'g-','linewidth',2);title('velocity vs time');legend('J_2');
subplot(3,3,6);plot(t,v(3,:),'r-','linewidth',2 );title('velocity vs time');legend('J_3');

subplot(3,3,7);plot(t,acc(1,:),'b-','linewidth',2);title('acceleration vs time');legend('J_1');
subplot(3,3,8);plot(t,acc(2,:),'g-','linewidth',2);title('acceleration vs time');legend('J_2');
subplot(3,3,9);plot(t,acc(3,:),'r-','linewidth',2 );title('acceleration vs time');legend('J_3');
hold off


figure
subplot(2,3,1:3)
plot(t,q(1,:),'b-','linewidth',2)
hold on
plot(t,q(2,:),'g.','linewidth',2)
hold on
plot(t,q(3,:),'r--','linewidth',2)
grid on
title('position vs time')
legend('joint_1','joint_2','joint_3')
axis([0 tf_new -inf inf])

subplot(2,3,4);plot(t,q(1,:),'b-','linewidth',2);sgtitle('position vs time');legend('J_1');
subplot(2,3,5);plot(t,q(2,:),'g-','linewidth',2);sgtitle('position vs time');legend('J_2');
subplot(2,3,6);plot(t,q(3,:),'r-','linewidth',2 );sgtitle('position vs time');legend('J_3');


figure;
subplot(2,3,1:3)
plot(t,v(1,:),'b-','linewidth',2)
hold on
plot(t,v(2,:),'g.','linewidth',2)
hold on
plot(t,v(3,:),'r--','linewidth',2)
title('velocity vs time')
legend('joint_1','joint_2','joint_3')
grid on
axis([0 tf_new -inf inf])

subplot(2,3,4);plot(t,v(1,:),'b-','linewidth',2);sgtitle('velocity vs time');legend('J_1');
subplot(2,3,5);plot(t,v(2,:),'g-','linewidth',2);sgtitle('velocity vs time');legend('J_2');
subplot(2,3,6);plot(t,v(3,:),'r-','linewidth',2 );sgtitle('velocity vs time');legend('J_3');

figure;
subplot(2,3,1:3)
plot(t,acc(1,:),'b-','linewidth',2)
hold on
plot(t,acc(2,:),'g.','linewidth',2)
hold on
plot(t,acc(3,:),'r--','linewidth',2)
title('acceleration vs time')
legend('joint_1','joint_2','joint_3')

subplot(2,3,4);plot(t,acc(1,:),'b-','linewidth',2);sgtitle('acceleration vs time');legend('J_1');
subplot(2,3,5);plot(t,acc(2,:),'g-','linewidth',2);sgtitle('acceleration vs time');legend('J_2');
subplot(2,3,6);plot(t,acc(3,:),'r-','linewidth',2 );sgtitle('acceleration vs time');legend('J_3');
grid on
axis([0 tf_new -inf inf])
