 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mostafa Osama M.othman@innopolis.university
% DoNRS course Home Assignment 4, task 3
% Solution for 3-dof robot:
% In this example, compute the trajectory for 3 joints of 3-dof robotic arm.   
% the controller frequency is 100 hz
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all 
close all;clc;

p1 = [ 1, 0, 1];
p2 = [sqrt(2)/2 sqrt(2)/2 1.2];
v_max = [1 1 1]; a_max = [10 10 10];
v0= [0 0 0];
dt = 0.01;
n = 0; 
N = 100;
 
while (floor(dt*10^n)~=dt*10^n)
    n=n+1;
end
E = 1*10^-n;
ta = v_max./a_max
delta_p = (p2(:)-p1(:));

if rem(ta,dt)~=0
    ta_new = round(ta,n)+E;
else
    ta_new = round(ta,n);
end

 tf = (delta_p(:))./v_max(:)  + ta_new(:);

if rem(tf,dt)~=0
    tf_new = round(tf,n)+E;
else
    tf_new = round(tf,n);
end


ta_new=max(ta_new)
tf_new=max(tf_new)
% v_old =((p2(:)-p1(:))./(tf-ta));
% a_old = v_old/ta;
% 
% v = ((p2(:)-p1(:))./(tf_new-ta_new))
% a = v/ta_new

t_lin = linspace(0,tf_new,N);

% positions
% x = ((p2(1)-p1(1))/tf_new).*t+p1(1);
% y = ((p2(2)-p1(2))/tf_new).*t+p1(2);
% z = ((p2(3)-p1(3))/tf_new).*t+p1(3);
    waypnts = [((p2(:)-p1(:))./tf_new).*t_lin+p1(:)]';
% velocities
% vx = ((p2(1)-p1(1))/tf_new);
% vy = ((p2(2)-p1(2))/tf_new);
% vz = ((p2(3)-p1(3))/tf_new);
    vel = ((p2(:)-p1(:))/tf_new);
m = -1; % 1 elbow up, -1 elbow down         
for i=1:N
    waypnt = waypnts(i,:);
    jointangles(i,:) = InverseKinematics(waypnt);
    if i == 1 || i==N
        jointVelocity(i,:) = [0 0 0];
    else
        J = Jacobian(jointangles(i,:));
        jointVelocity(i,:) =  (J\vel)';
    end
end
% for each joint:
n = 15;
num=0;
Q = [];
Vel = [];
Acc = [];
acc =[];
vel =[];
for i=1:N-1
    t1 = t_lin(i); t2 = t_lin(i+1);
    A = [1 t1 t1^2 t1^3
        0 1 2*t1 3*t1^2
        1 t2 t2^2 t2^3
        0 1 2*t2 3*t2^2];
    for j = 1:3
         q1 = jointangles(i,j); 
         q2 = jointangles(i+1,j);
         v1 = jointVelocity(i,j); 
         v2 = jointVelocity(i+1,j);
         c = [q1;v1;q2;v2];
         b = A\c;
         t = linspace(t1,t2,n);
         q(j,:) = b(1)+b(2).*t+b(3).*t.^2+b(4).*t.^3;
         vel(j,:) = b(2)+2*b(3).*t+3*b(4).*t.^2;
         acc(j,:) = 2* b(3)+6* b(4).*t;
         
    end   
    Q = [Q q];
    Vel = [Vel vel];
    Acc = [Acc acc];
    
end 
cartTrajectory = ForwardKinematics(Q'); 
for i=1:length(Q)
   jac= Jacobian(Q(:,i)');
   CartVelocity(:,i)= jac * Vel(:,i);
   CartAcceleration(i,:)= jac * Acc(:,i);
 

end
CartAcceleration=CartAcceleration';
cartTrajectory=cartTrajectory';
figure;

hold on
for i=1:50:length(Q)
   draw_myrobot([1 1 1],Q(:,i)')
  
   pause(0.01)
   cla
   plot3(waypnts(:,1),waypnts(:,2),waypnts(:,3),'r-','linewidth',1)
  end   
 draw_myrobot([1 1 1],Q(:,i)')
 num=n*N-n;
t  = linspace(0,tf_new,num);

figure
subplot(1,3,1)
plot(t,Q(1,:),'b-','linewidth',2)
hold on
plot(t,Q(2,:),'g-','linewidth',2)
hold on
plot(t,Q(3,:),'r-','linewidth',2)
grid on
title('joints position vs time')
legend('joint_1','joint_2','joint_3')
axis([0 tf_new -inf inf])

subplot(1,3,2)
plot(t,Vel(1,:),'b-','linewidth',2)
hold on
plot(t,Vel(2,:),'g-','linewidth',2)
hold on
plot(t,Vel(3,:),'r-','linewidth',2)
title('joints angular velocity vs time')
legend('joint_1','joint_2','joint_3')
grid on
axis([0 tf_new -inf inf])

subplot(1,3,3)
plot(t,Acc(1,:),'b-','linewidth',2)
hold on
plot(t,Acc(2,:),'g-','linewidth',2)
hold on
plot(t,Acc(3,:),'r-','linewidth',2)
title('joints acceleration vs time')
legend('joint_1','joint_2','joint_3')
grid on
axis([0 tf_new -inf inf])

figure;
subplot(3,3,1);plot(t,cartTrajectory(1,:),'b-','linewidth',2);title('Linear position vs time');legend('X');
subplot(3,3,2);plot(t,cartTrajectory(2,:),'g-','linewidth',2);sgtitle('Linear position vs time');legend('Y');
subplot(3,3,3);plot(t,cartTrajectory(3,:),'r-','linewidth',2 );title('Linear position vs time');legend('Z');

subplot(3,3,4);plot(t,CartVelocity(1,:),'b-','linewidth',2);title('Linear velocity vs time');legend('X');
subplot(3,3,5);plot(t,CartVelocity(2,:),'g-','linewidth',2);sgtitle('Linear velocity vs time');legend('Y');
subplot(3,3,6);plot(t,CartVelocity(3,:),'r-','linewidth',2 );title('Linear velocity vs time');legend('Z');

subplot(3,3,7);plot(t,CartAcceleration(1,:),'b-','linewidth',2);title('Linear acceleration vs time');legend('X');
subplot(3,3,8);plot(t,CartAcceleration(2,:),'g-','linewidth',2);sgtitle('Linear acceleration vs time');legend('Y');
subplot(3,3,9);plot(t,CartAcceleration(3,:),'r-','linewidth',2 );title('Linear acceleration vs time');legend('Z');
hold off


figure
subplot(2,3,1:3)
plot(t,Q(1,:),'b-','linewidth',2)
hold on
plot(t,Q(2,:),'g-','linewidth',2)
hold on
plot(t,Q(3,:),'r-','linewidth',2)
grid on
title('joints position vs time')
legend('joint_1','joint_2','joint_3')
axis([0 tf_new -inf inf])

subplot(2,3,4);plot(t,cartTrajectory(1,:),'b-','linewidth',2);title('Linear position vs time');legend('X');
subplot(2,3,5);plot(t,cartTrajectory(2,:),'g-','linewidth',2);title('Linear position vs time');legend('Y');
subplot(2,3,6);plot(t,cartTrajectory(3,:),'r-','linewidth',2 );title('Linear position vs time');legend('Z');


figure;
subplot(2,3,1:3)
plot(t,Vel(1,:),'b-','linewidth',2)
hold on
plot(t,Vel(2,:),'g-','linewidth',2)
hold on
plot(t,Vel(3,:),'r-','linewidth',2)
title('joints angular velocity vs time')
legend('joint_1','joint_2','joint_3')
grid on
axis([0 tf_new -inf inf])

subplot(2,3,4);plot(t,CartVelocity(1,:),'b-','linewidth',2);title('Linear velocity vs time');legend('X');
subplot(2,3,5);plot(t,CartVelocity(2,:),'g-','linewidth',2);title('Linear velocity vs time');legend('Y');
subplot(2,3,6);plot(t,CartVelocity(3,:),'r-','linewidth',2 );title('Linear velocity vs time');legend('Z');

figure;
subplot(2,3,1:3)
plot(t,Acc(1,:),'b-','linewidth',2)
hold on
plot(t,Acc(2,:),'g-','linewidth',2)
hold on
plot(t,Acc(3,:),'r-','linewidth',2)
title('joints acceleration vs time')
legend('joint_1','joint_2','joint_3')

subplot(2,3,4);plot(t,CartAcceleration(1,:),'b-','linewidth',2);title('Linear acceleration vs time');legend('X');
subplot(2,3,5);plot(t,CartAcceleration(2,:),'g-','linewidth',2);title('Linear acceleration vs time');legend('Y');
subplot(2,3,6);plot(t,CartAcceleration(3,:),'r-','linewidth',2 );title('Linear acceleration vs time');legend('Z');
grid on
axis([0 tf_new -inf inf])