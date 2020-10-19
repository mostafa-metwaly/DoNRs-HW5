function draw_myrobot(L,q_in)
% FK=simplify(Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3))

L1=1;
L2=1;
L3=1;
 
q1 = q_in(:,1);
q2 = q_in(:,2);
q3 = q_in(:,3);

 
hold on
view([225.95 14.40])
grid on
axis([-5 5 -inf inf])
title('RRR elbow Robot')
T1=Rz(q1)*Tz(L1);
plot3(0,0,0,'ro','MarkerSize',5,'LineWidth', 5);
plot3([0 T1(1,4)],[0 T1(2,4)],[0 T1(3,4)],'-b','LineWidth', 5);
plot3(T1(1,4),T1(2,4),T1(3,4),'ro','MarkerSize',5,'LineWidth', 5);

T2=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2);
plot3([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)],'-b','LineWidth', 5);
plot3(T2(1,4),T2(2,4),T2(3,4),'ro','MarkerSize',5,'LineWidth', 5);

T3=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3);
plot3([T2(1,4) T3(1,4)],[T2(2,4) T3(2,4)],[T2(3,4) T3(3,4)],'-b','LineWidth', 5);
% plot3(T3(1,4),T3e(2,4),T3(3,4),'ro','MarkerSize',5,'LineWidth',5); 

T4x=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3)*Tx(0.2);
quiver3(T3(1,4),T3(2,4),T3(3,4),T4x(1,4)-T3(1,4),T4x(2,4)-T3(2,4),T4x(3,4)-T3(3,4),0,'-p')
T4y=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3)*Ty(0.2);
quiver3(T3(1,4),T3(2,4),T3(3,4),T4y(1,4)-T3(1,4),T4y(2,4)-T3(2,4),T4y(3,4)-T3(3,4),0,'-c')
T4z=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3)*Tz(0.2);
quiver3(T3(1,4),T3(2,4),T3(3,4),T4z(1,4)-T3(1,4),T4z(2,4)-T3(2,4),T4z(3,4)-T3(3,4),0,'-g')


