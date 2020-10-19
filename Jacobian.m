function J=  Jacobian(q_in)

L1=1;
L2=1;
L3=1;
q1 = q_in(:,1);
q2 = q_in(:,2);
q3 = q_in(:,3);


% Jq =[ -sin(q1)*(cos(q2 + q3) + cos(q2)), -cos(q1)*(sin(q2 + q3) + sin(q2)), -sin(q2 + q3)*cos(q1)
%         cos(q1)*(cos(q2 + q3) + cos(q2)), -sin(q1)*(sin(q2 + q3) + sin(q2)), -sin(q2 + q3)*sin(q1)
%                                  0,          - cos(q2 + q3) - cos(q2),         -cos(q2 + q3)
%                                  0,                          -sin(q1),              -sin(q1)
%                                  0,                           cos(q1),               cos(q1)
%                                  1,                                 0,                     0];
                             
 J=[ -sin(q1)*(cos(q2 + q3) + cos(q2)), -cos(q1)*(sin(q2 + q3) + sin(q2)), -sin(q2 + q3)*cos(q1)
        cos(q1)*(cos(q2 + q3) + cos(q2)), -sin(q1)*(sin(q2 + q3) + sin(q2)), -sin(q2 + q3)*sin(q1)
                                 0,          - cos(q2 + q3) - cos(q2),         -cos(q2 + q3)];
                             
                             
% syms q1 q2 q3   real
%  % forward kinematics
% H = Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3);
% H=simplify(H);
% % extract rotation matrix
% R = simplify(H(1:3,1:3));
% % diff by q1
% Td=Rzd(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ry(q3)*Tx(L3)*...
%     [R^-1 zeros(3,1);0 0 0 1];
% % extract 6 components from 4x4 Td matrix to Jacobian 1st column
% J1 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' 
% % diff by q2
% Td=Rz(q1)*Tz(L1)*Ryd(q2)*Tx(L2)*Ry(q3)*Tx(L3)*...
%     [R^-1 zeros(3,1);0 0 0 1];
% % extract 6 components from 4x4 Td matrix to Jacobian 2nd column
% J2 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' 
% % diff by q3
% Td=Rz(q1)*Tz(L1)*Ry(q2)*Tx(L2)*Ryd(q3)*Tx(L3)*...
%     [R^-1 zeros(3,1);0 0 0 1];
% % extract 6 components from 4x4 Td matrix to Jacobian 2nd column
% J3 = [Td(1,4), Td(2,4), Td(3,4), Td(3,2), Td(1,3), Td(2,1)]' 
%  
% % Full Jacobian 3x3
% Jq = [simplify(J1), simplify(J2), simplify(J3)]
% 

