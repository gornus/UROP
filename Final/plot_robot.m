function[] = plot_robot(point, centrex2, linkl, rel, e9, e11, phi, theta)
% plotting one instance of the robot over the graph

P1 = [0;0];
P2 = P1;
P3 = [centrex2; 0];
P4 = P3;
cir_ang = linspace(0,2*pi);
r_p = linkl/10;

% for i = linspace(0, 2*Pi)
%     i
%     for j = linspace(theta(1), theta(2))
%         temp = Inverse_Kinematics([point],[P1 P2 P3 P4],rel, e9, e11, endl, i, j);
%         if isreal(temp)
%             [P5 P6 P7 P8 P9 P10 P11] = temp;
%         end
%     end
% end

[P5 P6 P7 P8 P9 P10 P11] = Inverse_Kinematics(point,[P1 P2 P3 P4],linkl,rel, e9, e11, phi, theta);
patch([P1(1,1); P5(1,1)], [P1(2,1); P5(2,1)],'black');
patch([P5(1,1); P9(1,1)], [P5(2,1); P9(2,1)],'black');
patch([P2(1,1); P6(1,1)], [P2(2,1); P6(2,1)],'black');
patch([P6(1,1); P9(1,1)], [P6(2,1); P9(2,1)],'black');
patch([P3(1,1); P7(1,1)], [P3(2,1); P7(2,1)],'black');
patch([P7(1,1); P10(1,1)], [P7(2,1); P10(2,1)],'black');
patch([P4(1,1); P8(1,1)], [P4(2,1); P8(2,1)],'black');
patch([P8(1,1); P10(1,1)], [P8(2,1); P10(2,1)],'black');
patch([P9(1,1); P11(1,1)], [P9(2,1); P11(2,1)], 'black');
patch([P11(1,1); P10(1,1)], [P11(2,1); P10(2,1)], 'black');
patch(point(1)+r_p*cos(cir_ang),point(2)+r_p*sin(cir_ang),'r'); % this plots the reference point of the end effector

end
