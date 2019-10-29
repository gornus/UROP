function [P5 P6 P7 P8 P10 P9 P11] = Inverse_Kinematics(endpos,anchorpos,linkl,rel, e9, e11, phi, theta) %radians input
%Runs inverse kinematics analysis to obtain joint positions for a given point
%   Detailed explanation goes here
    
    P1 = anchorpos(:,1);
    P2 = anchorpos(:,2);
    P3 = anchorpos(:,3);
    P4 = anchorpos(:,4);

    % Link lengths
    % first half
    d15 = linkl;
    d59 = linkl;
    d26 = linkl;
    d69 = linkl;
    % second half (that links to the redundant link)
    d37 = linkl;
    d710 = linkl;
    d48 = linkl;
    d810 = linkl;
    
    % Moving frame platform vector and link SB vector
    u = [e9;0]; % the end effector 9-e
    v = [e11;0] % e11
    w = [rel;0]; % "redundant link" 10-11


    P9 = [endpos(1)-(e9)*cos(phi);endpos(2)-(e9)*sin(phi)];
    P11 = [endpos(1)+(e11)*cos(phi);endpos(2)+(e11)*sin(phi)];
    % "redundant link" is at some angle allowed by gammma
    P10 = P11 + [norm(w)*cos(pi+phi+theta);norm(w)*sin(pi+phi+theta)];

    % bilateration method
   d19 = sqrt((P9(1,1)-P1(1,1))^2+(P9(2,1)-P1(2,1))^2);
   d29 = sqrt((P9(1,1)-P2(1,1))^2+(P9(2,1)-P2(2,1))^2);
   d310 = sqrt((P10(1,1)-P3(1,1))^2+(P10(2,1)-P3(2,1))^2);
   d410 = sqrt((P10(1,1)-P4(1,1))^2+(P10(2,1)-P4(2,1))^2);
   
   % bilateration method
   A_159 = sqrt((d19^2+d15^2+d59^2)^2-2*(d19^4+d15^4+d59^4))/4;
   A_269 = -sqrt((d29^2+d26^2+d69^2)^2-2*(d29^4+d26^4+d69^4))/4;
   A_3710 = sqrt((d310^2+d37^2+d710^2)^2-2*(d310^4+d37^4+d710^4))/4;
   A_4810 = -sqrt((d410^2+d48^2+d810^2)^2-2*(d410^4+d48^4+d810^4))/4;
   Z_159 = (1/(2*d19^2))*[(d19^2+d15^2-d59^2), -4*A_159; 4*A_159, (d19^2+d15^2-d59^2)];
   Z_269 = (1/(2*d29^2))*[(d29^2+d26^2-d69^2), -4*A_269; 4*A_269, (d29^2+d26^2-d69^2)];
   Z_3710 = (1/(2*d310^2))*[(d310^2+d37^2-d710^2), -4*A_3710; 4*A_3710, (d310^2+d37^2-d710^2)];
   Z_4810 = (1/(2*d410^2))*[(d410^2+d48^2-d810^2), -4*A_4810; 4*A_4810, (d410^2+d48^2-d810^2)];
   % vectors
   p19 = P9-P1;
   p29 = P9-P2;
   p310 = P10-P3;
   p410 = P10-P4;
   p15 = Z_159 * p19;
   p26 = Z_269 * p29;
   p37 = Z_3710 * p310;
   p48 = Z_4810 * p410;
   P5 = P1 + p15;
   P6 = P2 + p26;
   P7 = P3 + p37;
   P8 = P4 + p48;
end

