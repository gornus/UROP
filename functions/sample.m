function [fitness] = sample(input)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
% x = input(1);
% y = input(2);
% z = input(3);
% 
% temp = x*y*z;
% if temp < 0;
%     temp = 0;
% else
%     temp = temp + x;
% end
% 
% fitness = 1/temp;
% 


%   function to calculate the maximal workspace given the parameters
    syms x y
    Pi = sym(pi);
    
    
    anchorx = input(1);
    linkl = input(2);
    endl = 50;
    rel = 30;
%     gammma = input(5);
    [anchorx linkl endl rel];
    % Anchors
    P1 = [0;0];
    P2 = [anchorx;0];
    P3 = [0;0];
    P4 = [anchorx;0];

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
    v = [endl;0]; % the end effector 9-11
    w = [rel;0]; % "redundant link" 10-11


    gammma = 10;
    gammma = deg2rad(gammma);
    theta = [gammma, Pi-gammma];

    %% defining the intersecting shapes 
    syms x y

    % parametres of the first outer circle
    % transforming the circular workspace in reference with 0-2pi orientation of the end effector
    O1 = P1;
    R1 = d15 + d59 + norm(v)/2; 
    Cir1 = R1^2 == (x-O1(1))^2 + (y-O1(2))^2;
    Shape1 = [Cir1;0;2*Pi;O1(1);O1(2);R1;1]; % this shape construction is obselete

    % parametres of the second outer circle
    % transforming the circular workspace in reference with 0-2pi orientation of the end effector
    O2 = P2;
    R2 = d26 + d69 + norm(v)/2;
    Cir2 = R2^2 == (x-O2(1))^2 + (y-O2(2))^2;
    Shape2 = [Cir2;0;2*Pi;O2(1);O2(2);R2;2];

    % parametres of the third shape
    O3 = P3;
    R3 = d37 + d710 + norm(w) + norm(v)/2;
    Cir3 = R3^2 == (x-O3(1))^2 + (y-O3(2))^2;
    Shape3 = [Cir3;0;2*Pi;O3(1);O3(2);R3;3];

    % parametres of the fourth shape
    O4 = P4;
    R4 = d48 + d810 + norm(w) + norm(v)/2;
    Cir4 = R4^2 == (x-O4(1))^2 + (y-O4(2))^2;
    Shape4 = [Cir4;0;2*Pi;O4(1);O4(2);R4;4];

    % plot all the anchors and translated centres
    % patch(P1(1)+r*cos(cir_ang),P1(2)+r*sin(cir_ang),'r');
    % patch(P2(1)+r*cos(cir_ang),P2(2)+r*sin(cir_ang),'y');
    % patch(P3(1)+r*cos(cir_ang),P3(2)+r*sin(cir_ang),'b');
    % patch(P4(1)+r*cos(cir_ang),P4(2)+r*sin(cir_ang),'g');
    % patch(O1(1)+r*cos(cir_ang),O1(2)+r*sin(cir_ang),'r');
    % patch(O2(1)+r*cos(cir_ang),O2(2)+r*sin(cir_ang),'r');
    % patch(O3(1)+r*cos(cir_ang),O3(2)+r*sin(cir_ang),'r');
    % patch(O4(1)+r*cos(cir_ang),O4(2)+r*sin(cir_ang),'r');

    % plot for all four shapes for debugging
%     arcplot([Shape1(2) Shape1(3)], [Shape1(4) Shape1(5)], Shape1(6),'r--');
%     arcplot([Shape2(2) Shape2(3)], [Shape2(4) Shape2(5)], Shape2(6),'r--');
%     arcplot([Shape3(2) Shape3(3)], [Shape3(4) Shape3(5)], Shape3(6),'b--');
%     arcplot([Shape4(2) Shape4(3)], [Shape4(4) Shape4(5)], Shape4(6),'b--');

    intersections = [];

    %% calculating the relevant vertices
    % finding all the intersection points between two circles

    intersections = intersection(Shape1,Shape2,[x y],intersections);
    intersections = intersection(Shape1,Shape3,[x y],intersections);
    intersections = intersection(Shape1,Shape4,[x y],intersections);
    intersections = intersection(Shape2,Shape3,[x y],intersections);
    intersections = intersection(Shape2,Shape4,[x y],intersections);
    intersections = intersection(Shape3,Shape4,[x y],intersections);
    
    % finding the intersection points that are contained in all four circles
    corner = [];
    [m,n] = size(intersections);

%     hold on
%     pbaspect([1 1 1])
%     axis([-400 600 -500 500 ]);

    for i = 1:n
    %     patch(double(intersections(1,i))+r*cos(cir_ang),double(intersections(2,i))+r*sin(cir_ang),'k');
        if incircle([intersections(1,i) intersections(2,i)],Shape1)==1 % determine whether the points are in the first circle
%             if incircle([intersections(1,i) intersections(2,i)],Shape2)==1 % determine whether the points are in the second circle
%                 if incircle([intersections(1,i) intersections(2,i)],Shape3)==1
%                     if incircle([intersections(1,i) intersections(2,i)],Shape4)==1
                        corner = [corner, intersections(:,i)];
%                     end
%                 end
%             end
        end
    end
    [m2,n2] = size(corner);
    fitness = n2;
end
