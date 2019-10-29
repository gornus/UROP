tic

%  Calculate dexterous worksapce of Gosslein's robot
close all
clear all
l1 = 2;  % Length of links fpr each leg
l2 = 2;
gamma = 30;  % singularity avoidance angle
d10e = 1;  % Distance between P10 and end-effector
d11e = 1;  % distance from joint P11 to EE
d911 = 1;  % distance from joint P9 to P11
C1 = [0;0];  % Position of base joints
C2 = [2;0];

% Example 2
% l1 = 2;  % Length of links for each leg
% l2 = 2;
% gamma = 20;  % singularity avoidance angle
% d10e = 0.5;  % Distance between P10 and end-effector
% d11e = 0.5;  % distance from joint P11 to EE
% d911 = 0.5;  % distance from joint P9 to P11
% C1 = [0;0];  % Position of base joints
% C2 = [2;0];


gamma = deg2rad(gamma);

%  Define dimensions of robot
leg1_max = 2*l1*cos(gamma/2);  % maximum extension of legs
leg1_min = 2*l1*cos((pi-gamma)/2);  % minimum extension of legs
leg2_max = 2*l2*cos(gamma/2);  % maximum extension of legs
leg2_min = 2*l2*cos((pi-gamma)/2);  % minimum extension of legs
d9emax = sqrt(d911^2+d11e^2-2*d11e*d911*cos(pi-gamma));  % Max distance between P9 and end-effector
d9emin = sqrt(d911^2+d11e^2-2*d11e*d911*cos(gamma));  % Min distance between P9 and end-effector

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Outer boundary of worksapce of left side of the robot

if d10e > leg1_max % if d10e > max leg radius, there is no full rotation workspace

    WS1_OR = 0;    

else    
    WS1_OR = leg1_max - d10e;  % else the full rotation workspace is, generally, given by max leg radius minus d10e
    if (leg1_max-2*d10e<leg1_min) && (leg1_max-2*d10e>-leg1_min)  % If going down by 2 times this distance moves you into the min leg radius circle
        % If it moves you into the min leg radius circle, a further step is required:
        if d10e >= leg1_min  % If d10e > the min leg radius, move the end-effector down by the amount such that that the EE position minus d10e is at the origin minus the min leg radius.
            WS1_OR = -leg1_min + d10e;
        else  % If d10e < the min leg radius, then there is no full rotation workspace.
            WS1_OR = 0;
        end
    elseif  leg1_max-2*d10e<-leg1_min  % If it moves you past the min leg circle completely, it's okay. 
        WS1_OR = leg1_max - d10e;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inner boundary of worksapce of left side of the robot

WS1_ORb = 0;  % For this case, there is the possiblity of an inner annulus. Intially set it to zero
WS1_IRb = 0;

if d10e > leg1_max  % if d10e > max leg radius, there is no full rotation workspace

    WS1_IR = 0;    

else
    if d10e < leg1_min  % If d10e < min leg radius, then it’s given by min leg radius plus d10e.
        WS1_IR = leg1_min + d10e;  % else the full rotation workspace is, generally, given by min leg radius plus d10e
        if (2*d10e > leg1_max-leg1_min)  % However, if 2 times d10e > the difference between the max and min leg radii, there is no workspace.
            WS1_OR = 0;
            WS1_IR = 0;
        end
    else  % if d10e > min leg radius
        if (2*d10e < leg1_min + leg1_max) % If 2 times d10e < min leg radius plus max leg radius
            WS1_ORb = -leg1_min + d10e;  % the outer boundary equals the origin minus the min leg radius plus d10e. 
            WS1_IR = leg1_min + d10e;  % The inner region of the outer annulus is min leg radius plus d10e.
            if (2*d10e > leg1_max-leg1_min) % (if 2 times d10e > the difference between the max and min leg radii, there is no workspace).
                WS1_IR = 0;
                WS1_OR = 0;
            end
        else  
            WS1_OR = 0;
            WS1_IR = 0;
            WS1_IRb = 0;
            WS1_ORb = leg1_max - d10e;
        end
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Outer boundary of worksapce of right side of the robot
if d9emin > leg2_max % if d10e > max leg radius, there is no full rotation workspace

    WS2_OR = 0;    

else    
    WS2_OR = leg2_max - d9emin; % Start out with max leg radius minus min d9e.
    if (leg2_max-2*d9emin >= leg2_min)  % If max leg radius minus 2 times min d9e > min leg radius, it's okay.
        WS2_OR = leg2_max - d9emin;

    else  % If not, an extra step is required:

        if (leg2_max-2*d9emin > -leg2_min)  && (leg2_max-d9emin-d9emax <= -leg2_min)  % If max leg radius minus 2 times min d9e is higher than origin minus min leg radius, but max leg radius minus min d9e minus max d9e is lower than it, then it’s okay.

            WS2_OR = leg2_max - d9emin;
        elseif  (leg2_max-2*d9emin > -leg2_min)  && (leg2_max-d9emin-d9emax > -leg2_min) % If max leg radius minus min d9e minus max d9e is also higher than it, then we need another step:
            if (leg2_min > d9emax)  % If the min leg radius > max d9e, there is no workspace.
                WS2_OR = 0;
            else  % Otherwise, move move down until EE position minus max d9e is the same as origin minus min leg radius.
                WS2_OR = -leg2_min + d9emax; 
                if (d9emax+d9emin > leg2_min + leg2_max)  % If moving up moves you max leg radius, move down from max leg radius instead by d9emin 
                    WS2_OR = leg2_max - d9emin;
                end
            end            
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Inner boundary of worksapce of right side of the robot

WS2_ORb = 0;  % For this case, there is the possiblity of an inner annulus. Intially set it to zero
WS2_IRb = 0;

if d9emin > leg2_max  % If min d9e > max leg radius, there is no full rotation workspace.

    WS2_OR = 0;
    WS2_IR = 0;    

else
    if (d9emax < leg2_min)  % If max d9e < min leg radius, then it's given by min leg radius plus min d9e
        WS2_IR = leg2_min + d9emin;  % If max d9e < min leg radius, then it's given by min leg radius plus min d9e
        if (2*d9emin > leg2_max-leg2_min)  % However, if 2 times min d9e > the difference between the max and min leg radii, there is no workspace. 
            WS2_OR = 0;
            WS2_IR = 0;
        end
    else % If max d9e > min leg radius, then there is an extra annular area in the workspace:
        WS2_IRb = 0;  % The min workspace  boundary shrinks to zero. 
        WS2_ORb = -leg2_min + d9emax;  % The outer boundary of the inner ring is given by the origin minus the min leg radius plus the max d9e.
        if (WS2_ORb > leg2_max - d9emin)   % However, if this distance is within min d9e of the max leg radius, this boundary is given by max leg radius minus min d9e.
        WS2_ORb = leg2_max - d9emin;
        end
        if (2*d9emin < leg2_max-leg2_min)
            WS2_IR = leg2_min + d9emin;  % The second min boundary is given by the min leg radius plus min d9e
        else
            WS2_IR = 0;
        end
        if (WS2_IR < WS2_ORb)  % (If the inner boundary of the outer ring is less than the outer boundary of the inner ring, the 2 rings merge into one). 
            WS2_ORb = 0;
            WS2_IR = WS2_IRb;
        end
    end
end

WS1_OR
WS1_IR
WS1_ORb
WS2_OR
WS2_IR
WS2_ORb


th = linspace(0,2*pi);
figure(1)
%title(sprintf('l1:%d,l2:%d,gamma:%d,d10e:%d,d11e:%d,d911:%d\n',l1,l2,gamma,d10e,d11e,d911));
cla
plot(C1(1)+WS1_IR*cos(th),C1(2)+WS1_IR*sin(th),'b')
title(['l1:' num2str(l1) ' l2:' num2str(l2) ' gamma:' num2str(gamma) ' d10e:' num2str(d10e) ' d11e:' num2str(d11e) ' d911:' num2str(d911)]);
axis([min(C1(1)-5,C2(1)-5) max(C1(1)+5,C2(1)+5) min(C1(2)-5,C2(2)-5) max(C1(2)+5,C2(2)+5)])
hold on
plot(C1(1)+WS1_OR*cos(th),C1(2)+WS1_OR*sin(th),'b')
hold on
plot(C1(1)+WS1_ORb*cos(th),C1(2)+WS1_ORb*sin(th),'b')
hold on
plot(C1(1)+WS1_IRb*cos(th),C1(2)+WS1_IRb*sin(th),'b')
hold on
plot(C2(1)+WS2_OR*cos(th),C2(2)+WS2_OR*sin(th),'r')
hold on
plot(C2(1)+WS2_IR*cos(th),C2(2)+WS2_IR*sin(th),'r')
hold on
plot(C2(1)+WS2_ORb*cos(th),C2(2)+WS2_ORb*sin(th),'r')
hold on
plot(C2(1)+WS2_IRb*cos(th),C2(2)+WS2_IRb*sin(th),'r')
hold on

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical analysis of workspace of left side of robot

res1 = 0.001 + (leg1_max + d10e)/50;

EEArray1 = [];
for x=C1(1)-5:res1:C1(1)+5
    for y=C1(2)-5:res1:C1(2)+5
        for phi=0:pi/18:2*pi
            EE = [x;y];
            P10 = EE + d10e*[cos(phi);sin(phi)];
            d110 = sqrt((P10(1)-C1(1))^2+(P10(2)-C1(2))^2);
            if (d110>leg1_max) || (d110<leg1_min)
                break
            end
            if (phi==2*pi)
                EEArray1 = [EEArray1;x,y];
            end
        end
    end
    %fprintf('x: %d\n',x);
end

% for i=1:1:size(EEArray1,1)
%     plot(EEArray1(i,1)+0.01*cos(th),EEArray1(i,2)+0.01*sin(th),'k')
%     hold on
% end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Numerical analysis of workspace of right side of robot

[m,n] = size(EEArray1);

flag = 0;
flag2 = 0;
phiBreak = 0;
% res2 = 0.001 + (leg2_max + d9emax)/100;
jmax = 50;
phi_final = -2*pi;

EEArray2 = [];
for x = 1:m
% for x=C2(1)-5:res2:C2(1)+5
%     for y=C2(2)-5:res2:C2(2)+5
        flag = 0;
        j = 0;  % Set initial theta as gamma by setting j=0
        for phi = 2*pi:-pi/720:phi_final
            
%             EE = [x;y];
            EE = [EEArray1(x,1); EEArray1(x,2)];
            P11 = EE + d11e*[cos(phi);sin(phi)];
            theta = gamma + j*(pi-2*gamma)/jmax;
            P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
            d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
            
            if phi==2*pi  % If we're in the first iteration, pick the smallest value of theta that's possible
                while (d29>leg2_max) || (d29<leg2_min)  % While not in workspace, check all thetas
                    if j==jmax
                        phiBreak = 1;
                        break
                    end
                    j = j + 1;
                    theta = gamma + j*(pi-2*gamma)/jmax;
                    P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                    d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                end
            end
            if phiBreak == 1
                phiBreak = 0;
                break
            end
            
            
%             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
%                 fprintf('2x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
%             end
            
            while j>=0  % Make sure each theta is possible and is as small as possible
                theta = gamma + j*(pi-2*gamma)/jmax;
                P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                
%                 if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
%                     fprintf('3x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
%                 end
                
                if (d29<=leg2_max) && (d29>=leg2_min)  % If theta is possible here, make sure it's at minimum and then move on to next phi
                    while j > 0  % Check that theta is minimum
                        j = j - 1;
                        theta = gamma + j*(pi-2*gamma)/jmax;
                        P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        if (d29>leg2_max) || (d29<leg2_min)  % If this new theta is not okay, go back to previous theta
                            j = j + 1;
%                             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
%                                 fprintf('4x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
%                             end 
                            break
                        end
                    end
%                     if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
%                         fprintf('5x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
%                     end 
                    break
                else  % If theta is not possible here, check if a neighbouring theta is possible
                    while ((d29>leg2_max) || (d29<leg2_min)) && (phi<2*pi)
                        j = j + 1;  % If theta is not possible here, try next theta
                        theta = gamma + j*(pi-2*gamma)/jmax;
                        P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        if j>=jmax
                            flag = 2;  % Else this x,y does not have full phi rotation
%                             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
%                                 fprintf('6x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
%                             end 
                            break
                        end
%                         fprintf('x:%.2f, y:%.2f, phi:%.2f, j:%.2f\n',x,y,phi,j);
                    end
                    if flag~=2
                        flag2 = 1;  % If a theta is possible here, move on to next phi
                    end
                end
                if flag2==1  % Exit loop because this theta works therefore this phi works
                    flag2=0;  % reset flag2 to zero before exiting the j loop and entering the next phi iteration
                    break   
                end
                if (j==jmax) || (flag==2)  % We have found a theta that has no neighnouring possible thetas, therefore this position is not possible and we break without storing the point
                    flag=1;  % flag=1 signals we must break from phi loop once we have broken from the j loop 
                    break
                end
            end
            
%             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
%                 fprintf('7x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
%             end 
            
            if (flag==1) 
                break
            end
            if (x>6.8&&x<6.85) && (y>1.65&&y<1.75)
                fprintf('1x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
            end
            if (phi==phi_final)
                EEArray2 = [EEArray2;EE'];
            end
        end
%     end
end


plot(EEArray2(:,1),EEArray2(:,2),'o')
% for i=1:1:size(EEArray2,1)
%     plot(EEArray2(i,1)+0.01*cos(th),EEArray2(i,2)+0.01*sin(th),'b')
%     hold on
% end


[m,n] = size(EEArray2);
P1 = C1;
P2 = C1;
P3 = C2;
P4 = C2;
anchorpos = [P1 P2 P3 P4];
point = [EEArray2(floor(m/2),1); EEArray2(floor(m/2),2)];
[P5 P6 P7 P8 P9 P10 P11] = Inverse_Kinematics(point,anchorpos,l1,d911, d10e, d11e, pi/3, pi/2);
patch([P1(1,1); P5(1,1)], [P1(2,1); P5(2,1)],'black');
patch([P5(1,1); P10(1,1)], [P5(2,1); P10(2,1)],'black');
patch([P2(1,1); P6(1,1)], [P2(2,1); P6(2,1)],'black');
patch([P6(1,1); P10(1,1)], [P6(2,1); P10(2,1)],'black');
patch([P3(1,1); P7(1,1)], [P3(2,1); P7(2,1)],'black');
patch([P7(1,1); P9(1,1)], [P7(2,1); P9(2,1)],'black');
patch([P4(1,1); P8(1,1)], [P4(2,1); P8(2,1)],'black');
patch([P8(1,1); P9(1,1)], [P8(2,1); P9(2,1)],'black');
patch([P9(1,1); P11(1,1)], [P9(2,1); P11(2,1)], 'black');
patch([P11(1,1); P10(1,1)], [P11(2,1); P10(2,1)], 'black');
% patch([P11(1,1); point(1,1)], [P11(2,1); point(2,1)], 'blue');
% patch([point(1,1); P10(1,1)], [point(2,1); P10(2,1)], 'red');
patch(point(1)+0.05*cos(linspace(0,2*pi)),point(2)+0.05*sin(linspace(0,2*pi)),'k'); % this plots the reference point of the end effector
patch(C1(1)+0.02*cos(linspace(0,2*pi)),C1(2)+0.02*sin(linspace(0,2*pi)),'r'); % this plots the reference point of the end effector
patch(C2(1)+0.02*cos(linspace(0,2*pi)),C2(2)+0.02*sin(linspace(0,2*pi)),'r'); % this plots the reference point of the end effector


hold off


toc