%  Calculate dexterous worksapce of Gosslein's robot and prove it with numerical analysis
close all
clear all
% l1 = 2;  % Length of links fpr each leg
% l2 = 2;
% gamma = 0.05;  % singularity avoidance angle
% d10e = 1;  % Distance between P10 and end-effector
% d911 = 1;  % distance from joint P9 to P11
% d11e = 1;  % distance from joint P11 to EE
C1 = [0;0];  % Position of base joints
C2 = [8;0];

count = 0;
for l1 = 1:0.5:2
    for l2 = 1:0.5:2
        for gamma = 0:pi/6:pi/2
            for d10e = 0.5:0.5:1
                for d11e = 0.5:0.5:1
                    for d911 = 0.5:0.5:1
                        count = count+1;

                        %  Define dimensions of robot
                        leg1_max = 2*l1*cos(gamma/2);  % maximum extension of legs
                        leg1_min = 2*l1*cos((pi-gamma)/2);  % minimum extension of legs
                        leg2_max = 2*l2*cos(gamma/2);  % maximum extension of legs
                        leg2_min = 2*l2*cos((pi-gamma)/2);  % minimum extension of legs
                        d9emax = sqrt(d911^2+d11e^2-2*d11e*d911*cos(pi-gamma));  % Max distance between P9 and end-effector
                        d9emin = sqrt(d911^2+d11e^2-2*d11e*d911*cos(gamma));  % Min distance between P9 and end-effector
                        
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

                        % right side of the robot

                        WS2_OR = 0;
                        WS2_IR = 0;
                        WS2_ORb = 0;
                        if d9emin <= leg2_max % if d10e > max leg radius, there is no full rotation workspace  
                          if 2*d9emin <= leg2_max - leg2_min
                              WS2_OR = leg2_max - d9emin;
                              WS2_IR = leg2_min + d9emin;
                          else if d9emin+d9emax < leg2_max + leg2_min
                                  if d9emax >= leg2_min
                                      WS2_OR = d9emax - leg2_min;
                                  end
                              else
                                  WS2_OR = leg2_max - d9emin;
                              end
                          end
                          if d9emax < leg2_min
                              if 2*d9emin <= leg2_max - leg2_min
                                  WS2_IR = leg2_min + d9emin;
                              else
                                  WS2_OR = 0;
                                  WS2_IR = 0;
                              end
                          else
                              if d9emax + d9emin > leg2_max + leg2_min
                                  WS2_ORb = leg2_max - d9emin;
                              else
                                  WS2_ORb = d9emax - leg2_min;
                              end
                          end
                        end



                        if WS1_ORb >= WS1_IR
                            WS1_IR = 0;
                            if WS1_OR~=0
                                WS1_ORb = 0;
                            end
                        end
                        if WS2_ORb >= WS2_IR
                            WS2_IR = 0;
                            if WS2_OR~=0
                                WS2_ORb = 0;
                            end
                        end

                        th = linspace(0,2*pi);
                        figure(count)
                        title(sprintf('l1:%d,l2:%d,gamma:%d,d10e:%d,d11e:%d,d911:%d\n',l1,l2,gamma,d10e,d11e,d911));
                        cla
                        plot(C1(1)+WS1_IR*cos(th),C1(2)+WS1_IR*sin(th),'k')
                        title(['l1:' num2str(l1) ' l2:' num2str(l2) ' gamma:' num2str(gamma) ' d10e:' num2str(d10e) ' d11e:' num2str(d11e) ' d911:' num2str(d911)]);
                        axismax = max(max(C1(1)+5,C2(1)+WS2_OR+1),max(C1(2)+5,C2(2)+WS2_OR+1));
                        axismin = min(min(C1(1)-5,C2(1)-WS2_OR+1),min(C1(2)-5,C2(2)-WS2_OR+1));
                        axismax = max(C2(2)+WS2_OR+1,C2(1)+WS2_OR+1);
                        axismin = min(C2(2)-WS2_OR-1,C2(1)-WS2_OR-1);
                        
                        axis([axismin axismax axismin axismax])
                        pbaspect([1 1 1])
                        hold on
                        plot(C1(1)+WS1_OR*cos(th),C1(2)+WS1_OR*sin(th),'k')
                        hold on
                        plot(C1(1)+WS1_ORb*cos(th),C1(2)+WS1_ORb*sin(th),'k')
                        hold on
                        plot(C2(1)+WS2_OR*cos(th),C2(2)+WS2_OR*sin(th),'r')
                        hold on
                        plot(C2(1)+WS2_IR*cos(th),C2(2)+WS2_IR*sin(th),'r')
                        hold on
                        plot(C2(1)+WS2_ORb*cos(th),C2(2)+WS2_ORb*sin(th),'r')
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

                        for i=1:1:size(EEArray1,1)
                            plot(EEArray1(i,1)+0.01*cos(th),EEArray1(i,2)+0.01*sin(th),'g')
                            hold on
                        end



                        % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        % 
                        % % Numerical analysis of workspace of right side of robot
                        % 
                        % % [m,n] = size(EEArray1);
                        % 
                        % flag = 0;
                        % flag2 = 0;
                        % phiBreak = 0;
                        % res2 = 0.001 + (leg2_max + d9emax)/50;
                        % jmax = 20;
                        % phi_final = -2*pi;
                        % 
                        % EEArray2 = [];
                        % % for x = 1:m
                        % for x=C2(1)-WS2_OR:res2:C2(1)+WS2_OR
                        %     for y=C2(2)-WS2_OR:res2:C2(2)+WS2_OR
                        %         flag = 0;
                        %         j = 0;  % Set initial theta as gamma by setting j=0
                        %         for phi = 2*pi:-pi/60:phi_final
                        %             
                        %             EE = [x;y];
                        % %             EE = [EEArray1(x,1); EEArray1(x,2)];
                        %             P11 = EE + d11e*[cos(phi);sin(phi)];
                        %             theta = gamma + j*(pi-2*gamma)/jmax;
                        %             P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        %             d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        %             
                        %             if phi==2*pi  % If we're in the first iteration, pick the smallest value of theta that's possible
                        %                 while (d29>leg2_max) || (d29<leg2_min)  % While not in workspace, check all thetas
                        %                     if j==jmax
                        %                         phiBreak = 1;
                        %                         break
                        %                     end
                        %                     j = j + 1;
                        %                     theta = gamma + j*(pi-2*gamma)/jmax;
                        %                     P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        %                     d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        %                 end
                        %             end
                        %             if phiBreak == 1
                        %                 phiBreak = 0;
                        %                 break
                        %             end
                        %             
                        %             
                        % %             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
                        % %                 fprintf('2x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %             end
                        %             
                        %             while j>=0  % Make sure each theta is possible and is as small as possible
                        %                 theta = gamma + j*(pi-2*gamma)/jmax;
                        %                 P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        %                 d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        %                 
                        % %                 if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
                        % %                     fprintf('3x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %                 end
                        %                 
                        %                 if (d29<=leg2_max) && (d29>=leg2_min)  % If theta is possible here, make sure it's at minimum and then move on to next phi
                        %                     while j > 0  % Check that theta is minimum
                        %                         j = j - 1;
                        %                         theta = gamma + j*(pi-2*gamma)/jmax;
                        %                         P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        %                         d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        %                         if (d29>leg2_max) || (d29<leg2_min)  % If this new theta is not okay, go back to previous theta
                        %                             j = j + 1;
                        % %                             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
                        % %                                 fprintf('4x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %                             end 
                        %                             break
                        %                         end
                        %                     end
                        % %                     if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
                        % %                         fprintf('5x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %                     end 
                        %                     break
                        %                 else  % If theta is not possible here, check if a neighbouring theta is possible
                        %                     while ((d29>leg2_max) || (d29<leg2_min)) && (phi<2*pi)
                        %                         j = j + 1;  % If theta is not possible here, try next theta
                        %                         theta = gamma + j*(pi-2*gamma)/jmax;
                        %                         P9 = P11 + (d911/d11e)*[cos(theta),-sin(theta);sin(theta),cos(theta)]*(EE-P11);  % P9 by anti-clockwise rotation
                        %                         d29 = sqrt((P9(1)-C2(1))^2+(P9(2)-C2(2))^2);
                        %                         if j>=jmax
                        %                             flag = 2;  % Else this x,y does not have full phi rotation
                        % %                             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
                        % %                                 fprintf('6x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %                             end 
                        %                             break
                        %                         end
                        % %                         fprintf('x:%.2f, y:%.2f, phi:%.2f, j:%.2f\n',x,y,phi,j);
                        %                     end
                        %                     if flag~=2
                        %                         flag2 = 1;  % If a theta is possible here, move on to next phi
                        %                     end
                        %                 end
                        %                 if flag2==1  % Exit loop because this theta works therefore this phi works
                        %                     flag2=0;  % reset flag2 to zero before exiting the j loop and entering the next phi iteration
                        %                     break   
                        %                 end
                        %                 if (j==jmax) || (flag==2)  % We have found a theta that has no neighnouring possible thetas, therefore this position is not possible and we break without storing the point
                        %                     flag=1;  % flag=1 signals we must break from phi loop once we have broken from the j loop 
                        %                     break
                        %                 end
                        %             end
                        %             
                        % %             if (x>3.13&&x<3.15) && (y>-2.14&&y<-2.12)
                        % %                 fprintf('7x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %             end 
                        %             
                        %             if (flag==1) 
                        %                 break
                        %             end
                        % %             if (x>6.8&&x<6.85) && (y>1.65&&y<1.75)
                        % %                 fprintf('1x:%.2f, y:%.2f, phi:%.2f, j:%.2f, d29:%.2f\n',x,y,phi,j,d29);
                        % %             end
                        %             if (phi==phi_final)
                        %                 EEArray2 = [EEArray2;EE'];
                        %             end
                        %         end
                        %     end
                        % end
                        % 
                        % % [m,n] = size(EEArray2);
                        % % if m >0
                        % %     plot(EEArray2(:,1),EEArray2(:,2),'o')
                        % % end
                        % for i=1:1:size(EEArray2,1)
                        %     plot(EEArray2(i,1)+0.01*cos(th),EEArray2(i,2)+0.01*sin(th),'b')
                        %     hold on
                        % end

                        
                        hold off
                        drawnow
                        fprintf('l1:%d,l2:%d,gamma:%d,d10e:%d,d11e:%d,d911:%d\n',l1,l2,gamma,d10e,d11e,d911);
                    end
                end
            end
        end
    end
end


