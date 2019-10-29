function [fitness] = full_rotation_dexterous_workspace(input)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
%   function to calculate the maximal workspace given the parameters
    syms x y
    Pi = sym(pi);

    linkl = 2;
    gammma = 10;  % singularity avoidance angle
    d10e = 0.3;  % Distance between P10 and end-effector
    d11e = 0.3;  % distance from joint P11 to EE
    d911 = 0.5;  % distance from joint P9 to P11


    C1 = [0;0];  % Position of base joints
    C2 = [input(1);0];
    
    gammma = deg2rad(gammma);
    %  Define dimensions of robot
    l1 = linkl;  % Length of links fpr each leg
    l2 = linkl;
    leg1_max = 2*l1*cos(gammma/2);  % maximum extension of legs
    leg1_min = 2*l1*cos((Pi-gammma)/2);  % minimum extension of legs
    leg2_max = 2*l2*cos(gammma/2);  % maximum extension of legs
    leg2_min = 2*l2*cos((Pi-gammma)/2);  % minimum extension of legs
    d9emax = sqrt(d911^2+d11e^2-2*d11e*d911*cos(Pi-gammma));  % Max distance between P9 and end-effector
    d9emin = sqrt(d911^2+d11e^2-2*d11e*d911*cos(gammma));  % Min distance between P9 and end-effector
    
    %% formulation of the workspace boundaries
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

    if abs(WS1_IR - WS1_ORb) < 0.0000001
        WS1_IR = 0;
        WS1_ORb = 0;
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Boundaries of worksapce of right side of the robot
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
    if abs(WS2_IR - WS2_ORb) < 0.0000001
        WS2_IR = 0;
        WS2_ORb = 0;
    end


    %% defining the left side of the robot
    % parametres of the first outer circle
    P1 = C1;
    RL1 = WS1_OR;
    CirL1 = RL1^2 == (x-P1(1))^2 + (y-P1(2))^2;
    BL1 = [CirL1;0;2*Pi;P1(1);P1(2);RL1;1];

    % parametres of the first inner circle
    RL2 = WS1_IR;
    CirL2 = RL2^2 == (x-P1(1))^2 + (y-P1(2))^2;
    BL2 = [CirL2;0;2*Pi;P1(1);P1(2);RL2;2];

    % parametres of the first outer circle
    RL3 = WS1_ORb;
    CirL3 = RL3^2 == (x-P1(1))^2 + (y-P1(2))^2;
    BL3 = [CirL3;0;2*Pi;P1(1);P1(2);RL3;1];

    %% defining the right side of the robot
    % parametres of the first outer circle
    P2 = C2;
    RR1 = WS2_OR;
    CirR1 = RR1^2 == (x-P2(1))^2 + (y-P2(2))^2;
    BR1 = [CirR1;0;2*Pi;P2(1);P2(2);RR1;1];

    % parametres of the first inner circle
    RR2 = WS2_IR;
    CirR2 = RR2^2 == (x-P2(1))^2 + (y-P2(2))^2;
    BR2 = [CirR2;0;2*Pi;P2(1);P2(2);RR2;2];

    % parametres of the first outer circle
    RR3 = WS2_ORb;
    CirR3 = RR3^2 == (x-P2(1))^2 + (y-P2(2))^2;
    BR3 = [CirR3;0;2*Pi;P2(1);P2(2);RR3;1];

    %% plotting of the robot and workspace calculation
%     figure()
%     xmax = (max([C1(1),C2(1)]))+max([WS1_OR,WS1_IR,WS1_ORb,WS1_OR,WS1_IR,WS1_ORb]);
%     xmin = (min([C1(1),C2(1)]))-max([WS1_OR,WS1_IR,WS1_ORb,WS1_OR,WS1_IR,WS1_ORb]);
%     ymax = (max([C1(2),C2(2)]))+max([WS1_OR,WS1_IR,WS1_ORb,WS1_OR,WS1_IR,WS1_ORb]);
%     ymin = (min([C1(2),C2(2)]))-max([WS1_OR,WS1_IR,WS1_ORb,WS1_OR,WS1_IR,WS1_ORb]);
%     amax = floor(vpa(max([xmax ymax])))
%     amin = floor(vpa(min([xmin ymin])))
%     axis([min([amax amin])-1 max([amax amin])+1 min([amax amin])-1 max([amax amin])+1]);
%     xticks(amin:1:amax)
%     yticks(amin:1:amax)

    pbaspect([1 1 1])
    axis([-5 5 -5 5])
    grid on
    arcplot([BL1(2) BL1(3)], [BL1(4) BL1(5)], BL1(6),'k-');
    arcplot([BL2(2) BL2(3)], [BL2(4) BL2(5)], BL2(6),'k--');
    arcplot([BL3(2) BL3(3)], [BL3(4) BL3(5)], BL3(6),'k--');
    arcplot([BR1(2) BR1(3)], [BR1(4) BR1(5)], BR1(6),'b-');
    arcplot([BR2(2) BR2(3)], [BR2(4) BR2(5)], BR2(6),'b--');
    arcplot([BR3(2) BR3(3)], [BR3(4) BR3(5)], BR3(6),'b--');
    title(sprintf("i = %d",i))
    cir_ang = linspace(0,2*pi);
    patch(C1(1)+0.1*cos(cir_ang),C1(2)+0.1*sin(cir_ang),'r');
    patch(C2(1)+0.1*cos(cir_ang),C2(2)+0.1*sin(cir_ang),'r');

    area_list = [];
    
    %% intersecting the outer circles
%     fprintf("step 1\n")
    [ix1, iy] = solve([BL1(1), BR1(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL1, BR1);
    large = larger(BL1, BR1);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        area = cirarea(small);
    elseif size(ix1,1)>1
        area = intersection_area(BL1, BR1); %the intersection of outer boundaries
    else
        area = 0;
    end
    
    area;

    %% accounting for the inner boundaries
%     fprintf("step 2\n")
    % intersection of inner boundaries and outer boundaries
    [ix2, iy] = solve([BL2(1), BR1(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL2, BR1);
    large = larger(BL2, BR1);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        void1 = cirarea(small);
    elseif size(ix2,1)>1
        void1 = intersection_area(BL2, BR1); %the intersection of outer boundaries
    else
        void1 = 0;
    end
    
    [ix3, iy] = solve([BL1(1), BR2(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL1, BR2);
    large = larger(BL1, BR2);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        void1 = void1 + cirarea(small);
    elseif size(ix3,1)>1
        void1 = void1 + intersection_area(BL1, BR2); %the intersection of outer boundaries
    end

    % intersection of inner boundaries
    [ix4, iy] = solve([BL2(1), BR2(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL2, BR2);
    large = larger(BL2, BR2);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        void1 = void1 - cirarea(small);
        area = area - void1;
        area_list = [area];
    elseif size(ix4,1)>=1
        void1 = void1 - intersection_area(BL2, BR2); %the intersection of outer boundaries
        if size(ix2,1)>0 && size(ix3,1)>0
            void1 = void1/2;
            area = area/2;
        end
        area = area - void1;
        area_list = [area area];
    end
    
    

    %% accounting for the final inner circle region
%     fprintf("step 3\n")
    
    % considering the first inner circle region
    [ix5, iy] = solve([BL3(1), BR1(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL3, BR1);
    large = larger(BL3, BR1);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001% if the smaller boundary is completely inside the larger boundary
        area2 = cirarea(small);
    elseif size(ix5,1)>1
        area2 = intersection_area(BL3, BR1); %the intersection of outer boundaries
    else
        area2 = 0;
    end


    [ix6, iy] = solve([BL3(1), BR2(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL3, BR2);
    large = larger(BL3, BR2);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        area2 = area2 - cirarea(small);
    elseif size(ix6,1)>1
        area2 = area2 - intersection_area(BL3, BR2); %the intersection of outer boundaries
    end

    area_list = [area_list area2];

    % considering the second inner circle region
    [ix7, iy] = solve([BL1(1), BR3(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL1, BR3);
    large = larger(BL1, BR3);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        area3 = cirarea(small);
    elseif size(ix7,1)>1
        area3 = intersection_area(BL1, BR3); %the intersection of outer boundaries
    else
        area3 = 0;
    end

    [ix8, iy] = solve([BL2(1), BR3(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL2, BR3);
    large = larger(BL2, BR3);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        area3 = area3 - cirarea(small);
    elseif size(ix8,1)>1
        area3 = area3 - intersection_area(BL2, BR3); %the intersection of outer boundaries
    end
    area_list = [area_list area3];

    % considering the two inner circle regions
    [ix9, iy] = solve([BL3(1), BR3(1)], [x y],'Real', true); % determine whether the outer boundaries intersect
    small = smaller(BL3, BR3);
    large = larger(BL3, BR3);
    dist = sqrt((large(4)-small(4))^2+(large(5)-small(5))^2)+small(6);
    if dist <= large(6)+0.000000001 % if the smaller boundary is completely inside the larger boundary
        area4 = cirarea(small);
    elseif size(ix9,1)>1
        area4 = intersection_area(BL3, BR3); %the intersection of outer boundaries
    else
        area4 = 0;
    end
    
    %% find the largest workspace region
    area_list = vpa([area_list area4])
    AT = max(area_list);
    fitness = 1/AT
    plot_robot([0 3], C2(1), linkl, d911, d10e, d11e, 0, Pi/4)

end

