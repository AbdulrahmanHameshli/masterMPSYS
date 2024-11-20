function [u,zpred] = goToPos(type, q, goal, wheels)
% No longer part of the exercise
if type(1) == 3 && type(2) == 0
    % type (3,0)
    K = [cos(q(3)) -sin(q(3)) 0;
     sin(q(3))  cos(q(3)) 0;
     0                  0 1];
    

    zpred = q;

    err = [goal;q(3)] - zpred;
    if abs(err) < 5
        u = K\err;
    else
        u = K \ sign(err);
    end
elseif type(1) == 2
    if type(2) == 0
        % type (2,0)
        e = 0.1;
        delta = pi/2;
        zpred = [q(1)+e*cos(q(3)+delta);
                 q(2)+e*sin(q(3)+delta)];
        K = [-sin(q(3)) -e*sin(q(3)+delta); cos(q(3)) e*cos(q(3)+delta)];
        err = goal - zpred;
        if abs(err) < 5
            u = K\err;
        else
            u = K \ sign(err);
        end
    elseif type(2) == 1
        % type (2,1)
        e = 0.2;
        delta = pi/2;
        zpred = [q(1) + e*cos(q(3)+delta);
                 q(2) + e*sin(q(3)+delta);
                 q(4)];
        K = [(-sin(q(4) + q(3))), (-e*sin(delta + q(3))), 0;
             cos(q(4) + q(3)), e*cos(delta + q(3)), 0;
             0,0,1];
        u = zeros(3,1);
        err = [goal - zpred(1:2); diffangleunwrap(q(4), zpred(3))];
        if abs(err) < 4
            u = K\err;
        else
            u = K\[sign(err(1:2)); err(3)];
        end
    end
elseif type(1) == 1
    if type(2) == 1
        % type (1,1)
        L = point2line(wheels(1).pose(1:2), wheels(2).pose(1:2), wheels(3).pose(1:2));
        e = 0.1;
        zpred = [q(1) + L* sin(q(3)) + e*cos(q(3)+q(4));
                 q(2) - L* cos(q(3)) + e*sin(q(3)+q(4))];

        a = q(3) + q(4);
        K = [L*cos(a)-e*sin(a)*cos(q(4)), -e*sin(a);
             L*sin(a)-e*cos(a)*cos(q(4)),  e*cos(a)];
        err = goal - zpred;
        if abs(err) < 10
            u = K\err;
        else
            u = K \ sign(goal-zpred);
        end
    elseif type(2) == 2
        % type (1,2)
        
        L = norm(wheels(1).pose(1:2) - wheels(2).pose(1:2)) / 2;
        e = 0.1;
        zpred = [q(1)+L*cos(q(3))-e*sin(q(3)+q(4));
                 q(2)+L*sin(q(3))+e*cos(q(3)+q(4));
                 q(5)];
        K = [(e*cos(q(4) + q(3))*sin(q(4) - q(5)) - L*(sin(q(4))*sin(q(5) + q(3)) + sin(q(5))*sin(q(4) + q(3))) + L*sin(q(3))*sin(q(4) - q(5))), +(-e*cos(q(4) + q(3))), 0;
             (L*(sin(q(4))*cos(q(5) + q(3)) + sin(q(5))*cos(q(4) + q(3))) + e*sin(q(4) + q(3))*sin(q(4) - q(5)) - L*cos(q(3))*sin(q(4) - q(5))), (-e*sin(q(4) + q(3))), 0;
             0,0,1];
        if sin(q(5)) == 0
            % K is singular
            err = [goal;0] - zpred;
            if abs(err(1:2)) < 1
                u = pinv(K) * err;
            else
                u = pinv(K) * sign([goal;pi/3]-zpred);
            end
        else
            err = [goal;0] - zpred;
            if abs(err(1:2)) < 1
                u = K\err;
            else
                u = K \ sign([goal;pi/3]-zpred);
            end
        end
    end
else
    zpred = [nan;nan];
    u = zeros(3,1);
end
end




function d = point2line(P1, P2, P)
    % Extract coordinates
    x1 = P1(1); y1 = P1(2);
    x2 = P2(1); y2 = P2(2);
    x0 = P(1);  y0 = P(2);

    d = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1) / sqrt((y2 - y1)^2 + (x2 - x1)^2);
end