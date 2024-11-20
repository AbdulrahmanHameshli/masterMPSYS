function G = fwdKin(type,q, wheels)
% No longer part of the exercise
if type(1) == 3 && type(2) == 0
    % type (3,0)
    G = [cos(q(3)) -sin(q(3)) 0;
         sin(q(3))  cos(q(3)) 0;
         0                  0 1];
elseif type(1) == 2
    if type(2) == 0
        % type (2,0)
    G = [-sin(q(3)) 0;
          cos(q(3)) 0;
                  0 1];
    elseif type(2) == 1
        % type (2,1)
        G = [-sin(q(3)+q(4)) 0 0;
              cos(q(3)+q(4)) 0 0;
                           0 1 0;
                           0 0 1];
    end
elseif type(1) == 1
    if type(2) == 1
        % type (1,1)
        L = point2line(wheels(1).pose(1:2), wheels(2).pose(1:2), wheels(3).pose(1:2));
        G = [-L*sin(q(3))*sin(q(4)) 0;
              L*cos(q(3))*sin(q(4)) 0;
                          cos(q(4)) 0;
                                  0 1];

    elseif type(2) == 2
        % type (1,2)
        L = norm(wheels(1).pose(1:2) - wheels(2).pose(1:2)) / 2;
        G = [-L*(sin(q(4))*sin(q(3)+q(5))+sin(q(5))*sin(q(3)+q(4))), 0, 0;
              L*(sin(q(4))*cos(q(3)+q(5))+sin(q(5))*cos(q(3)+q(4))), 0, 0;
                                                     sin(q(5)-q(4)), 0, 0;
                                                                  0, 1, 0;
                                                                  0, 0, 1];
    
    end
else
    % robot is non degenerate
    G = zeros(size(q));
end
end

function d = point2line(P1, P2, P)
    % Extract coordinates
    x1 = P1(1); y1 = P1(2);
    x2 = P2(1); y2 = P2(2);
    x0 = P(1);  y0 = P(2);

    d = abs((y2 - y1)*x0 - (x2 - x1)*y0 + x2*y1 - y2*x1) / sqrt((y2 - y1)^2 + (x2 - x1)^2);
end