 
function [J1, J2, C1, C2, dm, ds,Cstar] = computeConstraints(wheels)
    % wheels: a set of wheels (see pdf)
    % Computes the matrices of constraints J1, J2 for pure rolling and
    % C1, C2 for non-slipping constraints, and the robot type in WMR taxonomy.

    J1 = []; J2 = []; C1 = []; C2 = [];
    C1s = []; 
    Cstar = []; 

    for i = 1:length(wheels)
        pose = wheels(i).pose;
        params = wheels(i).params;
        r = params(1);

        
        x = pose(1);
        y = pose(2);
        l = sqrt(x^2 + y^2);
        theta = pose(3);
        beta = theta - atan2(y, x);

        switch wheels(i).type
            case 0  % mn.,
                % Pure rolling constraint
                J1 = cat(1, J1, [-sin(theta), cos(theta), l * cos(beta)]);
                J2 = blkdiag(J2, r);
                
                % Nonslip constraint
                C1 = cat(1, C1, [cos(theta), sin(theta), l * sin(beta)]);
                C2 = cat(1, C2, 0);
                Cstar = cat(1, Cstar, [cos(theta), sin(theta), l * sin(beta)]);


            case 1  % Steered orientable standard wheel
                % Pure rolling constraint
                J1 = cat(1, J1, [-sin(theta), cos(theta), l * cos(beta)]);
                J2 = blkdiag(J2, r);
                
                % Nonslip constraint
                C1 = cat(1, C1, [cos(theta), sin(theta), l * sin(beta)]);
                C1s = cat(1, C1s, [cos(theta), sin(theta), l * sin(beta)]);  % For steerability
                Cstar = cat(1, Cstar, [cos(theta), sin(theta), l * sin(beta)]);

                C2 = cat(1, C2, 0);

            case {2, 3}  % Steered or passive caster wheel
                d = params(3);  % Offset for caster wheels
                % Pure rolling constraint
                J1 = cat(1, J1, [-sin(theta), cos(theta), l * cos(beta)]);
                J2 = blkdiag(J2, r);
                
                % Nonslip constraint
                C1 = cat(1, C1, [cos(theta), sin(theta), l * sin(beta)]);
                C2 = cat(1, C2, d);
            case 4  % Swedish wheel
                Gamma = params(4);  % Gamma angle for Swedish wheel
                % Only one rolling constraint
                J1 = cat(1, J1, [-sin(theta + Gamma), cos(theta + Gamma), l * cos(beta + Gamma)]);
                J2 = blkdiag(J2, r * cos(Gamma));

            otherwise
                error("Wheel type not supported");
        end
    end

    % Combine constraints for computing mobility and steerability

    % Compute degrees of mobility (dm) and steerability (ds)
    dm = 3 - rank(Cstar);
    ds = rank(C1s);
%     disp(Cstar)

end
