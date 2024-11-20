function qcdot = casterKin(q, qdot, casters)
% No longer part of the exercise
    [~,~,C1,C2,~,~] = computeconstraints(casters);

    qcdot = inv(C2) * C1 * [cos(q(3)) sin(q(3)) 0; -sin(q(3)) cos(q(3)) 0; 0 0 1] * qdot;
end

