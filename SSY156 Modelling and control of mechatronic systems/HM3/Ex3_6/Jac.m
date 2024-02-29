function J = Jac(t,z,p,j,joints)
%JAC Computes Geometric JAcobian
%   This function computes the geometric jacobian
%Input:
% t: List of link positions, including t0_0 (n+1)
% z: list of axis of motion, including z0_0 (n+1)
% p: target point in link id
% j: link id > 0 <= n
% joints: List of joint types (n)

syms J real

z3D=[0;0;0];

% DOFs
n=numel(t)-1;

J(6,n)=0;

% n Columns
for i=1:n
    % Check if the joint contributes to changes in velocities
    if(i<=j)
        %Check the type of joint
        if joints{i}=='R'
            % Revolute Joint
            v=simplify(cross(z{i},(p-t{i})));
            w=z{i};
        else
            % Prismatic Joint
            v=z{i};
            w=z3D;
        end
    else
        %NO contribution to the velocity
        v=z3D;
        w=z3D;
    end
    J(1:6,i)=[v;w];
end



end

