P = load("P.mat").P;
P_new = load("P_new.mat").P_new;
%% IMPLEMENT YOUR SOLUTION FROM HERE
% A Closed ICP
figure
P_prim = P - mean(P,2);
P_new_prim =P_new - mean(P_new,2);
hold on
scatter(P(1,:),P(2,:),"b")
scatter(P_new(1,:),P_new(2,:),"r")
hold off


[U,D,V]=svd(P_prim *P_new_prim');
R = U * V';
if det(R) < 0
    R(:, end) = -R(:, end);
end

T = mean(P,2) - R*mean(P_new,2);
% R
P_new_rotated = R * P_new + T;
figure
hold on 
scatter(P(1,:),P(2,:),"r","x")
scatter(P_new_rotated(1,:),P_new_rotated(2,:),"b")
% mean((P-P_new_rotated).^2,2)

hold off

%%
% B Closed ICP


x = [0;0;0];

Omega = diag([1 1]);
max_iterations = 1000;
tolerance = 10^-6;
for iterations=1:max_iterations
    
    H = zeros(3,3);
    b= zeros(3,1);
    t = [x(1);x(2)];
    Theta = x(3);

    for i=1:size(P_new,2)
        
        
        
        h = [cos(Theta) -sin(Theta);sin(Theta) cos(Theta)] *        P_new(:,i) + t;
        e = h - P(:,i);
        j = [eye(2) [-sin(Theta) -cos(Theta); cos(Theta) -sin(Theta)]*P_new(:,i)];
        
        
        H = H + j' * Omega  * j;
        b = b + j' * Omega * e;
    end

        dx = H \ (-b);
        x = x + dx;
        
        if norm(dx) < tolerance
            fprintf('Converged after %d iterations.\n', iterations);
            break;
        end
    
end

Theta = x(3);
P_new_rotated_ICP = [cos(Theta) -sin(Theta);sin(Theta) cos(Theta)] * P_new + [x(1);x(2)];

figure
hold on
scatter(P(1,:),P(2,:),"r","x")
scatter(P_new_rotated_ICP(1,:),P_new_rotated_ICP(2,:),"b")
% mean((P-P_new_rotated_ICP).^2,2)
%%
% C



clear all

P = load("P.mat").P;
P_new = load("P_new.mat").P_new;
  
x = [0 0 0 0  10 2]';
Omega = eye(2,2);


for i =1 :2
    H = zeros(6,6);
    b = zeros(6,1);

    for points=1:size(P_new,2)
        r1 = x(1:2);
        r2 = x(3:4);
        T = x(5:6);
        h = [r1' ; r2'] * P_new(:,points) + T;
        j = [P_new(:,points)' 0 0 1 0 ;0 0 P_new(:,points)' 0 1];
        e = h - P(:,points);
        
        H = H + j' * Omega  * j;
        b = b + j' * Omega * e;
    
    end
    
    dx = H \ (-b);
    x = x + dx; 
end
A = [x(1:2)';x(3:4)'];
[U,D,V] = svd(A);
R = U*V';


if det(R) < 0
    R(:, end) = -R(:, end);
end

% T = me an(P, 2) - R * mean(P_new, 2);
T= [x(5)*D(2,2); x(5)*D(1,1) ];

P_new_rotated_ICP_relaxed = R * P_new +  T;



figure
hold on
scatter(P(1,:),P(2,:),"r","x")
scatter(P_new_rotated_ICP_relaxed(1,:),P_new_rotated_ICP_relaxed(2,:),"b")
% mean((P-P_new_rotated_ICP_relaxed).^2,2)



