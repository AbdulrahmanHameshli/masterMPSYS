    % IMPLEMENT YOUR SOLUTION FROM HERE
    
    clear all
    
    clc
    data = load('point_correspondences.mat').point_correspondences;
    figure;
    hold on; 
    GlobalT = [0 0]';
    GlobalR = [1 0 ;  0 1 ];
    x_relaxed = [0 0 0 0 0 0]';
    x = [0 0 0];
    Omega = eye(2,2);
    
    for data_index =1:size(data,2)

        current_scan = data(data_index);
        current_scan_points = current_scan.points;
        current_scan_correspondence_with = current_scan.correspondence_with;
        current_scan_correspondences = current_scan.correspondences;
        current_scan_matching_points = transpose(current_scan_points(current_scan_correspondences(:,1), :));
    
        % Correspondence scan
        correspondence_scan = data(current_scan_correspondence_with);
        correspondence_scan_points = correspondence_scan.points;
        correspondence_scan_matching_points = transpose(correspondence_scan_points(current_scan_correspondences(:,2), :));

%         [pcl,T,R] = Relaxed_icp(current_scan_matching_points,correspondence_scan_matching_points,x_relaxed, Omega);
        [pcl,T,R,Theta] = Iterative_icp(current_scan_matching_points,correspondence_scan_matching_points,x, Omega);

%         x_relaxed = [GlobalR(1,1) GlobalR(1,2) GlobalR(2,1) GlobalR(2,2) GlobalT(1) GlobalT(2)]';
        x = [T;Theta];
%         x = [0 0 0.25*pi];

        GlobalT = GlobalR * T + GlobalT;
        GlobalR = GlobalR * R;
        Global_PCL = GlobalR * current_scan_matching_points + GlobalT;

        scatter(Global_PCL(1,:), Global_PCL(2,:), 20, 'filled',"b");
        scatter(GlobalT(1),GlobalT(2),"*","r")
        pause(0.001)

    end



function [rotated_PC ,T, R] = Relaxed_icp(current_scan_matching_points,correspondence_scan_matching_points,x, Omega)
            H = zeros(6,6);
            b = zeros(6,1);
     
            for points_index = 1: size(correspondence_scan_matching_points,2)
                r1 = x(1:2);
                r2 = x(3:4);
                T = x(5:6);
                h = [r1' ; r2'] * current_scan_matching_points(:,points_index) + T;
        
                j = [current_scan_matching_points(:,points_index)' 0 0 1 0 ;
                    0 0 current_scan_matching_points(:,points_index)' 0 1] ;
        
                e = h - correspondence_scan_matching_points(:,points_index);
                
                H = H + j' * Omega  * j;
%                 lambda = 1e-6; % Regularization parameter
%                 H = H + lambda * eye(size(H));
                b = b + j' * Omega * e;
                
            end
        
            dx = H \ (-b);
    
            x = x + dx;
            A = [x(1:2)';x(3:4)'];
            [U,D,V] = svd(A);
            R = U*V';
            
                
            if det(R) < 0
                R(:, end) = -R(:, end);
            end
            
            T = mean(correspondence_scan_matching_points,2) - R * mean(current_scan_matching_points,2);
            rotated_PC = R * current_scan_matching_points + T;
                   
       
end







function [plc ,T, R, Theta] =  Iterative_icp(P_new,P,x, Omega)
max_iterations = 100;
tolerance = 10^6;

for iterations=1:max_iterations
    
    H = zeros(3,3);
    b= zeros(3,1);
    t = [x(1);x(2)];
    Theta = x(3);

    for i=1:size(P_new,2)
        
        
        
        h = [cos(Theta) -sin(Theta);sin(Theta) cos(Theta)] * P_new(:,i) + t;
        e = h - P(:,i);
        j = [eye(2) [-sin(Theta) -cos(Theta); cos(Theta) -sin(Theta)]*P_new(:,i)];
        
        
        H = H + j' * Omega  * j;
        b = b + j' * Omega * e;
    end

        dx = H \ (-b);
        x = x + dx;
%         
%         if norm(dx) < tolerance
%             fprintf('Converged after %d iterations.\n', iterations);
%             break;
%         end
    
end

Theta = x(3);

R = [cos(Theta) -sin(Theta);sin(Theta) cos(Theta)];
T = [x(1);x(2)];

plc = [cos(Theta) -sin(Theta);sin(Theta) cos(Theta)] * P_new + [x(1);x(2)];
end