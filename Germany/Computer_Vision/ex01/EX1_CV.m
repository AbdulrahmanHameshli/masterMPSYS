% H 1.1
% a
disp("------------------------------------------------------------")
% Define the 5x5 signal based on the image
signal = [0 0 0 0 0;
          0 0 0 0 0;
          5 5 0 0 0;
          5 5 0 0 0;
          5 5 0 0 0];

% Initialize matrices for x and y derivatives
fx = zeros(size(signal));
fy = zeros(size(signal));

% Compute the spatial derivatives for the inner 3x3 region
for x = 2:4
    for y = 2:4
        fx(x, y) = (signal(x, y+1) - signal(x, y-1)) / 2;
        fy(x, y) = (signal(x+1, y) - signal(x-1, y)) / 2;
    end
end

fx
fy

% b & c
disp("------------------------------------------------------------")
% Initialize a 3x3 cell array to store the classifications
classifications = cell(3,3);

% Loop through each pixel in the 3x3 region to compute J and classify
for i = 2:4
    for j = 2:4
        % Compute structure tensor for the current pixel
        disp("J" + i + "," + j)
        J0 = compute_structure_tensor(fx(i,j), fy(i,j))
        % Compute eigenvalues of J0
        eigenvalues = eig(J0);
        lambda1 = eigenvalues(1);
        lambda2 = eigenvalues(2);

        % Classify pixel based on eigenvalues
        if lambda1 < 1e-5 && lambda2 < 1e-5
            classifications{i-1, j-1} = "Flat Area";
        elseif (lambda1 > 1e-5 && lambda2 < 1e-5) || (lambda1 < 1e-5 && lambda2 > 1e-5)
            classifications{i-1, j-1} = "Edge";
        else
            classifications{i-1, j-1} = "Corner";
        end
    end
end

% Display the classifications for each pixel in the inner 3x3 region
disp("Classifications for each pixel in the 3x3 region:");
disp(classifications);

% d
disp("------------------------------------------------------------")
% Define the binomial kernel
kernel = (1/16) * [1 2 1; 2 4 2; 1 2 1];

% Compute structure tensors for each inner pixel and store them in matrices
Jxx = zeros(3, 3);
Jyy = zeros(3, 3);
Jxy = zeros(3, 3);

% Calculate structure tensor components for each pixel in the inner 3x3 region
for i = 1:3
    for j = 1:3
        % Compute structure tensor for the current pixel
        current_pixel_j = compute_structure_tensor(fx(i, j), fy(i, j));
        
        % Store each component of J0 separately for convolution
        Jxx(i, j) = current_pixel_j(1, 1);
        Jxy(i, j) = current_pixel_j(1, 2);
        Jyy(i, j) = current_pixel_j(2, 2);
    end
end

% Convolve each component with the binomial kernel
Jxx_smoothed = conv2(Jxx, kernel, 'same');
Jyy_smoothed = conv2(Jyy, kernel, 'same');
Jxy_smoothed = conv2(Jxy, kernel, 'same');

% Extract the central pixel's smoothed structure tensor components
Jxx_center = Jxx_smoothed(2, 2);
Jyy_center = Jyy_smoothed(2, 2);
Jxy_center = Jxy_smoothed(2, 2);

% Reconstruct the smoothed structure tensor for the central pixel
J_smoothed = [Jxx_center, Jxy_center; Jxy_center, Jyy_center];

% Calculate eigenvalues of the smoothed structure tensor
eigenvalues = eig(J_smoothed);
lambda1 = eigenvalues(1);
lambda2 = eigenvalues(2);

% Classify the central pixel based on the smoothed structure tensor eigenvalues
if lambda1 < 1e-5 && lambda2 < 1e-5
    central_pixel_classification = "Flat Area";
elseif (lambda1 > 1e-5 && lambda2 < 1e-5) || (lambda1 < 1e-5 && lambda2 > 1e-5)
    central_pixel_classification = "Edge";
else
    central_pixel_classification = "Corner";
end

% Display the smoothed structure tensor and classification
disp("Smoothed Structure Tensor for Central Pixel:");
disp(J_smoothed);
disp("Classification of Central Pixel after Convolution:");
disp(central_pixel_classification);

% e
disp("------------------------------------------------------------")
% Define the 3x3 matrices for R, G, and B channels
R = [5 5 0; 5 5 0; 5 5 0];
G = [0 0 5; 0 0 5; 0 0 5];
B = [0 0 0; 0 0 0; 0 0 0];

% Central pixel coordinates
i = 2;
j = 2;

% Compute gradients in x- and y-directions for each channel

% R channel
fx_R = (R(i+1, j) - R(i-1, j)) / 2;
fy_R = (R(i, j+1) - R(i, j-1)) / 2;

% G channel
fx_G = (G(i+1, j) - G(i-1, j)) / 2;
fy_G = (G(i, j+1) - G(i, j-1)) / 2;

% B channel (all zeros, so gradient is zero)
fx_B = 0;
fy_B = 0;

% Sum of gradients
fx_sum = fx_R + fx_G + fx_B;
fy_sum = fy_R + fy_G + fy_B;

% Compute the norm of the sum of gradients
norm_sum_gradients = sqrt(fx_sum^2 + fy_sum^2);

% Display the result
disp("Norm of the sum of gradients at the central pixel:");
disp(norm_sum_gradients);

% f
disp("------------------------------------------------------------")
% Define the 3x3 matrices for R, G, and B channels
R = [5 5 0; 5 5 0; 5 5 0];
G = [0 0 5; 0 0 5; 0 0 5];
B = [0 0 0; 0 0 0; 0 0 0];

% Central pixel coordinates
i = 2;
j = 2;

% Compute gradients in x- and y-directions for each channel

% R channel
fx_R = (R(i+1, j) - R(i-1, j)) / 2;
fy_R = (R(i, j+1) - R(i, j-1)) / 2;
gradient_norm_R = sqrt(fx_R^2 + fy_R^2);

% G channel
fx_G = (G(i+1, j) - G(i-1, j)) / 2;
fy_G = (G(i, j+1) - G(i, j-1)) / 2;
gradient_norm_G = sqrt(fx_G^2 + fy_G^2);

% B channel (all zeros, so gradient is zero)
fx_B = 0;
fy_B = 0;
gradient_norm_B = sqrt(fx_B^2 + fy_B^2);

% Joint Gradient Vector
joint_gradient_vector = [gradient_norm_R; gradient_norm_G; gradient_norm_B];

% Calculate the norm of the Joint Gradient Vector
joint_color_gradient = norm(joint_gradient_vector);

% Display the result
disp("Joint Color Gradient at the central pixel:");
disp(joint_color_gradient);

% g
disp("------------------------------------------------------------")
% Define the 3x3 matrices for R, G, and B channels
R = [5 5 0; 5 5 0; 5 5 0];
G = [0 0 5; 0 0 5; 0 0 5];
B = [0 0 0; 0 0 0; 0 0 0];

% Central pixel coordinates
i = 2;
j = 2;

% Compute gradients in x- and y-directions for each channel

% R channel
fx_R = (R(i, j+1) - R(i, j-1)) / 2;
fy_R = (R(i+1, j) - R(i-1, j)) / 2;
grad_R = [fx_R; fy_R];

% G channel
fx_G = (G(i, j+1) - G(i, j-1)) / 2;
fy_G = (G(i+1, j) - G(i-1, j)) / 2;
grad_G = [fx_G; fy_G];

% B channel (all zeros, so gradient is zero)
fx_B = 0;
fy_B = 0;
grad_B = [fx_B; fy_B];

% Compute the structure tensor for each channel
tensor_R = grad_R * grad_R'
tensor_G = grad_G * grad_G'
tensor_B = grad_B * grad_B'

% Sum the structure tensors to get the joint color structure tensor
joint_color_structure_tensor = tensor_R + tensor_G + tensor_B;

% Display the result
disp("Joint Color Structure Tensor at the central pixel:");
disp(joint_color_structure_tensor);

function J0 = compute_structure_tensor(fx, fy)
    % Compute the structure tensor for given fx and fy matrices in a region

    % Element-wise square of fx and fy
    fx_squared = fx .^ 2;
    fy_squared = fy .^ 2;
    
    % Element-wise product of fx and fy
    fx_fy = fx * fy;

    % Construct the structure tensor
    J0 = [fx_squared, fx_fy; 
          fx_fy, fy_squared];
end