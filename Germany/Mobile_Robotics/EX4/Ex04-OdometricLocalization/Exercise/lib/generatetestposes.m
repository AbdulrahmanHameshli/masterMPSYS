function xvec = generatetestposes(xstart)

% Scale factors
SX = 1.0;
SY = 1.0;

% Unit test poses
i = 1;
%xvec(1:2,i) = [-0.7  0.3]; i = i + 1;
xvec(1:2,i) = [-0.6  1.2]; i = i + 1;
xvec(1:2,i) = [ 0.0  1.0]; i = i + 1;
%xvec(1:2,i) = [ 0.5  0.8]; i = i + 1;
xvec(1:2,i) = [ 0.9  0.5]; i = i + 1;
xvec(1:2,i) = [ 1.7  0.0]; i = i + 1;
%xvec(1:2,i) = [ 0.9 -0.5]; i = i + 1;
xvec(1:2,i) = [ 0.5 -0.8]; i = i + 1;
%xvec(1:2,i) = [ 0.0 -1.0]; i = i + 1;
%xvec(1:2,i) = [-0.6 -1.2]; i = i + 1;
%xvec(1:2,i) = [-0.7 -0.3]; i = i + 1;

% Scale
S = eye(2);
S(1,1) = SX;
S(2,2) = SY;
xvec = S * xvec;

% Rotate
theta = xstart(3);
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
xvec = R * xvec;

% Translate
T = [xstart(1); xstart(2)]*ones(1,size(xvec,2));
xvec = xvec + T;