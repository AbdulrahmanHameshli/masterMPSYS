% helper script to initialize some variables
wheelr = 0.09; wheelw = 0.04; wheeld = 0.15;

c1.pose = [0; 0; 0];
c1.shape = [-0.2 0.2 0.2 -0.2 -0.2; 0 0 -0.4 -0.4 0];
w1(1).type = 0;
w1(1).pose = [0.2;0;0];
w1(1).params = [wheelr; wheelw];
w1(2).type = 0;
w1(2).pose = [-0.2; 0; 0];
w1(2).params = [wheelr; wheelw];
w1(3).type = 1;
w1(3).pose = [0; -0.35; pi/6];
w1(3).params = [wheelr; wheelw];

c2.pose = [0; 0; 0];
c2.shape = [-0.8 0.8 0.8 -0.8 -0.8; 0.1 0.1 -0.1 -0.1 0.1];
w2(1).type = 1;
w2(1).pose = [0.6;0;0];
w2(1).params = [wheelr; wheelw];
w2(2).type = 1;
w2(2).pose = [-0.6; 0; 0.2];
w2(2).params = [wheelr; wheelw];

w2(3).type = 2;
w2(3).pose = [0; -0.1; 0];
w2(3).params = [wheelr; wheelw; wheeld];
w2(4).type = 2;
w2(4).pose = [0; 0.1; 0];
w2(4).params = [wheelr; wheelw; wheeld];


c3.pose = [0; 0; 0];
c3.shape = [-0.4 0.4 0.4 -0.4 -0.4; 0.2 0.2 -0.2 -0.2 0.2];

w3(1).type = 0;
w3(1).pose = [0.3;0;0];
w3(1).params = [wheelr; wheelw];
w3(2).type = 0;
w3(2).pose = [-0.3; 0; 0];
w3(2).params = [wheelr; wheelw];

w3(3).type = 2;
w3(3).pose = [0.1; 0; 0];
w3(3).params = [wheelr; wheelw; wheeld];
w3(4).type = 2;
w3(4).pose = [-0.1; 0.0; 0];
w3(4).params = [wheelr; wheelw; wheeld];

c4.pose = [0; 0; 0];
c4.shape = [-0.3 0.3 0.3 -0.3 -0.3; 0.3 0.3 -0.3 -0.3 0.3];
w4(1).type = 2;
w4(1).pose = [0.3;0.3;0];
w4(1).params = [wheelr; wheelw; wheeld];
w4(2).type = 2;
w4(2).pose = [-0.3; 0.3; 0];
w4(2).params = [wheelr; wheelw; wheeld];
w4(3).type = 2;
w4(3).pose = [0; -0.3; 0];
w4(3).params = [wheelr; wheelw; wheeld];

c5.pose = [0; 0; 0];
c5.shape = [-0.3 0.3 0.3 -0.3 -0.3; 0.3 0.3 -0.3 -0.3 0.3];
w5(1).type = 1;
w5(1).pose = [0; 0; 0];
w5(1).params = [wheelr; wheelw];
w5(2).type = 2;
w5(2).pose = [0.3;0.3;0];
w5(2).params = [wheelr; wheelw; wheeld];
w5(3).type = 2;
w5(3).pose = [-0.3; 0.3; 0];
w5(3).params = [wheelr; wheelw; wheeld];

C6.pose = [0; 0; 0];
rsize = 0.7;
c = 0.22;
c60 = cos(pi/3); s60 = sin(pi/3);
shape = [
  c 1-c 1-c*c60 1-(1-c)*c60 (1-c)*c60 c*c60 c;
  0 0 c*s60 (1-c)*s60 (1-c)*s60 c*s60 0
  ];
shape = rsize*shape;
shape = shape - mean(shape(:,1:end-1),2);
C6.shape = [0 1; -1 0]*shape;

c7.pose = [0; 0; 0];
c7.shape = [-0.3 0.3 0.3 -0.3 -0.3; 0.3 0.3 -0.3 -0.3 0.3];
w7(1).type = 0;
w7(1).pose = [0.3;0.3;pi/12];
w7(1).params = [wheelr; wheelw];
w7(2).type = 0;
w7(2).pose = [-0.3; 0.3;pi/3];
w7(2).params = [wheelr; wheelw];
w7(3).type = 0;
w7(3).pose = [0; -0.3; 0];
w7(3).params = [wheelr; wheelw];