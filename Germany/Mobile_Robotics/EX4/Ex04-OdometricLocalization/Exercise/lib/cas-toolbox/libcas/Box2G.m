% [Gs,Gp] = Box2G(M);
%
% Casts the matrix M which holds a map in form of "box data" into the
% Gs- and Gp-structures with Gs(i).x = [xs ys xe ye] and Gp(i).x = [x y]
% v.1.0, 2003-08-04, koa
% v.1.1, 2003-08-07, koa: Gp for beacons added

function [Gs,Gp] = Box2G(M)

BEACON = 8;

%%% Segments
j = 1;
for i = 1:size(M,1),
  if M(i,5) ~= BEACON,
    rect = [min([M(i,1),M(i,3)]), min([M(i,2),M(i,4)]), max([M(i,1),M(i,3)]), max([M(i,2),M(i,4)])];
    % first segment
    xs = rect(1);
    ys = rect(2);
    xe = rect(1);
    ye = rect(4);
    Gs(j).x(1) = max([xs,xe]);
    Gs(j).x(2) = max([ys,ye]);
    Gs(j).x(3) = min([xs,xe]);
    Gs(j).x(4) = min([ys,ye]);
    Gs(j).c = zeros(4);
    Gs(j).id = j;
    Gs(j).lid = j;
    j = j + 1;
    % second segment
    xs = rect(1);
    ys = rect(4);
    xe = rect(3);
    ye = rect(4);
    Gs(j).x(1) = max([xs,xe]);
    Gs(j).x(2) = max([ys,ye]);
    Gs(j).x(3) = min([xs,xe]);
    Gs(j).x(4) = min([ys,ye]);
    Gs(j).c = zeros(4);
    Gs(j).id = j;
    Gs(j).lid = j;
    j = j + 1;
    % third segment
    xs = rect(3);
    ys = rect(4);
    xe = rect(3);
    ye = rect(2);
    Gs(j).x(1) = min([xs,xe]);
    Gs(j).x(2) = min([ys,ye]);
    Gs(j).x(3) = max([xs,xe]);
    Gs(j).x(4) = max([ys,ye]);
    Gs(j).c = zeros(4);
    Gs(j).id = j;
    Gs(j).lid = j;
    j = j + 1;
    % forth segment
    xs = rect(3);
    ys = rect(2);
    xe = rect(1);
    ye = rect(2);
    Gs(j).x(1) = min([xs,xe]);
    Gs(j).x(2) = min([ys,ye]);
    Gs(j).x(3) = max([xs,xe]);
    Gs(j).x(4) = max([ys,ye]);
    Gs(j).c = zeros(4);
    Gs(j).id = j;
    Gs(j).lid = j;
    j = j + 1;
  end;
end;

%%% Beacons
j = 1;
for i = 1:size(M,1),
  if M(i,5) == BEACON,
    Gp(j).x(1) = M(i,1);
    Gp(j).x(2) = M(i,2);
    Gp(j).id = j;
    j = j + 1;
  end;
end;

