%DRAWRECT Draw rounded rectangle.
%   DRAWRECT(X,W,H,R,FILLED,COLOR) draws a rectangle with
%   round corners of radius R, width W and height H, centered at
%   pose X where X is the 3x1 vector [x y theta]. With FILLED = 1
%   the rectangle is filled with color COLOR, with FILLED = 0
%   only the contour is drawn. COLOR is a [r g b]-vector or a
%   Matlab color string such as 'r' or 'g'.
%
%   Note that 2R must be greater or equal than the smaller of the
%   two values W,H. For 2R = W = H, DRAWRECT draws a circle.
%
%   H = DRAWRECT(...) returns the graphic handle H.
%
%   See also DRAWREFERENCE, PLOT.

% v.1.0, 11.10.03, Kai Arras, CAS-KTH


function h = drawrect(x,w,h,r,filled,color)

% Constants
RESO = pi/18;   % angular resolution of arc primitive

% Prepare vectors and transform matrices
arc = 0:RESO:pi/2;
n   = length(arc);
vec = ones(1,n);
T = [x(1); x(2)];
R = [cos(x(3)), -sin(x(3)); sin(x(3)), cos(x(3))];

% Compute and concatenate contour points
p  = [r*cos(arc); r*sin(arc)];
p1 = p               + [ w/2-r; h/2-r]*vec;
p2 = [ 0  1;-1  0]*p + [ w/2-r;-h/2+r]*vec;
p3 = [-1  0; 0 -1]*p + [-w/2+r;-h/2+r]*vec;
p4 = [ 0 -1; 1  0]*p + [-w/2+r; h/2-r]*vec;
p = cat(2,p1,p4,p3,p2);
p(:,4*n+1) = p(:,1);   % close contour

% Transform points to pose x
p = R*p + T*ones(1,4*n+1);

% Draw
if filled,
  %h = fill(p(1,:),p(2,:),color,'EdgeColor','none');
  h = fill(p(1,:),p(2,:),color);
  set(h,'EdgeColor','none');
else
  h = plot(p(1,:),p(2,:),'Color',color);
end
