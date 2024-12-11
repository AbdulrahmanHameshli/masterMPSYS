% Script that implements a thins out too noisy and dense laser data
% Kai Arras, Dec 2006
%
% Annahme: der Scan liegt in the matrix "points" vor, welche die Struktur
%  x1 x2 ... xn
%  y1 y2 ... yn
% hat. Hier ein Beispiel-Code zum Einlesen:

% read in scan data
fname = 'scanFelix2.m';
eval(fname(1:end-2))
if ~exist('scan'), disp('Error: could not find "scan" data matrix'); break; end;

% x,y data -> points, horizontal vector
icolx = 2;
icoly = 3;
points = [scan(:,icolx), scan(:,icoly)]';

% plot input scan and start point
figure(1); clf; set(gca,'Box','On'); hold on;
plot(points(1,:),points(2,:),'o','MarkerSize',5,'Color',[1 .6 .6]);
plot(points(1,1),points(2,1),'rd'); axis equal;

% Filter Parameters
Dm = 0.2;   % thin-out distance, this will be the minimal distance between points
DM = 0.5;   % jump distance, jumps bigger than DM will be preserved

% Filter Algorithm -> resulting scan will be in matrix "pointsthin"
len = size(points,2);            % determine number of points in input scan
clear pointsthin;                % init the target data structure
k = 1;                           % init the target index k
pointsthin(1:2,k) = points(1:2,1);   % always insert first point
pcurr = pointsthin(1:2,k);       % last inserted point becomes current focus point pcurr
i = 2;                           % init scan index to 2 as first points is already inserted
while i <= len,                  % loop stops when scan index reaches len
  %h = plot(pcurr(1),pcurr(2),'kd');
  %axis([pcurr(1)-1 pcurr(1)+1 pcurr(2)-1 pcurr(2)+1]);
  pnext = points(1:2,i);         % store point i
  d = norm(pcurr-pnext);         % calculate Euclidian distance d between points
  if d < DM,                     % test if d is smaller than jump distance DM
    if d < Dm,                   % if yes, test if d is smaller than thin out distance Dm
      clear buf;                 % if yes, init point buffer array buf
      j = 1;                     % init point buffer index j
      buf(1:2,j) = pnext;        % always insert point i
      %buf(3,j) = plot(pnext(1),pnext(2),'co');
      pold = pnext;              % store point i (used for jump detection)
      while (d < Dm) & (i+j <= len),   % iterate until i+j reach len or the mean distance d is > Dm
        pnew = points(1:2,i+j);  % store point i+j pnew
        buf(1:2,j+1) = pnew;     % insert pnew into buffer
        %buf(3,j+1) = plot(pnew(1),pnew(2),'mo');
        dn = norm(pold-pnew);    % calculate Eucl. distance dn between neighboring points
        if dn < DM,              % test if dn is smaller than DM
          pm = mean(buf(1:2,:),2);   % if yes (normal case), average over all points in buffer -> pm
          d = norm(pcurr-pm);    % calculate Eucl. distance d between current focus point and pm
          pold = pnew;           % pnew becomes pold
        else                     % else case means that jump has occurred (during averaging)
          pm = pnew;             % if jump, forget buffer and take pnew as new point  
          d = Inf;               % set d to infinity to abort while loop
        end;
        j = j + 1;               % increase buffer index j in any case
      end;
      k = k + 1;                 % increase target index k before insert
      pointsthin(1:2,k) = pm;    % insert mean point pm (or pnew in case of jump)
      pcurr = pm;                % last inserted point becomes current focus point pcurr
      %plot(pm(1),pm(2),'b+','MarkerSize',5); 
      %pause;
      %for ii = 1:size(buf,2), delete(buf(3,ii)); end;
      i = i + j;                 % increase scan index i to i+j to start at new point 
    else                         % else case means that no filtering needed, points far enough
      k = k + 1;                 % increase target index k before insert
      pointsthin(1:2,k) = pnext; % insert point i
      pcurr = pnext;             % last inserted point becomes current focus point pcurr
      %plot(pnext(1),pnext(2),'g+','MarkerSize',5);
      %pause;
      i = i + 1;                 % increase scan index i
    end;
    %delete(h);
  else                           % else case means that jump has occurred (at new focus point)
    k = k + 1;                   % increase target index k before insert
    pointsthin(1:2,k) = pnext;   % insert point i
    pcurr = pnext;               % last inserted point becomes current focus point pcurr
    %plot(pnext(1),pnext(2),'r+','MarkerSize',5);
    i = i + 1;                   % increase scan index i
  end;
end;

% plot resulting scan
plot(pointsthin(1,:),pointsthin(2,:),'b+','MarkerSize',5);