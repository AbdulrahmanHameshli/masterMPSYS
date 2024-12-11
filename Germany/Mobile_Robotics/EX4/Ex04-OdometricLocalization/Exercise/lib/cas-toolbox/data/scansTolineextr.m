% scansTolineextr.m
%
% Script to transform LDOEM scans with nx6-syntax
%    index x y ignore ignore intensity
% to datastructures that are compatible with
% the toolbox implementation of the line
% extractino algorithm and call the algorithm.
%
% How to use: the scripts expects a matrix
% called SCAN in the file called fname
%
% v.1.0., 31.1.2006, koa (c) Nurobot

% column parameters
icolx = 1;
icoly = 2;
icolintens = 3;
% sensor parameters
STDRHO = 0.03;
scndata.params.stdrho = STDRHO; % constant radial uncertainty
% algorithm parameters
params.windowsize = 13;         % window size in points
params.threshfidel = 2;         % model fidelity threshold
params.minlength = 1.0;         % minimal segment length in [m]
params.cyclic = 0;              % cyclic data or not
params.compensa = 1*pi/180;     % heurist. compens. for raw data corr., [rad]
params.compensr = 0.01;         % heurist. compens. for raw data corr., [m]
params.fusealpha = 0.99999;      % signific. level for segment fusion

% read in scan data
fname = 'kai_trap_scan.m';
fnamenoext = fname(1:end-2);
eval(fnamenoext)
if exist('scan'),
  disp('Scan data matrix read in');
else
  disp('Error: could not find "scan" data matrix');
  break;
end;

% prepare scan data structure
% timestamp
scndata.steps.time = 0;
% scan length
len = size(scan,1);
scndata.steps.data1 = len;
% x data
scndata.steps.data2 = scan(:,icolx);
% y data
scndata.steps.data3 = scan(:,icoly);
% {R} to {S} transform
scndata.params.xs = [0; 0; 0];

% call line extraction
[L,segs,lines] = extractlines(scndata,params,1)

% % generate files for XO-target and TestNuLineExtr
% [phi,rho] = cart2pol(scan(:,2),scan(:,3));
% A(1:len,1) = phi;
% A(1:len,2) = rho;
% A(1:len,3) = 0;
% A(1:len,4) = STDRHO;
% A(1:len,5) = scan(:,icolintens); % intensity
% save(fnamenoext,'A','-ascii','-double');

