%% Note: This is a Matlab script and at the end of this file you
%% find definitions of functions that are used in this script. Read
%% through both the script and the functions in the end to gain an
%% understanding of the entire design and evaluation process. 

% Do some cleanup
clc
clear variables

%Local model differentiator filter design (lmfir_diff)
M=30;
m0=0;
p=5;
w0 = ones(2*M+1,1); %rectangular weights
w1 = chebwin(2*M+1); %Chebyshev weights
h = lmfir_diff(@monofun,@monodiff,p,M,m0,w0);
% The notation  @fun results into the function handle of a function
% named fun

% Load the reference signals
load hip2.mat
 
% Here are some sample plots to illustrate the behavior of your filter.
% Feel free to modify, re-use, or completely remove the following lines.

% Plot the filter coefficiencts and magnitude/phase response
figure(1);
stem(h);
title('Filter coefficients');

figure(2);
N_fft = 1e3;    %Zero-pad FFT for increased frequency resolution of
                %the DTFT
plot((0:N_fft-1)/N_fft,abs(fft(h, N_fft)));
title('Filter magnitude response');
xlabel('A frequency unit (which?)');
ylabel('|H|');

figure(3);
plot((0:N_fft-1)/N_fft,unwrap(angle(fft(h, N_fft))));
title('Filter phase response');
xlabel('A frequency unit (which?)');
ylabel('arg(H)');

% Plot the reference signals
figure(4);
plot(noisy_position);
hold on;
plot(true_position);
title('Reference signals');
ylabel('Some unit?');
xlabel('Some unit?');
legend('Noisy position', 'True position');

% Generate a plot of the periodogram of the noise
% We can "cheat" and get the noise by subtracting the true signal from the
%  measured position
n = noisy_position - true_position;
figure(5);
plot((0:N_fft-1)/N_fft,abs(fft(n,N_fft)).^2);
xlabel('Some frequency unit?');
ylabel('Periodogram of noise');
title('Frequency distribution of noise in measured position');


figure(6);
vhat_true0 = conv([1 -1],true_position); 
vhat_noisy0 = conv([1 -1],noisy_position);
plot([vhat_true0,vhat_noisy0])
axis([0 500 0 30]);
xlabel('Sample index');
ylabel('Some label?');
legend('true position','noisy position')
title('Velocity estimate trivial FIR');



figure(7);
vhat_true = conv(h, true_position); %%TBC: generate the velocity signal estimate 
                                       % from true_position by
                                       % convolving with designed lmfir_diff filter
    vhat_noisy = conv(h, noisy_position); %%TBC: generate the velocity signal estimate using
                                       % from noisy_position by
                                       % convolving with designed lmfir_diff filter
plot([vhat_true,vhat_noisy])
hold on
plot([vhat_true0]); 
hold off
axis([0 500 0 30]);
xlabel('Sample index');
ylabel('Some ylabel?');
legend('true position','noisy position','true position trivial fir')
title('Velocity estimate lmfir');


figure(2200)
y1 = [noisy_position; flip(noisy_position)];
y2 = [noisy_position; zeros(length(noisy_position),1)];
hold on
plot(y1)
plot(y2)
hold off

figure(2201)

hold on
plot(conv(h,y1))
plot(conv(h,y2))
hold off


%% Below are function definitions
%Evaluate the monomial f_i(m)
function f = monofun(i,m)
if i==0
    f = 1;
else
    f = m^(i);
end
end
% Evalute the monomial derivative, d/dm f_i(m) 
function fdiff = monodiff(i,m)
if i==0
    fdiff = 0;
elseif i==1
    fdiff = 1;
else
    fdiff = i*m^(i-1);
end
end



function [h,R] = lmfir(bfun,p,M,m0,w)
% Fir filter design based on local models
%
% bfunc = a function handle the basis function used
%         where bfun(i,m) should evaluate to the value for basis function index i and function argument m 
%     p = number of basis functions (indexed from 0 to p-1)
%     M = Window size. The disigned filter will be of length 2M+1
%     m0 = index of desired predicted value. For m0=0 the filter will provide symmetrix smoothing if $m0=M$ the
%         filter will provide the causal filtering output, i.e. sammple output is the esimate of the signal at tha last 
%         sample.       
%     w = vector with positive weights for the associated LS problem

if nargin<5,
    w = ones(2*M+1,1);
end
% Build R
R = zeros(2*M+1,p);
f = zeros(p,1);
for nidx=1:p,
    for k=1:(2*M+1)
        R(k,nidx) = bfun(nidx-1, (k-M-1));
    end
    f(nidx) = bfun(nidx-1,m0);
end
h = f'*inv(R'*diag(w)*R)*R'*diag(w);
h = h(end:-1:1);
end

function [h,R] = lmfir_diff(bfun,fpfun,p,M,m0,w)
% Fir filter design based on local models
%
% bfun = a function handle for the basis functions used
%         where bfun(i,m) should evaluate to the value for basis function index i and function argument m 
% fpfun = a function handle for the derivative of the basis function used
%         where fpfun(i,m) should evaluate to the derivative (w.r.t m) for basis function index i and function argument m 
%     p = number of basis functions (indexed from 0 to p-1)
%     M = Window size. The disigned filter will be of length 2M+1
%     m0 = index of desired predicted value. For m0=0 the filter will provide symmetric smoothing if $m0=M$ the
%         filter will provide the causal filtering output, i.e. sammple output is the esimate of the signal at tha last 
%         sample.       
%     w = vector with positive weights for the associated LS problem

if nargin<6,
    w = ones(2*M+1,1);
end

% Build R
R = zeros(2*M+1,p);
f = zeros(p,1);
for nidx=1:p,
    for k=1:(2*M+1)
        R(k,nidx) = bfun(nidx-1, (k-M-1));
    end
    f(nidx) = fpfun(nidx-1,m0);
end
h = transpose(f)*inv(transpose(R)*diag(w)*R)*transpose(R)*diag(w);; %%TBC Generate the vecor of the fir filter coefficents
      %%according to Equation (5) in PM 
h = h(end:-1:1);
end
