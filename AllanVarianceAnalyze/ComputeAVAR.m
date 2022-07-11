% Compute Allan Variance on an input signal
% Reproduced and slightly modified from original [1] for completeness.

%[1] Freescale Semiconductor, Inc, Allan Variance: Noise Analysis for
% Gyroscopes (February 2015) for completeness of package.
function [ad,t] = ComputeAVAR(ydot,dt)

pts = 500; % Arbitrary size of log spaced vector
[N,M] = size(ydot); % figure out how big the output data set is
n = 2.^(0:floor(log2(N/2)))'; % determine largest bin size
maxN = n(end);
endLogInc = log10(maxN);
% create log spaced vector average factor
m = unique(ceil(logspace(0,endLogInc,pts)))'; 
t = m*dt; % T = length of time for each cluster
% integration of samples over time to obtain output angle
theta = cumsum(ydot)*dt; 
% array of dimensions (cluster periods) X (#variables)
AV = zeros(length(t),M); 
for i=1:length(m) % loop over the various cluster sizes
    for k=1:N-2*m(i) % implements the summation in the AV equation
    AV(i,:) = AV(i,:) + (theta(k+2*m(i),:) -...
        2*theta(k+m(i),:) + theta(k,:)).^2;
    end
end
AV = AV./repmat((2*t.^2.*(N-2*m)),1,M); % Allan Variance
ad = sqrt(AV); % Allan Deviation