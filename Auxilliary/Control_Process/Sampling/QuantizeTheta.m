function [theta_q, Count] = QuantizeTheta(theta, PPR)
% Absolute angle quantizer with signed counter output

% Counts per revolution
CPR     = 4 * PPR;
TWO_PI  = 2*pi;

% Quantize to signed counter
Count   = int32(floor(theta * (CPR / TWO_PI)));

% Reconstruct angle from counter
theta_q = double(Count) * (TWO_PI / CPR);

 
