function [omega, delta_theta] = SpeedFromTheta(theta, Ts, FILTER_COEFF)
% SpeedFromTheta
% Same logic as C code:
% - rollover handling around 2*pi
% - IIR speed filter

TWO_PI = 2*pi;

persistent theta_pre omega_state
if isempty(theta_pre)
    theta_pre    = theta;
    omega_state  = 0.0;
end

% Compute angle difference
delta_theta = theta - theta_pre;

% Handle rollover (prevent discontinuities)
if delta_theta < -0.9 * TWO_PI
    delta_theta = delta_theta + TWO_PI;
elseif delta_theta > 0.9 * TWO_PI
    delta_theta = delta_theta - TWO_PI;
end

% Raw speed
omega_raw   = delta_theta / Ts;

% Weighted moving average (IIR low-pass)
omega_state = (1.0 - FILTER_COEFF) * omega_state + FILTER_COEFF * omega_raw;

% Output
omega       = omega_state;

% Update previous theta
theta_pre   = theta;
