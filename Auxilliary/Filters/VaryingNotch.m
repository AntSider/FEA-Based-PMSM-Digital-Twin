function y = VaryingNotch(u, wn, gmin, zeta, Ts)
%#codegen
%
% Multi-channel time-varying notch filter.
%
% Each element of u is filtered independently.
%
% Inputs:
%   u     : Input vector [N x 1]
%   wn    : Notch angular frequency [rad/s]
%           Scalar or vector [N x 1]
%   gmin  : Gain at the notch center frequency
%           Scalar or vector [N x 1]
%   zeta  : Damping ratio
%           Scalar or vector [N x 1]
%   Ts    : Filter execution sample time [s]
%
% Output:
%   y     : Filtered output vector [N x 1]

persistent u1 u2 y1 y2

N = numel(u);

% Initialize independent states for all channels
if isempty(u1)
    u1 = zeros(N,1);
    u2 = zeros(N,1);
    y1 = zeros(N,1);
    y2 = zeros(N,1);
end

% Force a column-vector output
y = zeros(N,1);

% Bilinear transformation constant
K = 2.0 / Ts;

for k = 1:N

    % Select the notch frequency
    if numel(wn) == 1
        wn_k = wn;
    else
        wn_k = wn(k);
    end

    % Select the notch depth
    if numel(gmin) == 1
        gmin_k = gmin;
    else
        gmin_k = gmin(k);
    end

    % Select the damping ratio
    if numel(zeta) == 1
        zeta_k = zeta;
    else
        zeta_k = zeta(k);
    end

    % Numerator coefficients
    b0 = K^2 + 2.0*gmin_k*zeta_k*wn_k*K + wn_k^2;
    b1 = 2.0*(wn_k^2 - K^2);
    b2 = K^2 - 2.0*gmin_k*zeta_k*wn_k*K + wn_k^2;

    % Denominator coefficients
    a0 = K^2 + 2.0*zeta_k*wn_k*K + wn_k^2;
    a1 = 2.0*(wn_k^2 - K^2);
    a2 = K^2 - 2.0*zeta_k*wn_k*K + wn_k^2;

    % Second-order IIR difference equation
    y(k) = (b0*u(k)  + ...
            b1*u1(k) + ...
            b2*u2(k) - ...
            a1*y1(k) - ...
            a2*y2(k)) / a0;

    % Update input states
    u2(k) = u1(k);
    u1(k) = u(k);

    % Update output states
    y2(k) = y1(k);
    y1(k) = y(k);

end

end