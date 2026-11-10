function y = LPF_1stOrder(x, fc, Ts)
%#codegen
% x  : vector with any number of signals
% fc : cutoff frequency [Hz]
% Ts : sampling time [s]
% y  : filtered vector, same size as x

persistent yPrev

if isempty(yPrev)
    yPrev = zeros(size(x));
end

alpha = exp(-2*pi*fc*Ts);

y = alpha.*yPrev + (1-alpha).*x;

yPrev = y;

