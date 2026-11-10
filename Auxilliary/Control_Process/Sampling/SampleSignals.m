function SampledSignal = SampleSignals_Updated(Signal, TriSignal)
%#codegen
% Sample all elements of Signal when TriSignal reaches its positive peak.
% The sampled values are held until the next sampling instant.
%
% Inputs:
%   Signal       - Scalar or fixed-size vector of input signals
%   TriSignal    - Common triangular carrier signal
%
% Output:
%   SampledSignal - Sampled and held signal, same size as Signal

persistent LastSampleValues

% Persistent variables must be initialized before any other use
if isempty(LastSampleValues)
    LastSampleValues = zeros(size(Signal), 'like', Signal);
end

% Sample all signal elements simultaneously at the carrier peak
if abs(TriSignal - 1) <= 1e-6
    LastSampleValues = Signal;
end

% Zero-order hold output
SampledSignal = LastSampleValues;

