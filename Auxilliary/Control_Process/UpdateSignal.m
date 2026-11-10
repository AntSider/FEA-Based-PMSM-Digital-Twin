function y = UpdateSignal(ChangeTimes, Time, Values)
%#codegen
% ZOH scheduler: returns Values(:,k) where k = last index with ChangeTimes(k) <= Time

n   = numel(ChangeTimes);
tol = 1e-9;                       % numeric tolerance

% How many change times have passed?
k = sum(Time >= (ChangeTimes - tol));
if k < 1, k = 1; end
if k > n, k = n; end

% Support scalar vector or MxN matrix of values
if isvector(Values)
    % Accept row or column vector
    y = Values(min(k, numel(Values)));
else
    % Values is MxN (columns correspond to ChangeTimes)
    if size(Values,2) < k
        k = size(Values,2);
    end
    y = Values(:,k);
end

