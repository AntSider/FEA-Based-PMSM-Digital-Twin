function InterpOut = InterpFEA_Data_ForSimulink(RawIn, RawVec1, RawVec2, InterpVec1, InterpVec2)

%% Interpolation across IS 
% Initialize Interpolared Output
InterpOut_Vec1 = zeros( [ length(InterpVec1), size(RawIn, [2 3]) ] );

% Interpolation Starts across Current
for iVec = 1 : length(RawVec2)
    InterpOut_Vec1(:, iVec, :) = interp1(RawVec1,  RawIn(:,iVec,:) , InterpVec1, 'pchip');
end

%% Interpolation across IEA 
% Initialize Interpolared Output 
InterpOut      = zeros( [ length(InterpVec1), length(InterpVec2), size(RawIn, 3) ] );

% Interpolation Starts across Vec2
for iVec = 1 : length(InterpVec1)
    InterpOut(iVec,:,:)        = interp1(RawVec2', squeeze( InterpOut_Vec1(iVec,:,:) ), InterpVec2', 'spline');
end