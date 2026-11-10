function Out = ReplaceNaNInfs(In)
% Function, which corrects NaN or Infs

% Replace Inf or NaN Value with 0
if ( isinf(In) || isnan(In) )
    Out = 0;

else 
    Out = In;

end 