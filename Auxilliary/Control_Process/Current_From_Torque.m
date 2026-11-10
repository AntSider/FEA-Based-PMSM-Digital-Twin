function [IdRef, IqRef] = Current_From_Torque(TeCommand, IEA_Command, Psimag, Ld, Lq, PolePair)

%% == Step 1. Torque to Current Coefficients == %%

% Current angle
delta                   = deg2rad(IEA_Command);

% Requested torque magnitude
TeRef                   = abs(TeCommand);

% Torque direction
TorqueSign              = sign(TeCommand);

% Torque coefficients
A                       = 1.5*PolePair*(Ld-Lq)*cos(delta)*sin(delta);
C                       = 1.5*PolePair*Psimag*sin(delta);



%% == Step 2. Calculate Current Magnitude == %%

% Initialization
IsRef                   = 0;

if abs(A) < 1e-12

    % Linear torque equation
    if C > 1e-12
        IsRef           = TeRef/C;
    end

else

    % Quadratic discriminant
    D                   = C^2 + 4*A*TeRef;

    if D >= 0

        % Two possible current magnitudes
        Is1             = (-C + sqrt(D))/(2*A);
        Is2             = (-C - sqrt(D))/(2*A);

        % Select smallest nonnegative solution
        if Is1 >= 0 && Is2 >= 0
            IsRef       = min(Is1,Is2);
        elseif Is1 >= 0
            IsRef       = Is1;
        elseif Is2 >= 0
            IsRef       = Is2;
        end

    end

end



%% == Step 3. Calculate dq Current References == %%

IdRef                   = IsRef*cos(delta);
IqRef                   = TorqueSign*IsRef*sin(delta);

 