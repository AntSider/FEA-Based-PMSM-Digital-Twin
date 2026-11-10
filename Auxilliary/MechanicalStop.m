function TorqueNetLimited = MechanicalStop(Te, TorqueNet, Speed)
%#codegen
% Prevents load-induced rotation opposite to electromagnetic torque.
% Allows positive and negative electromagnetic torque commands.

% Speed tolerance
SpeedTolerance       = 1e-6;

% Stop shaft under opposing load torque
if abs(Speed) <= SpeedTolerance && ...
        ((Te > 0 && TorqueNet < 0) || (Te < 0 && TorqueNet > 0))

    TorqueNetLimited = 0;

else

    TorqueNetLimited = TorqueNet;

end

 