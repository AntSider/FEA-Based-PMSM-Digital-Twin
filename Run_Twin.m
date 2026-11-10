% Function which runs Matlab && Simulink for motor control == (Standalone Model)

%% ~ Step 1. Initialization & call the path
% Clear all data
clear; close all; clc;

% Define current script directory
dir_Database    = fileparts(which('Run_Twin.m'));
cd(dir_Database);

% Add general path of directory
addpath(genpath(dir_Database));



%% ~ Step 2. Fetch FEA results

% Machine FEMM file
My_FEA          = 'FEA_Results';

% Path to whole model directory
FEMM_Path       = fullfile(dir_Database, My_FEA);

% Load FEA Data
LoadData        = load(fullfile(dir_Database, [My_FEA '.mat']));
FieldData       = fieldnames(LoadData);

% Create Struct FEA_Data
FEA_Data        = LoadData.(FieldData{1});

% Clear Unnecessary Data
clear LoadData FieldData;




%% ~ Step 3. Open Simulink Model for simulation

% Simulink model
model           = "Twin_Builder";

% Check if the model is loaded
if ~bdIsLoaded(model)
    open_system(fullfile(dir_Database, model + ".slx"));
end



%% == Step 4. Segregate FEA results to each component && Interpolate them accordingly 

%% Interpolating Methods: -- Is -> 'pchip'  ( to keep the saturation's effect in shape)
%%                        -- IEA-> 'spline' ( Te = k * Br * Bnet * sin(IEA), keep sinusoidal effect )

% Analysis' Vectors ....
IS_FEA          = FEA_Data.IS;             % Current Amplitude
IEA_FEA         = FEA_Data.IEA;            % Internal Electrical Angle
SR_FEA          = FEA_Data.SynchAngle;     % Synchronous DQ frame Electrical Angle
Speed_FEA       = FEA_Data.Speed_vec;      % Speed Vector used for Losses' Calculation
Fsw_FEA         = FEA_Data.fswV_Vec;       % Switching Frequency Vector used for Harmonic FEA

% Interpolating Vectors ....
InterpSamples   = 13; % InterpSamples = InterpSamples + 1;
IS_Interp       = linspace(FEA_Data.IS(1),  FEA_Data.IS(end),  InterpSamples);
IEA_Interp      = linspace(FEA_Data.IEA(1), FEA_Data.IEA(end), InterpSamples);

% Motor's Parameters
PolePair        = FEA_Data.Poles / 2;                                                                          % Motor's Pole Pair
J               = FEA_Data.Jinertia;                                                                           % Inertia of Motor
J_Control       = FEA_Data.Jinertia;
B 	            = 0.00001;                                                                                     % Viscocity of air molecules for motor's load
K               = 1;                                                                                           % Park Parameter - constant amplitude

% Optimal Current Distribution DQ
PMT_full         = FEA_Data.OptPMT_Table(:,1);
Speed_full       = FEA_Data.Speed_Table(1,:);
Env_full         = FEA_Data.EnvTorque_FW;

% Evenly Spaeced Vectors
PMT_Vec          = linspace(0,   100,   InterpSamples);
Speed_Vec        = linspace(min(Speed_full), max(Speed_full), InterpSamples);
MaxSpeed         = max(Speed_Vec);

% Envelope of Torque
EnvTorque_FW     = interp1(Speed_full, Env_full, Speed_Vec, 'pchip', 'extrap');

% Create interpolants
F_ID             = griddedInterpolant({PMT_full, Speed_full}, FEA_Data.OptIS .* cosd(FEA_Data.OptIEA));
F_IQ             = griddedInterpolant({PMT_full, Speed_full}, FEA_Data.OptIS .* sind(FEA_Data.OptIEA));

% Evaluate on new uniform grid
[PMT_grid, Speed_grid]...
                 = ndgrid(PMT_Vec, Speed_Vec);

% Interpolated PMT ID,Q
PMT_ID           = round( F_ID(PMT_grid, Speed_grid), 2);
PMT_IQ           = round( F_IQ(PMT_grid, Speed_grid), 2);


% == Interpolate FEA Data

% Torque
Torque_FEA      = InterpFEA_Data_ForSimulink(FEA_Data.TorqueFEA, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);      % Electromagnetic Torque 
MaxTe = max(Torque_FEA, [], 'all'); 

% === Lumped Parameters DQ ....
% Currents
ID_FEA          = InterpFEA_Data_ForSimulink(FEA_Data.ID, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);             % Current D
IQ_FEA          = InterpFEA_Data_ForSimulink(FEA_Data.IQ, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);             % Current Q

% Core Currents
FEA_Data.IDc( isnan(FEA_Data.IDc)  ) = 0;
FEA_Data.IQc( isnan(FEA_Data.IQc)  ) = 0;

% Interpolate Core Currents
IDcore_FEA      = InterpFEA_Data_ForSimulink(FEA_Data.IDc, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);            % Core Current D
IQcore_FEA      = InterpFEA_Data_ForSimulink(FEA_Data.IQc, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);            % Core Current Q

% Fluxes
FluxD_FEA       = InterpFEA_Data_ForSimulink(FEA_Data.FluxD, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);          % Flux Linkage D
FluxQ_FEA       = InterpFEA_Data_ForSimulink(FEA_Data.FluxQ, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);          % Flux Linkage Q

% Interpolate PM Flux at Irms
PMFlux_FEA      = interp1(IS_FEA, squeeze( FEA_Data.FluxD(:,IEA_FEA == 90, :) ), IS_Interp, 'pchip' );         % Permanent Magnet Flux Linkage
PMFlux_Single   = mean( PMFlux_FEA, 'all' );

% Inductances DQ
LD_FEA          = InterpFEA_Data_ForSimulink(FEA_Data.LD, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);             % Inductance D
LQ_FEA          = InterpFEA_Data_ForSimulink(FEA_Data.LQ, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);             % Inductance Q
LD_Single       = mean(LD_FEA, 'all');      % Single value of Total Inductance D
LQ_Single       = mean(LQ_FEA, 'all');      % Single value of Total Inductance Q

% Back EMF per Hz DQ
BEMFD_FEA_PerHz = InterpFEA_Data_ForSimulink(FEA_Data.BackEMFD_perHz, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Back EMF per Hz D
BEMFQ_FEA_PerHz = InterpFEA_Data_ForSimulink(FEA_Data.BackEMFQ_perHz, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Back EMF per Hz Q

% Back EMF per Hz A
BEMFA_PerHz_FEA = InterpFEA_Data_ForSimulink(FEA_Data.BackEMFA_perHz, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Back EMF per Hz A

% Resistances
Rseries_FEA     = InterpFEA_Data_ForSimulink(FEA_Data.Rs,    IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);        % Winding Series Resistance
RDC             = squeeze( Rseries_FEA(1,1,1) ); % DC Resistance
Rcore_FEA       = InterpFEA_Data_ForSimulink(FEA_Data.Rcore, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp);        % Traverse Branch - Core Loss Equivalent Resitance


% === Loss Parameters
% Losses Machine ...
FEA_Data.CopperLosses( isnan(FEA_Data.CopperLosses)  ) = 0;
CopperLoss_FEA  = InterpFEA_Data_ForSimulink(FEA_Data.CopperLosses, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Copper Losses
IronLoss_FEA    = InterpFEA_Data_ForSimulink(FEA_Data.IronLosses,   IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Iron Losses
ProxiLoss_FEA   = InterpFEA_Data_ForSimulink(FEA_Data.ProxiLosses,  IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Proximity Losses
MagnetLoss_FEA  = InterpFEA_Data_ForSimulink(FEA_Data.MagnetLosses, IS_FEA, IEA_FEA, IS_Interp, IEA_Interp); % Permanent Magnet Losses
WindageLoss_FEA = squeeze( FEA_Data.WindLosses(1,1,:) );   % Windage Losses
FrictLoss_FEA   = squeeze( FEA_Data.FrictLosses(1,1,:) );  % Friction Losses

% Losses Inverter ...
% Initialization
CondLoss_Inv    = zeros([length(IS_Interp), size(FEA_Data.ConductionLoss, [2 3] ) ]);
SwitchLoss_Inv  = zeros([length(IS_Interp), size(FEA_Data.ConductionLoss, [2 3] ) ]);
GateLoss_Inv    = zeros([length(IS_Interp), size(FEA_Data.ConductionLoss, [2 3] ) ]);

% Interpolate Inverter Losses
for iVDS = 1 : size(FEA_Data.ConductionLoss, 2)

    for iFsw = 1 : size(FEA_Data.SwitchingLoss, 3)
        CondLoss_Inv(:, iVDS, iFsw)   = interp1( IS_FEA, squeeze(FEA_Data.ConductionLoss(:, iVDS, iFsw)), IS_Interp, 'pchip');
        SwitchLoss_Inv(:, iVDS, iFsw) = interp1( IS_FEA, squeeze(FEA_Data.SwitchingLoss(:, iVDS, iFsw)),  IS_Interp, 'pchip');
        GateLoss_Inv(:, iVDS, iFsw)   = interp1( IS_FEA, squeeze(FEA_Data.GateLoss(:, iVDS, iFsw)),       IS_Interp, 'pchip');

    end % Sweep Fsw

end % Sweep VDS

% Maximum Torque's Lead Angle (delta)
TeMean          = mean(Torque_FEA, 3);

% Find linear index of the maximum value in the whole 3D matrix
[~, idx_linear] = max( TeMean(:) );

% Convert linear index to 2D subscripts (i, k)
[~, k]          = ind2sub(size(TeMean), idx_linear);

% Final Optimal IEA
IEA_Command     = IEA_Interp(k);

% Transform FEA Input Data to Interpolated Data
IS_FEA          = IS_Interp;
IEA_FEA         = IEA_Interp;





%% == Step 5. Determine Control Settings for PWM Modulation Strategies 

% DC Voltage
Vdc	            = 56;

% Maximum AC current
Ismax           = 70;

% Modulation Index maximum
ModInd_Limit    = 0.98;                     % Limit Modulation Index  [0 - 1]

% Control Type
DriveMethod       = 1;  % 0: Voltage Control, 1: Current Control, 2: Speed Control, 3: Torque Control, 4: Position Control

% Modulation Strategy
ModStrategy       = 1;  % 0: SPWM 1:SVPWM 2:THIPWM 3:THZSPWM 4:DPWM1 5:DPWMMAX 6:DPWMMIN

% Switching frequency
f_sw            = 10e3;

% Sampling time
Ts              = 1/f_sw;

% Dead time
Tdt             = 0.3 * 1e-6; % 300ns

% Fundamental DQ Bandwidths
% Bandwiths
tr	            = 5e-3;                    % Current Control's Trise

% Current Control's Bandwith
ac	            = log(9)/tr;

% Speed Control's Bandwidth
as	            = 0.08 * ac;

% Position Control's Bandwidth
ath	            = 0.25 * as;

% Linear modulation region max AC voltage
VsMax           = Vdc/2;

% Overmodulate Reference Voltage
if ModStrategy == 0,  OverMod = 1* ModInd_Limit;      % SPWM
else,                 OverMod = 1.1547 * ModInd_Limit; % SVPWM, THIPWM, THZSPWM, DPWM1
end

% Continuous or Discontinious PWM
if     ModStrategy == 4,                     RedSwitching = 1/3;   % DPWM1
elseif ModStrategy == 5 || ModStrategy == 6, RedSwitching = 1/6;   % DPWMMin or DPWMMax
else,                                        RedSwitching = 1;     % Continuous PWM
end




%% == Step 6. Choose motor excitation from Inverter (Duty Cycle Ref, Current Ref, Speed Ref, Throttle Ref) 

%%%%% Voltage Control %%%%%
% Modulation Index Amplitude
Vs_pu           = 0.98;                        % Modulation Index Command [pu]

% Vs_Vec     = [0.21, 0.31, 0.44, 0.56, 0.625, 0.82, 0.94]
% Speed_Vec  = [50, 80, 120, 160, 180,  240, 280, 300];

%%%%% Current Control  %%%%%
% Current's Amplitude
Is_Amp          = 40;                          % Current Command          [A]

%%%%% Speed Control  %%%%%
% Speed to achieve
speedTarget     = 200;                         % Speed Command            [RPM]

%%%%% Torque Control  %%%%%
% Torque percentage to maximum Torque
Throttle        = 45;                          % Throttle Command         [%]

% Shaft's Mechanical Load
mechLoad        = 1;                          % Shaft's Load


% T_Shaft = [0.3, 0.9, 1.8, 2.7, 3.6, 4.5, 5.3, 6.2, 7, 8, 9, 9.8, 12.19, 14.16, 15.79];




%% == Step 7. Determine Simulation time, according to how many mechanical rotations we need  

% Simulation time -> 1 Mechanical Rotation, Pole Pairs*Mech Rotation Electrical Rotations
% Number of Mechanical Rotation
mechRotations   = 1;

% AD2S1205 Evaluation Board - Resolver Data - Resolver Poles -> 4
PPR             = 1024;  % Pulses per Revolution

% Low Pass Filter for Speed estimation
FILTER_COEFF    = 0.23;
Ts_Sp           = 2 * Ts;

% Mechanical Angular Velocity [Rad/sec]
wMechTarget     = ceil(2 * pi * abs(speedTarget) / 60);   % Angular Velocity

% Radians
mechThetaTarget = mechRotations * 2 * pi;

% Simulation time
simTime         = mechThetaTarget / wMechTarget; % [Sec]

% Evaluate speed controller response with load changes or reference speed changes
Times           = [0, 0.2, 0.4, 0.6, 0.8] * simTime;
ShafthLoad      = [10, 25, 3, 8, 10];         % [Start, Increase, Decrease, Increase, Decrease]
RefSpeed        = [50, 400, 250, 500, 400]; % [Start, Increase, Decrease, Increase, Decrease]
RefTheta        = [50, 0, 140, 300, -50]; % [Start, Increase, Decrease, Increase, Decrease]



%% == Step 8. Run Simulation 

% Take output structure
out             = sim(model);







%% ==== Results ==== %%
% Time
time               = out.tout;

% Continuously Rotated Theta Shaft
ThetaShaft         = out.ThetaShaft;


%%% Lumped Parameters ABC %%%

% Reference Voltages ABC
VaRef              = squeeze( out.VabcRef(1, :, :) );
VbRef              = squeeze( out.VabcRef(2, :, :) );
VcRef              = squeeze( out.VabcRef(3, :, :) );

% Inverter's Phase Voltages
if out.VabcInv(:, 1) == 0
    Va_Inv         = VaRef;
    Vb_Inv         = VbRef;
    Vc_Inv         = VcRef;
else
    Va_Inv         = out.VabcInv(:, 1);
    Vb_Inv         = out.VabcInv(:, 2);
    Vc_Inv         = out.VabcInv(:, 3);
end

% Back EMF ABC
bEMFA              = out.backEMFs(:,1);
bEMFB              = out.backEMFs(:,2);
bEMFC              = out.backEMFs(:,3);

% Currents ABC
Ia                 = out.Iabc(:,1);
Ib                 = out.Iabc(:,2);
Ic                 = out.Iabc(:,3);


%%% Lumped Parameters DQ %%%

% Reference Current
ID_Ref             = out.ID_Ref;
IQ_Ref             = out.IQ_Ref;

% Currents DQ
Id                 = out.IdqMotor(:,1);
Iq                 = out.IdqMotor(:,2);

% Inductances DQ
Ld                 = out.L_DQ(:,1);
Lq                 = out.L_DQ(:,2);

% BEMF DQ
BEMFD              = out.backEMFs_DQ(:,1);
BEMFQ              = out.backEMFs_DQ(:,2);

% Flux DQ
FluxD              = out.Flux_DQ(:,1);
FluxQ              = out.Flux_DQ(:,2);


%%% Losses %%%

% Iron Losses
IronLoss           = out.IronLosses;

% Copper Losses
CopperLoss         = out.CopperLosses;

% Proximity Losses
ProxiLoss          = out.ProxiLosses;

% Magnet Losses
MagnetLoss         = out.MagnetLosses;

% Total Losses
PLoss              = out.TotalLosses;


%%% Inverter Losses %%%

% Conduction Losses
ConductionLoss     = out.ConductionLosses;

% Switching Losses
SwitchingLoss      = out.SwitchingLosses;

% Gate Losses
GateLoss           = out.GateLosses;

% Inverter Losses
InverterLoss       = ConductionLoss + SwitchingLoss + GateLoss;


%%% Power Data %%%

% Pout
Pout               = out.Pout;

% Pin
Pin                = Pout + PLoss;

% Torque, Speed
T_Shaft            = out.T_Shaft;
SpeedRPM           = out.SpeedRPM;

% Efficiency Motor
EfficiencyMot      = out.Efficiency;

% Efficiency System
EfficiencySys      = 100 * Pout ./ (Pout + PLoss + InverterLoss);

% Efficiency Inverter
EfficiencyInv      = 100 * Pin ./ (Pin + InverterLoss);


  

%%%% Results Window and Plot Results %%%%

% Position Control: Entire Simulation
% Other Drive Methods: Last Electrical Period

if DriveMethod == 4
    % Entire simulation
    idx                     = 1:length(time);

else
    % Last electrical period
    [~,idxStart]            = min(abs(ThetaShaft-(ThetaShaft(end)-2*pi)));
    idx                     = idxStart:length(time);
end


%%% Time
time_1P                     = time(idx);


%%% ABC

% VABC Reference
Va_Ref_1P                   = VaRef(idx);
Vb_Ref_1P                   = VbRef(idx);
Vc_Ref_1P                   = VcRef(idx);

% Motor Voltages
Va_Inv_1P                   = Va_Inv(idx);
Vb_Inv_1P                   = Vb_Inv(idx);
Vc_Inv_1P                   = Vc_Inv(idx);

% Motor Currents
Ia_Inv_1P                   = Ia(idx);
Ib_Inv_1P                   = Ib(idx);
Ic_Inv_1P                   = Ic(idx);

% Motor BEMFs
BEMFa_Inv_1P                = bEMFA(idx);
BEMFb_Inv_1P                = bEMFB(idx);
BEMFc_Inv_1P                = bEMFC(idx);


%%% DQ

% IDQ
Id_1P                       = Id(idx);
Iq_1P                       = Iq(idx);

% Reference IDQ
Idr_1P                      = ID_Ref(idx);
Iqr_1P                      = IQ_Ref(idx);

% BEMFDQ
BEMFD_1P                    = BEMFD(idx);
BEMFQ_1P                    = BEMFQ(idx);

% LDQ
Ld_1P                       = Ld(idx);
Lq_1P                       = Lq(idx);

% FluxDQ
FluxD_1P                    = FluxD(idx);
FluxQ_1P                    = FluxQ(idx);


%%% Losses

% Machine Losses
IronLoss_1P                 = IronLoss(idx);
CopperLoss_1P               = CopperLoss(idx);
ProxiLoss_1P                = ProxiLoss(idx);
MagnetLoss_1P               = MagnetLoss(idx);
PLoss_1P                    = PLoss(idx);

% Inverter Losses
InverterLoss_1P             = InverterLoss(idx);


%%% Power

% Input Power
Pin_1P                      = Pin(idx);

% Output Power
Pout_1P                     = Pout(idx);

% Torque & Speed
T_Shaft_1P                  = T_Shaft(idx);
SpeedRPM_1P                 = SpeedRPM(idx);

% Efficiency Machine
EfficiencyMot_1P            = EfficiencyMot(idx);

% Efficiency Inverter
EfficiencyInv_1P            = EfficiencyInv(idx);

% Efficiency System
EfficiencySys_1P            = EfficiencySys(idx);


%%% Perform FFT

if DriveMethod ~= 4
    FFT_V                   = GetFFT(Va_Inv_1P,time_1P);
    FFT_I                   = GetFFT(Ia_Inv_1P,time_1P);
    FFT_EMF                 = GetFFT(BEMFa_Inv_1P,time_1P);
    FFT_Te                  = GetFFT(T_Shaft_1P,time_1P);
end




%%%% Plot Results %%%%
% Settings
Font_Size                   = 18;
LineWidth                   = 1.5;






%% ~~ Plot ABC ~~
figure(); hold on; set(gcf,'color','w');

% -- Plot VABC
s = subplot(2,2,1);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Va_Inv_1P,'LineWidth',LineWidth);
plot(time_1P,Vb_Inv_1P,'LineWidth',LineWidth);
plot(time_1P,Vc_Inv_1P,'LineWidth',LineWidth);
axis tight;

xlabel('Time[s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('VABC[V]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('VABC','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');


% -- Plot VABC Ref
s = subplot(2,2,2);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Va_Ref_1P,'LineWidth',LineWidth);
plot(time_1P,Vb_Ref_1P,'LineWidth',LineWidth);
plot(time_1P,Vc_Ref_1P,'LineWidth',LineWidth);
axis tight;

xlabel('Time[s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('VABC Ref','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('VABC Ref','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');


% -- Plot Currents
s = subplot(2,2,3);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Ia_Inv_1P,'LineWidth',LineWidth);
plot(time_1P,Ib_Inv_1P,'LineWidth',LineWidth);
plot(time_1P,Ic_Inv_1P,'LineWidth',LineWidth);
axis tight;

xlabel('Time[s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('Current[A]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('ABC Currents','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');


% -- Plot BEMFs
s = subplot(2,2,4);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,BEMFa_Inv_1P,'LineWidth',LineWidth);
plot(time_1P,BEMFb_Inv_1P,'LineWidth',LineWidth);
plot(time_1P,BEMFc_Inv_1P,'LineWidth',LineWidth);
axis tight;

xlabel('Time[s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('BEMF[V]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('ABC BEMF','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');

tempTitle = sprintf('ABC Voltages \\& Currents');
tempTitle = append('\underline{\textbf{',tempTitle,'}}');
sgtitle(tempTitle,'FontName','Times New Roman','FontSize',1.25*Font_Size,'Interpreter','latex');



%% ~~ Plot Phase A Fourier ~~
if DriveMethod ~= 4

    figure(); hold on; set(gcf,'color','w');

    % Phase-A Voltage
    s = subplot(2,3,1);
    set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;
    plot(time_1P,Va_Inv_1P,'LineWidth',LineWidth);
    axis tight;
    xlabel('Time[s]','Interpreter','latex');
    ylabel('VA[V]','Interpreter','latex');
    title('VA','Interpreter','latex');

    % Phase-A Voltage FFT
    s = subplot(2,3,4);
    set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;
    bar(FFT_V.HarmOrder(2:end-1),FFT_V.Amp(2:end-1));
    axis tight;
    xlabel('$\mathrm{Harmonics}[\nu]$','Interpreter','latex');
    ylabel('VA FFT[V]','Interpreter','latex');
    title('VA FFT','Interpreter','latex');

    % Phase-A Current
    s = subplot(2,3,2);
    set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;
    plot(time_1P,Ia_Inv_1P,'LineWidth',LineWidth);
    axis tight;
    xlabel('Time[s]','Interpreter','latex');
    ylabel('IA[A]','Interpreter','latex');
    title('IA','Interpreter','latex');

    % Phase-A Current FFT
    s = subplot(2,3,5);
    set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;
    bar(FFT_I.HarmOrder(2:40)-1,FFT_I.Amp(2:40));
    axis tight;
    xlabel('$\mathrm{Harmonics}[\nu]$','Interpreter','latex');
    ylabel('IA FFT[A]','Interpreter','latex');
    title('IA FFT','Interpreter','latex');

    % Phase-A Back EMF
    s = subplot(2,3,3);
    set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;
    plot(time_1P,BEMFa_Inv_1P,'LineWidth',LineWidth);
    axis tight;
    xlabel('Time[s]','Interpreter','latex');
    ylabel('BEMFA[V]','Interpreter','latex');
    title('BEMFA','Interpreter','latex');

    % Phase-A Back EMF FFT
    s = subplot(2,3,6);
    set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;
    bar(FFT_EMF.HarmOrder(2:40)-1,FFT_EMF.Amp(2:40));
    axis tight;
    xlabel('$\mathrm{Harmonics}[\nu]$','Interpreter','latex');
    ylabel('BEMFA[V]','Interpreter','latex');
    title('BEMFA FFT','Interpreter','latex');

    sgtitle('\underline{\textbf{VI FFT}}', ...
        'FontName','Times New Roman','FontSize',1.25*Font_Size,'Interpreter','latex');

end



%% ~~ Plot DQ ~~
figure(); hold on; set(gcf,'color','w');

% -- Subplot 1: Id and Iq
s = subplot(2,2,1);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Id_1P,'LineWidth',LineWidth);
plot(time_1P,Iq_1P,'LineWidth',LineWidth);

xlabel('Time [s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('Current [A]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('Id, Iq Currents','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
legend({'$I_d$','$I_q$'},'Interpreter','latex','Location','best');


% -- Subplot 2: FluxD and FluxQ
s = subplot(2,2,2);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,FluxD_1P,'LineWidth',LineWidth);
plot(time_1P,FluxQ_1P,'LineWidth',LineWidth);

xlabel('Time [s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('Flux [Vs]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('Flux Linkages','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
legend({'$\psi_d$','$\psi_q$'},'Interpreter','latex','Location','best');


% -- Subplot 3: BEMFd and BEMFq
s = subplot(2,2,3);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,BEMFD_1P,'LineWidth',LineWidth);
plot(time_1P,BEMFQ_1P,'LineWidth',LineWidth);

xlabel('Time [s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('Voltage [V]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('BEMF dq','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
legend({'$e_d$','$e_q$'},'Interpreter','latex','Location','best');


% -- Subplot 4: Ld and Lq
s = subplot(2,2,4);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Ld_1P,'LineWidth',LineWidth);
plot(time_1P,Lq_1P,'LineWidth',LineWidth);

xlabel('Time [s]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
ylabel('Inductance [H]','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
title('Ld, Lq Inductances','FontName','Times New Roman','FontSize',Font_Size,'Interpreter','latex');
legend({'$L_d$','$L_q$'},'Interpreter','latex','Location','best');

sgtitle('\underline{\textbf{DQ Frame: Currents, Flux, BEMF, Inductance}}', ...
    'FontName','Times New Roman','FontSize',1.25*Font_Size,'Interpreter','latex');



%% ~~ Plot Losses and Efficiency Breakdown ~~
figure(); set(gcf,'Color','w');

% --- Subplot 1: Inverter Losses ---
s1 = subplot(3,1,1);
set(s1,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add','GridLineStyle','--');

plot(time_1P,InverterLoss_1P,'r','LineWidth',LineWidth);
xlabel('Time [s]','Interpreter','latex');
ylabel('Loss [W]','Interpreter','latex');
yline(mean(InverterLoss_1P),'-', ...
    sprintf('Mean = %.2f W',mean(InverterLoss_1P)), ...
    'Interpreter','latex','FontSize',Font_Size);
title('\textbf{Inverter Losses}','Interpreter','latex');
grid on; axis tight;


% --- Subplot 2: Machine Losses ---
s2 = subplot(3,1,2);
set(s2,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add','GridLineStyle','--');

plot(time_1P,PLoss_1P,'b','LineWidth',LineWidth);
xlabel('Time [s]','Interpreter','latex');
ylabel('Loss [W]','Interpreter','latex');
yline(mean(PLoss_1P),'-', ...
    sprintf('Mean = %.2f W',mean(PLoss_1P)), ...
    'Interpreter','latex','FontSize',Font_Size);
title('\textbf{Machine Losses}','Interpreter','latex');
grid on; axis tight;


% --- Subplot 3: System Efficiency ---
s3 = subplot(3,1,3);
set(s3,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add','GridLineStyle','--');

plot(time_1P,EfficiencySys_1P,'b','LineWidth',LineWidth);
xlabel('Time [s]','Interpreter','latex');
ylabel('Efficiency [\%]','Interpreter','latex');

yline(mean(EfficiencySys_1P),'-', ...
    sprintf('\\textbf{Mean Efficiency = %.2f\\%%}',mean(EfficiencySys_1P)), ...
    'Interpreter','latex','FontSize',Font_Size, ...
    'LabelVerticalAlignment','bottom', ...
    'LabelHorizontalAlignment','right');

title('\textbf{System Efficiency}','Interpreter','latex');
grid on; axis tight;

sgtitle('\underline{\textbf{Time-Domain System Efficiency}}', ...
    'Interpreter','latex','FontName','Times New Roman','FontSize',1.25*Font_Size);



%% ~~ Plot Losses and Efficiency Breakdown ~~
figure(); set(gcf,'color','w');

% Subplot 1: Individual Loss Components
s = subplot(2,2,1);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,IronLoss_1P,'LineWidth',LineWidth);
plot(time_1P,CopperLoss_1P,'LineWidth',LineWidth);
plot(time_1P,ProxiLoss_1P,'LineWidth',LineWidth);
plot(time_1P,MagnetLoss_1P,'LineWidth',LineWidth);
plot(time_1P,PLoss_1P,'b','LineWidth',LineWidth);

yline(mean(PLoss_1P),'-', ...
    sprintf('Mean = %.2fW',mean(PLoss_1P)), ...
    'FontSize',Font_Size,'FontWeight','bold', ...
    'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');

axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Losses [W]','Interpreter','latex');
title('\textbf{Loss Breakdown}','Interpreter','latex');
legend({'Iron','Copper','Proximity','Magnet','Total'}, ...
    'FontSize',0.8*Font_Size,'Interpreter','latex','Location','best');


% Subplot 2: Output Power
s = subplot(2,2,2);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Pout_1P,'m','LineWidth',LineWidth);
axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Pout [W]','Interpreter','latex');
title('\textbf{Output Power}','Interpreter','latex');


% Subplot 3: Input Power
s = subplot(2,2,3);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,Pin_1P,'LineWidth',LineWidth);
axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Pin [W]','Interpreter','latex');
title('\textbf{Input Power}','Interpreter','latex');


% Subplot 4: Efficiency
s = subplot(2,2,4);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,EfficiencyMot_1P,'r','LineWidth',LineWidth);
yline(mean(EfficiencyMot_1P),'-', ...
    sprintf('Mean = %.2f%%',mean(EfficiencyMot_1P)), ...
    'FontSize',Font_Size,'FontWeight','bold', ...
    'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');

axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Efficiency [\%]','Interpreter','latex');
title('\textbf{Efficiency}','Interpreter','latex');

sgtitle('\underline{\textbf{Power and Loss Analysis}}', ...
    'FontName','Times New Roman','FontSize',1.25*Font_Size,'Interpreter','latex');



%% ~~ Plot Copper Losses ~~
figure(); set(gcf,'color','w');

s = subplot(1,1,1);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,CopperLoss_1P,'LineWidth',LineWidth,'Color','r');

yline(mean(CopperLoss_1P),'k-', ...
    sprintf('Mean = %.2f W',mean(CopperLoss_1P)), ...
    'FontSize',Font_Size,'FontWeight','bold', ...
    'LabelHorizontalAlignment','left','LabelVerticalAlignment','bottom');

axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Copper Loss [W]','Interpreter','latex');
title('\textbf{Copper Losses}','Interpreter','latex');
legend({'Copper'},'FontSize',0.8*Font_Size,'Interpreter','latex','Location','best');



%% ~~ Plot Losses and Efficiency Breakdown (3-panel) ~~
figure(); set(gcf,'color','w');

% Subplot 1: Torque
s = subplot(3,1,1);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,T_Shaft_1P,'r','LineWidth',LineWidth);
axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Torque [Nm]','Interpreter','latex');
title('\textbf{Torque}','Interpreter','latex');


% Subplot 2: Speed
s = subplot(3,1,2);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,SpeedRPM_1P,'k','LineWidth',LineWidth);
axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Speed [RPM]','Interpreter','latex');
title('\textbf{Speed}','Interpreter','latex');


% Subplot 3: Efficiency
s = subplot(3,1,3);
set(s,'FontName','Times New Roman','FontSize',Font_Size,'NextPlot','add'); grid on;

plot(time_1P,EfficiencyMot_1P,'g','LineWidth',LineWidth);
axis tight;
xlabel('Time [s]','Interpreter','latex');
ylabel('Efficiency [\%]','Interpreter','latex');
title('\textbf{Efficiency}','Interpreter','latex');

sgtitle('\underline{\textbf{Torque, Speed, and Efficiency}}', ...
    'FontName','Times New Roman','FontSize',1.25*Font_Size,'Interpreter','latex');





%% ~~ Plot Controllers Current Response ~~
switch DriveMethod

    case 1
        controlType = 'Current Control';

    case 2
        controlType = 'Speed Control';

    case 3
        controlType = 'Torque Control';

    case 4
        controlType = 'Position Control';

    otherwise
        controlType = 'Open Loop';
end



%% Speed Control
if DriveMethod == 2

    %% ~~ Figure: Reference vs Speed ~~ Sweep Loads
    figure('Color','w');
    ax = axes('FontName','Times New Roman','FontSize',Font_Size);
    hold(ax,'on'); grid(ax,'on');

    if isscalar(speedTarget)

        p1 = yline(speedTarget,'r-', ...
            'LineWidth',LineWidth,'DisplayName','Ref speed');

    else

        p1 = plot(time,speedTarget,'--', ...
            'LineWidth',LineWidth,'DisplayName','Ref speed');

    end

    p2 = plot(time,SpeedRPM,'-', ...
        'LineWidth',LineWidth,'DisplayName','Measured speed');

    axis tight; box off;
    xlabel('Time [s]','Interpreter','latex');
    ylabel('Speed [rpm]','Interpreter','latex');
    title('Mechanical Speed','Interpreter','latex');
    legend([p1 p2],'Location','best','Interpreter','latex','Box','off');


    %% ~~ Figure: Reference vs Speed ~~ Sweep Ref Speeds
    figure('Color','w');
    ax = axes('FontName','Times New Roman','FontSize',Font_Size);
    hold(ax,'on'); grid(ax,'on');

    if numel(RefSpeed) == numel(time)

        Ref_on_time = RefSpeed(:);

    elseif exist('Times','var') && numel(RefSpeed) == numel(Times)

        Ref_on_time = interp1(Times(:),RefSpeed(:),time(:),'previous','extrap');

    else

        tref = linspace(time(1),time(end),numel(RefSpeed));
        Ref_on_time = interp1(tref,RefSpeed(:),time(:),'previous','extrap');

    end

    p1 = plot(time,Ref_on_time,'--', ...
        'LineWidth',max(0.5,0.9*LineWidth), ...
        'DisplayName','Ref speed');

    p2 = plot(time,SpeedRPM,'-', ...
        'LineWidth',LineWidth, ...
        'DisplayName','Measured speed');

    axis tight; box off;
    xlabel('Time [s]','Interpreter','latex');
    ylabel('Speed [rpm]','Interpreter','latex');
    title('Mechanical Speed','Interpreter','latex');
    legend([p1 p2],'Location','best','Interpreter','latex','Box','off');

end



%% ~~ Plot Reference vs Actual Currents ~~
if DriveMethod ~= 0

    figure(); set(gcf,'color','w');

    % 1. Id: Ref vs Actual
    s = subplot(2,2,1);
    set(s,'FontName','Times New Roman','FontSize',Font_Size, ...
        'FontWeight','bold','NextPlot','add'); grid on;

    plot(time_1P,Idr_1P,'LineWidth',LineWidth,'DisplayName','Id\_Ref');
    plot(time_1P,Id_1P,'LineWidth',LineWidth,'DisplayName','Id');

    xlabel('Time [s]','Interpreter','latex');
    ylabel('$I_d$ [A]','Interpreter','latex');
    title('\textbf{Direct Axis Current}','Interpreter','latex');
    legend('show','Location','best','FontSize',Font_Size*0.8,'Interpreter','latex');


    % 2. Iq: Ref vs Actual
    s = subplot(2,2,2);
    set(s,'FontName','Times New Roman','FontSize',Font_Size, ...
        'FontWeight','bold','NextPlot','add'); grid on;

    plot(time_1P,Iqr_1P,'LineWidth',LineWidth,'DisplayName','Iq\_Ref');
    plot(time_1P,Iq_1P,'LineWidth',LineWidth,'DisplayName','Iq');

    xlabel('Time [s]','Interpreter','latex');
    ylabel('$I_q$ [A]','Interpreter','latex');
    title('\textbf{Quadrature Axis Current}','Interpreter','latex');
    legend('show','Location','best','FontSize',Font_Size*0.8,'Interpreter','latex');


    % 3. Speed
    s = subplot(2,2,3);
    set(s,'FontName','Times New Roman','FontSize',Font_Size, ...
        'FontWeight','bold','NextPlot','add'); grid on;

    plot(time_1P,SpeedRPM_1P,'b','LineWidth',LineWidth);

    xlabel('Time [s]','Interpreter','latex');
    ylabel('Speed [RPM]','Interpreter','latex');
    title('\textbf{Speed}','Interpreter','latex');


    % 4. Torque
    s = subplot(2,2,4);
    set(s,'FontName','Times New Roman','FontSize',Font_Size, ...
        'FontWeight','bold','NextPlot','add'); grid on;

    plot(time_1P,T_Shaft_1P,'m','LineWidth',LineWidth);

    xlabel('Time [s]','Interpreter','latex');
    ylabel('Torque [Nm]','Interpreter','latex');
    title('\textbf{Torque}','Interpreter','latex');

    sgtitle(['\underline{\textbf{Controller Currents, Speed, and Torque (',controlType,')}}'], ...
        'FontName','Times New Roman','FontSize',1.1*Font_Size,'Interpreter','latex');

end



%% Position Control
if DriveMethod == 4

    % Sampled time
    Sampled_Time       = out.Sampled_Time;

    % Reference Mechanical Position
    Theta_Ref          = out.Theta_Ref;

    % Measured Mechanical Position
    Theta_MechShaft    = rad2deg(out.Theta_MechShaft);


    %% ~~ Figure: Reference vs Actual Position ~~
    figure('Color','w');
    ax = axes('FontName','Times New Roman','FontSize',Font_Size);
    hold(ax,'on'); grid(ax,'on');

    % Reference mechanical position
    p1 = plot(Sampled_Time,Theta_Ref,'--', ...
        'LineWidth',LineWidth, ...
        'DisplayName','Ref position');

    % Measured mechanical position
    p2 = plot(Sampled_Time,Theta_MechShaft,'-', ...
        'LineWidth',LineWidth, ...
        'DisplayName','Measured position');

    axis tight; box off;
    xlabel('Time [s]','Interpreter','latex');
    ylabel('Position [deg]','Interpreter','latex');
    title('Mechanical Position','Interpreter','latex');
    legend([p1 p2],'Location','best','Interpreter','latex','Box','off');

end
















