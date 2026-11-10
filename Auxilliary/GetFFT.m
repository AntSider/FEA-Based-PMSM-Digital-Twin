function FFT_RES = GetFFT(Signal, time)
% function, which perfoms an FFT analysis on a time signal
% Reference: "https://youtu.be/mkGsMWi_j4Q?si=dzH3tUWTBL6uYmU3"

% Refinement of Signal as a row vector
sizeOfSig   = size(Signal);
if sizeOfSig(1) ~= 1
    Signal  = Signal';
end 

%% Take abs and phase of signal
% Length of Signal 
L                      = length(Signal);

% % Time Vector 
% time                   = time;

% Sampling frequency
Fs                     = 1/mean(diff(time));

% Frequency domain vector
HarmFreq               = Fs*(0:(L/2))/L;

% Harmonic Order 
HarmOrder              = 1:length(HarmFreq);

% Compute the FFT of the signal
ComplexCoef            = fft(Signal);

% Compute the phase of the transform, removing small-magnitude transform values.
tol                    = 0.005;

% Compute the two-sided spectrum P2
TwoSidedSpect          = abs(ComplexCoef/L); % Normalize harmonic amplitudes, with division by L 

% Compute the single-sided spectrum P1
Harm_Amp               = TwoSidedSpect(1:fix(L/2)+1); 

% Multiply by 2 all frequency components, to get all the harmonic's energy right
Harm_Amp(2:end-1)      = 2*Harm_Amp(2:end-1); 
Harm_Amp( Harm_Amp < tol ) = 0; 

% Take the phase of the signal. This is equivalent to the phase offset with a cosine signal -> RefSignal = 1*cos(wt)
TwoSided_Phase         = wrapTo180(rad2deg(angle(ComplexCoef)));

% Find the Single Spectrum harmonic phases
Harm_Phase             = TwoSided_Phase(1:fix(L/2)+1);


% Harm_Phase(Harm_Amp < 1e-5 | Harm_Phase < 1e-6) = 0; % Fix too little values of FFT, due to discontinuous time axis

%% Save Output Data as a Struct
FFT_RES.HarmOrder   = HarmOrder;
FFT_RES.Amp         = Harm_Amp;
FFT_RES.Ang         = Harm_Phase;
FFT_RES.HarmFreq    = HarmFreq;
FFT_RES.ComplexCoef = ComplexCoef;