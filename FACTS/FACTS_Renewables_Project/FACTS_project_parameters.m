Psc = 6000e6;              % Three-phase short-circuit power [VA]
Vp_rms = 150e3;            % RMS Phase voltage [V]
Vl_rms = sqrt(3)*Vp_rms;   % RMS line voltage (phase-to-phase) [V]
f = 60;                    % Line frequency [Hz]

L = (Vl_rms^2 / Psc)*(1 / (2*pi*f) );  % Internal inductance [H]
X_R = 10;                              % Reactance-to-resistance ratio
R = 2*pi*f*L/X_R;                      % Internal resistance [ohm]