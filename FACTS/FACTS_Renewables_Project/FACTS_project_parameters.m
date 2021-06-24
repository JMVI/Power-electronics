Psc = 6000e6;              % Three-phase short-circuit power [VA]
Vp_rms = 150e3;            % RMS Phase voltage [V]
Vl_rms = sqrt(3)*Vp_rms;   % RMS line voltage (phase-to-phase) [V]
f = 60;                    % Line frequency [Hz]

L = (Vl_rms^2 / Psc)*(1/(2*pi*f));    % Internal inductance [H]
X_R = 10;                             % Reactance-to-resistance ratio
R = 2*pi*f*L/X_R;                     % Internal resistance [ohm]

% Transmission line parameters
R_lgth = 1.7e-5;
X_lgth = 3.5e-4;
L_tl = 25e3;

R_tl = R_lgth*L_tl;
X_tl = X_lgth*L_tl;
L_tl = X_tl/(2*pi*f);

RL = 14;
XL = 14.282;

Rload = RL - R_tl - R;
Xload = XL - X_tl - 2*pi*f*L;