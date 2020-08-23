% ------------------------------------------------------------------------------
% Copyright (C) 2020 Jaime M. Villegas I. <jaime7592@gmail.com>
% ------------------------------------------------------------------------------
% Filename      : Buck_Boost.m
% Description   : DC analysis of non-ideal Buck-Boost Converter
% Version       : 01.00
% Revision      : 00
% Last modified : 08/20/2020
% References    : R.W. Erickson, D. Maksimovic. "Fundamentals of Power
%                 Electronics". 2nd ed. (2004)
% ------------------------------------------------------------------------------
clear
clc

% Converter parameters

Vg = -10;         % Input voltage
V = 25;           % Output voltage
d = V/(V-Vg);     % Ideal duty-cycle
Iout = 2;         % Output current
Rload = V/Iout;   % Load resistance
fs = 100e3;       % Switching frequency
Ts = 1/fs;        % Switching period
IL_ripple = 0.2;  % Inductor current ripple
IC_ripple = 0.1;  % Capacitor current ripple
VC_ripple = 0.25; % Capacitor voltage ripple

L = -Vg*d*Ts/(2*IL_ripple);                      % Inductor
C = -Vg*(d^2)*Ts/(2*Rload*((1-d)^2)*VC_ripple);  % Capacitor

Dmin = 0.005;
Dmax = 0.995;
D = Dmin:0.001:Dmax;   % Converter duty cycle

% Diode
Rd = 10e-3;  % Diode on-state resistance
Vd = 0.7;    % Diode forward voltage

% Transistor
Rt = 20e-3;  % Transistor on-state resistance

% Inductor
K_RL = [0.01 0.05 0.1];   % Load resistance to inductor resistance ratio
RL = K_RL*Rload;          % Inductor resistance

% Effective losses
for i = 1:3 Re(i,:) = D*Rt + (1-D)*Rd + RL(i);end
Ve = (1-D)*Vd;

% Conversion ratio
M_BBs_ideal = -D./(1-D);
M_BBs_real = -Rload*(1-D).*(Vg*D + Ve)./(Rload*(1-D).^2 + Re);

% Efficiency
Eff_BBs = Rload*(1 + Ve./(D*Vg))./(Rload + Re./(1-D).^2);

figure(1)
subplot(2,2,1)
plot(D, Re(1,:), 'b', D, Re(2,:), 'r', D, Re(3,:), 'g')
xlabel("Duty Cycle (D)")
ylabel("Re(D)")
title("Effective resistance - Buck-Boost converter")

subplot(2,2,2)
plot(D, Ve)
xlabel("Duty Cycle (D)")
ylabel("Ve(D)")
title("Effective voltage drop - Buck-Boost converter")

subplot(2,2,3)
plot(D, M_BBs_ideal, 'k', D, M_BBs_real(1,:), 'b', D, M_BBs_real(2,:), 'r', D, M_BBs_real(3,:), 'g')
xlabel("Duty Cycle (D)")
ylabel("Vout(D)")
title("Conversion ratio  - Buck-Boost converter")

subplot(2,2,4)
plot(D, Eff_BBs(1,:), 'b', D, Eff_BBs(2,:), 'r', D, Eff_BBs(3,:), 'g')
axis([Dmin Dmax 0 1])
xlabel("Duty Cycle (D)")
ylabel("n(D)")
title("Efficiency - Buck-Boost converter")