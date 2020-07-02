% ------------------------------------------------------------------------------
% Copyright (C) 2020 Jaime M. Villegas I. <jaime7592@gmail.com>
% ------------------------------------------------------------------------------
% Filename      : Boost.m
% Description   : DC analysis of non-ideal Boost Converter
% Version       : 01.00
% Revision      : 00
% Last modified : 06/27/2020
% References    : R.W. Erickson, D. Maksimovic. "Fundamentals of Power
%                 Electronics". 2nd ed. (2004)
% ------------------------------------------------------------------------------
clear
clc

% Converter parameters

Vg = 25;         % Input voltage
V = 40;          % Output voltage
Iout = 5;        % Output current
Rload = V/Iout;  % Load resistance

Dmin = 0.005;
Dmax = 0.995;
D = Dmin:0.001:Dmax;   % Converter duty cycle

% Diode
Rd = 10e-3;  % Diode on-state resistance
Vd = 0.4;    % Diode forward voltage

% Transistor
Rt = 20e-3;  % Transistor on-state resistance

% Inductor
K_RL = [0.01 0.05 0.1];   % Load resistance to inductor resistance ratio
RL = K_RL*Rload;          % Inductor resistance

% Effective conduction losses
for i = 1:3 Re(i,:) = D*Rt + (1-D)*Rd + RL(i);end
Ve = (1-D)*Vd;

% Conversion ratio
M_Bs_ideal = 1./(1-D);
M_Bs_real = Rload*(1-D).*(Vg - Ve)./(Rload*(1-D).^2 + Re);

% Efficiency
Eff_Bs = Rload*(1-D).^2 .*(1 - Ve/Vg)./(Rload*(1-D).^2 + Re);

figure(1)
subplot(2,2,1)
plot(D, Re(1,:), 'b', D, Re(2,:), 'r', D, Re(3,:), 'g')
xlabel("Duty Cycle (D)")
ylabel("Re(D)")
title("Effective resistance - Boost converter")

subplot(2,2,2)
plot(D, Ve)
xlabel("Duty Cycle (D)")
ylabel("Ve(D)")
title("Effective voltage drop - Boost converter")

subplot(2,2,3)
plot(D, M_Bs_ideal, 'k', D, M_Bs_real(1,:), 'b', D, M_Bs_real(2,:), 'r', D, M_Bs_real(3,:), 'g')
xlabel("Duty Cycle (D)")
ylabel("Vout(D)")
title("Conversion ratio  - Boost converter")

subplot(2,2,4)
plot(D, Eff_Bs(1,:), 'b', D, Eff_Bs(2,:), 'r', D, Eff_Bs(3,:), 'g')
axis([Dmin Dmax 0 1])
xlabel("Duty Cycle (D)")
ylabel("n(D)")
title("Efficiency - Boost converter")