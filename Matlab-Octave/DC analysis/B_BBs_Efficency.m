% ------------------------------------------------------------------------------
% Copyright (C) 2020 Jaime M. Villegas I. <jaime7592@gmail.com>
% ------------------------------------------------------------------------------
% Filename      : B_BBs_Efficency.m
% Description   : Efficiency comparison between a Buck and a Buck-Boost 
%                 converter using DC analysis.
% Version       : 01.00
% Revision      : 00
% Last modified : 07/02/2020
% References    : R.W. Erickson, D. Maksimovic. "Fundamentals of Power
%                 Electronics". 2nd ed. (2004)
% ------------------------------------------------------------------------------
clear
clc
% Converter parameters

Vg = [500 -500]; % Input voltage (Buck -> Vg = 500 / Buck-Boost -> Vg = -500)
V = 400;         % Output voltage
Iout = 10;       % Output current
Rload = V/Iout;  % Load resistance

D = 0:0.001:1;   % Converter duty cycle

% Diode
Rd = 1e-3;   % Diode on-state resistance
Vd = 1.2;    % Diode forward voltage

% Transistor
Rt = 30e-3;  % Transistor on-state resistance

% Inductor
K_RL = 0.01;     % Load resistance to inductor resistance ratio
RL = K_RL*Rload; % Inductor resistance

% Effective losses
Re = D*Rt + (1-D)*Rd + RL;
Ve = (1-D)*Vd;

% Buck converter
Vout_B = Rload*(D*Vg(1) - Ve)./(Rload + Re);
Eff_B = Rload*(1 - Ve./(Vg(1)*D))./(Re + Rload);

figure(1)
subplot(2,2,1)
plot(D, Re)
xlabel("Duty Cycle (D)")
ylabel("Re(D)")
title("Effective resistance - Buck converter")

subplot(2,2,2)
plot(D, Ve)
xlabel("Duty Cycle (D)")
ylabel("Ve(D)")
title("Effective voltage drop - Buck converter")

subplot(2,2,3)
plot(D, Vout_B)
xlabel("Duty Cycle (D)")
ylabel("Vout(D)")
title("Output voltage - Buck converter")

subplot(2,2,4)
plot(D, Eff_B)
xlabel("Duty Cycle (D)")
ylabel("n(D)")
title("Efficiency - Buck converter")
axis([0 1 0 1])

% Buck-Boost converter
Vout_BBs = -Rload*(Ve./(1-D) + D.*Vg(2)./(1-D))./(Rload + Re./(1-D).^2);
Eff_BBs = Rload*(1 + Ve./(Vg(2)*D))./(Rload + Re./(1-D).^2);

figure(2)
subplot(2,2,1)
plot(D, Re)
xlabel("Duty Cycle (D)")
ylabel("Re(D)")
title("Effective resistance - Buck-Boost converter")

subplot(2,2,2)
plot(D, Ve)
xlabel("Duty Cycle (D)")
ylabel("Ve(D)")
title("Effective voltage drop - Buck-Boost converter")

subplot(2,2,3)
plot(D, Vout_BBs)
xlabel("Duty Cycle (D)")
ylabel("Vout(D)")
title("Output voltage - Buck-Boost converter")

subplot(2,2,4)
plot(D, Eff_BBs)
xlabel("Duty Cycle (D)")
ylabel("n(D)")
title("Efficiency - Buck-Boost converter")
axis([0 1 0 1])

D_B = D(810);
D_BBs = D(455);

disp("Buck converter efficiency for V = 400 V @ 10 A")
disp(Eff_B(810))
disp("Buck-Boost converter efficiency for V = 400 V @ 10 A")
disp(Eff_BBs(455))