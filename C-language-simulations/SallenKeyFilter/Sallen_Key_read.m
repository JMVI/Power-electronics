% ------------------------------------------------------------------------------
% Copyright (C) 2021 Jaime M. Villegas I. <jaime7592@gmail.com>
% ------------------------------------------------------------------------------
% Filename      : Sallen_Key_read.m
% Description   : Results analysis of 2nd Order Sallen-Key Filter simulation 
% Version       : 01.00
% Revision      : 00
% Last modified : 05/31/2021
% ------------------------------------------------------------------------------
clear
clc
pkg load control

% Circuit and state space parameters
R1 = 10e3;
R2 = 10e3;
R3 = 30e3;
R4 = 51e3;

C1 = 470e-9;
C2 = 470e-9;

G = 1 + R4 / R3;

Vref = 25;

k1 = R1*R2*C1*C2;
k2 = (R1 + R2)*C1 + R1*C2*(1 - G);

% PID - Controller
KP = 7.86;
KI = 2015.4;
KD = 0.075;

%% Ziegler-Nichols Method
% Tcr = 0.4992 - 0.4914;
% Kcr = 13.1;
% td = 0.125 * Tcr;
% ti = 0.5 * Tcr;
% Kp = 0.6 * Kcr;
% Ki = Kp / ti;
% Kd = Kp * td;


% Transfer functions
s = tf('s');
Vc1_Vin = 1/ ( k1*s^2 + k2*s + 1);
Gc = pid(KP, KI, KD);
Vc1_Vin_CL = feedback(Vc1_Vin*Gc);

% Activate close loop simulation
Sim_closed_loop = 1;

% Simulation parameters
sim_num_steps = 50000;
sim_step_time = 10e-6;

% Time axis
t = 0 : sim_step_time : (sim_num_steps - 1)*sim_step_time;

% Input signal
Vu = Vref*ones(length(t), 1);

% Run continuous time analysis
if(Sim_closed_loop == 0)
  [y, tout, x] = lsim(Vc1_Vin, Vu, t);
else
  [y, tout, x] = lsim(Vc1_Vin_CL, Vu, t);
end

% Opens file
fp = fopen("sim_dat.dat", "r");

% Reads stored variable
Vc1 = fread(fp, sim_num_steps, "double");

% Closes file
fclose(fp);

% Plot results
figure(1)
plot(t, Vc1', '--r', "linewidth", 2, tout, y, 'b', "linewidth", 1);
axis([ 0, sim_num_steps*sim_step_time, 0, 1.05*max(Vc1) ])
xlabel('Time (s)', 'FontSize', 14)
ylabel('Voltage (V)', 'FontSize', 14)
legend("Euler", "Continuous")

if(Sim_closed_loop == 0)
  title('Sallen - Key capacitor voltage (Open-loop)', 'FontSize', 16)
else
  title('Sallen - Key capacitor voltage (Close-loop)', 'FontSize', 16)
end

% Plot error
figure(2)
semilogy(tout, abs((y - Vc1)./Vc1))
xlabel('Time (s)', 'FontSize', 14)
ylabel('Relative error', 'FontSize', 14)
title('Sallen - Key capacitor voltage simulation error', 'FontSize', 16)
