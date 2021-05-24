% ------------------------------------------------------------------------------
% Copyright (C) 2021 Jaime M. Villegas I. <jaime7592@gmail.com>
% ------------------------------------------------------------------------------
% Filename      : Sallen_Key_read.m
% Description   : Results analysis of 2nd Order Sallen-Key Filter simulation 
% Version       : 01.00
% Revision      : 00
% Last modified : 05/24/2021
% ------------------------------------------------------------------------------
clear
clc

% Simulation parameters
sim_num_steps = 5000;
sim_step_time = 100e-6;

% Time axis
t = 0 : sim_step_time : (sim_num_steps - 1)*sim_step_time;

% Opens file
fp = fopen("sim_dat.dat", "r");

% Reads stored variable
Vc1 = fread(fp, sim_num_steps, "double");

% Closes file
fclose(fp);

% Plots results
plot(t, Vc1', "linewidth", 1);
axis([ 0, 0.5, 0, 1.05*max(Vc1) ])
xlabel('Time (s)', 'FontSize', 14)
ylabel('Voltage (V)', 'FontSize', 14)
title('Sallen - Key capacitor voltage (Open-loop)', 'FontSize', 16)

% Ziegler-Nichols Method
Tcr = 0.4992 - 0.4914;
Kcr = 13.1;
td = 0.125 * Tcr;
ti = 0.5 * Tcr;
Kp = 0.6 * Kcr;
Ki = Kp / ti;
Kd = Kp * td;