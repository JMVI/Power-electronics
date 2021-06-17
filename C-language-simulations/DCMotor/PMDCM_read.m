% ------------------------------------------------------------------------------
% Copyright (C) 2021 Jaime M. Villegas I. <jaime7592@gmail.com>
% ------------------------------------------------------------------------------
% Filename      : PMDCM_read.m
% Description   : Results analysis of Permanent Magnet DC Motor simulation. 
%                 Closed-loop analysis with cascade current and angular speed 
%                 controller. 
% Version       : 01.00
% Revision      : 00
% Last modified : 06/17/2021
% ------------------------------------------------------------------------------
clear
clc

% Time axis
sim_num_steps = 100000;
sim_step_time = 1e-3;
t = 0 : sim_step_time : (sim_num_steps - 1)*sim_step_time;

% Opens file
fp = fopen("sim_dat.txt", "r");

% Reads stored variable
data = dlmread(fp);

% Closes file
fclose(fp);

% Speed reference
wref = 160 * ones(1, length(t));

% Plots results
figure(1)
plot(t, data(:,1)', "linewidth", 2);
axis([ 0, sim_num_steps*sim_step_time, 0, 1.5 ])
xlabel('Time (s)', 'FontSize', 14)
ylabel('Current (A)', 'FontSize', 14)
title('Closed-loop armature current', 'FontSize', 16)

figure(2)
plot(t, wref, '--r', "linewidth", 2, t, data(:,2), 'b', "linewidth", 2);
axis([ 0, sim_num_steps*sim_step_time, 0, 170 ])
xlabel('Time (s)', 'FontSize', 14)
ylabel('Angular speed (rad/s)', 'FontSize', 14)
title('Closed-loop angular speed', 'FontSize', 16)

figure(3)
plot(t, data(:,3), 'b', "linewidth", 2)
xlabel('Time (s)', 'FontSize', 14)
ylabel('Voltage (V)', 'FontSize', 14)
title('Applied armature voltage', 'FontSize', 16)
