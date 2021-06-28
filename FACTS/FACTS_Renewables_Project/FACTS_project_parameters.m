% ------------------------------------------------------------------------------
% Copyright (C) 2021 Jaime M. Villegas I. <21-91460@usb.ve>
% ------------------------------------------------------------------------------
% Filename      : FACTS_project_parameters.m
% Description   : Results analysis of three-phase system with FACTS compensation
% Version       : 01.00
% Revision      : 00
% Last modified : 06/27/2021
% ------------------------------------------------------------------------------
clear
clc

% Three-phase generator parameters

Psc = 6000e6;              % Three-phase short-circuit power [VA]
Vp_rms = 150e3;            % RMS Phase voltage [V]
Vl_rms = sqrt(3)*Vp_rms;   % RMS line voltage (phase-to-phase) [V]
f = 60;                    % Line frequency [Hz]

Lg = (Vl_rms^2 / Psc)*(1/(2*pi*f));    % Internal inductance [H]
X_R = 10;                              % Reactance-to-resistance ratio
Rg = 2*pi*f*Lg/X_R;                    % Internal resistance [ohm]

% Transmission line parameters

R_lgth = 1.7e-5;          % Resistance per length unit (ohm/m)
X_lgth = 3.5e-4;          % Reactance per length unit (ohm/m)
L_tl = 25e3;              % Transmission line length (m)

R_tl = R_lgth*L_tl;       % Line resistance (ohm)
X_tl = X_lgth*L_tl;       % Line reactance (ohm)
L_tl = X_tl/(2*pi*f);     % Line inductance (H)

% Load impedance parameters

Rload = 14;               % Load resistance (ohm)          
Lload = 37.88e-3;         % Load inductance (H)
Xload = 2*pi*f*Lload;     % Load reactance (ohm)

% Generator voltage and current file reading

fp = fopen("TPGenerator.csv", "r");     % Opens file
data = csvread(fp);                     % Reads stored variable
tg = data(:, 1);                        % Time axis
T = tg(end - 1);                        % Integration period
Vg = data(:, 2);                        % Generator voltage
Vg_RMS = sqrt((1/T)*trapz(tg, Vg.^2));  % RMS generator voltage
Ig = data(:, 4);                        % Generator current
Ig_RMS = sqrt((1/T)*trapz(tg, Ig.^2));  % RMS generator current
pg = Vg.*Ig;                            % Instantaneous generator power
PG = (1/T)*trapz(tg, pg)/1e6;           % Average active power of generator (MW)
SG = Vg_RMS*Ig_RMS/1e6;                 % Generator complex power (MVA)
fclose(fp);                             % Closes file

% Load current file reading and load power consumption

fp = fopen("Zload.csv", "r");           % Opens file
data = csvread(fp);                     % Reads stored variable
tload = data(:, 1);                     % Time axis
Iload = data(:, 2);                     % Load current
pload = Rload*Iload.^2;                 % Instantaneous active power consumption
PLOAD = (1/T)*trapz(tload, pload)/1e6;  % Active power consumption (MW)
qload = Xload*Iload.^2;                 % Reactive power consumption
QLOAD = (1/T)*trapz(tload, qload)/1e6;  % Consumed reactive power (MVAR)
SLOAD = sqrt(PLOAD^2 + QLOAD^2);        % Complex power consumption (MVA)
fclose(fp);                             % Closes file

% Wind farm voltage and current file reading

fp = fopen("WindFarm.csv", "r");                % Opens file
data = csvread(fp);                             % Reads stored variable
twind = data(:, 1);                             % Time axis
T = twind(end - 1);                             % Integration period
Vwind = data(:, 2);                             % Wind farm voltage
Vwind_RMS = sqrt((1/T)*trapz(twind, Vwind.^2)); % RMS wind farm voltage
Iwind = data(:, 4);                             % Wind farm current
Iwind_RMS = sqrt((1/T)*trapz(twind, Iwind.^2)); % RMS wind farm current
pwind = Vwind.*Iwind;                           % Instantaneous wind farm power
PWIND = (1/T)*trapz(twind, pwind)/1e6;          % Active power of wind farm (MW)
SWIND = Vwind_RMS*Iwind_RMS/1e6;                % Wind farm complex power (MVA)
fclose(fp);   

% plot(tload(1:length(tload)-1), Pload(1:length(Pload)-1))

