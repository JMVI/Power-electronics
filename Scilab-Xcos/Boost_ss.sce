// -----------------------------------------------------------------------------
// Copyright (C) 2020 Jaime M. Villegas I. [jaime7592@gmail.com]
// -----------------------------------------------------------------------------
// Filename      : Boost_ss.m
// Description   : Small-signal analysis of non-ideal Boost Converter. Modeling,
//                 open-loop and closed loop analysis (voltage control)
// Version       : 01.00
// Revision      : 00
// Last modified : 07/08/2020
// References    : R.W. Erickson, D. Maksimovic. "Fundamentals of Power
//                 Electronics". 2nd ed. (2004)
// -----------------------------------------------------------------------------
clear
clc

/*************************** Converter parameters ****************************/

// Converter quiescent parameters
Vg = 48;               // Input voltage
Rload = 12;            // Load resistance
D = 0.6;               // Duty cycle
V = Vg/(1-D);          // Output voltage
I = V/(Rload*(1-D));   // Output current


// Diode
Rd = 10e-3;  // Diode on-state resistance
Vd = 0.7;    // Diode forward voltage

// Transistor
Rt = 10e-3;    // Transistor on-state resistance

// Inductor
L = 100e-6;    // Inductor value
RL = 0.05;     // Inductor resistance

// Capacitor
Cout = 33e-6   // Capacitor value

// *************************** Simulation parameters *************************/

// 1. Simulation time
tsim = 10e-3;
tstep = 100e-9;

// 2. Line voltage and load current perturbations
Vg_stp = 50;
tstp_Vg = 0.8*tsim;

Iload_stp = 10;
tstp_Iload = 0.5*tsim;

/***************************** Converter model *******************************/

// State-space model
A = [-(RL+D*Rt+(1-D)*Rd)/L, -1/L; 1/Cout, -1/(Rload*Cout)];
B = [D/L, (Vg+Rd*I+Vd-Rt*I)/L, 0; 0, 0, 1];
C = [1, 0; 0, 1];

// Polynomial variable
s = poly(0,"s");
Y = C*inv(s*eye(2,2) - A)*B;

// Transfer functions
Gid = syslin('c', Y(1,2));    // iL(s)/d(s)
Gvg = syslin('c', Y(2,1));    // v(s)/vg(s)
Gvd = syslin('c', Y(2,2));    // v(s)/d(s)

// Output impedance
re = RL+D*Rt+(1-D)*Rd                             // Equivalent loss resistance
Zout = ( 1/( (s*L + re)/(1-D)^2 ) + s*Cout + 1/Rload  )^(-1);

/************************** Closed loop analysis ******************************/

// Closed-loop parameters
Vm = 4;       // PWM modulator gain
Vref = 5;     // Reference
H = Vref/V;   // Sensor gain

// Uncompensated loop
T_u = H/Vm                                  // Loop gain

// 1. Gvd(s)
Tvd_u = syslin('c', T_u*Y(2,2));            // Uncompensated Gvd(s) t. function
Ncoeff_Tvd_u = coeff(Tvd_u.num);            // Numerator coefficients
Dcoeff_Tvd_u = coeff(Tvd_u.den);            // Denominator coefficients
Tvu0 = Ncoeff_Tvd_u(1)/Dcoeff_Tvd_u(1);     // Low frequency gain
w0 = sqrt(Dcoeff_Tvd_u(1));                 // Resonant frequency
f0 = w0/(2*%pi);

// 2. Gvg(s)
Tvg_u = syslin('c', Y(2,1)/(1 + T_u));      // Uncompensated Gvg(s) t. function

// 3. Zout(s)
Zout_u = syslin('c', Zout/(1 + T_u));       // // Uncompensated output impedance

// Uncompensated phase margin and crossover frequency
[phi_u, fc_u] = p_margin(Tvd_u);

// Desired phase margin and crossover frequency
phi = 50;
fc = 20000;

// Compensator design (PID)
fz = fc*sqrt( (1 - sind(phi) )/( 1 + sind(phi) ) );
fp = fc*sqrt( (1 + sind(phi) )/( 1 - sind(phi) ) );
fL = fc/10;
Gc0 = (fc/f0)^2 * 1/Tvu0 * sqrt(fz/fp);

wz = 2*%pi*fz;
wp = 2*%pi*fp;
wL = 2*%pi*fL;

Gc = Gc0*(1+wL/s)*(1+s/wz)/(1+s/wp);

// Compensated loop gain
T_c = Gc*H/Vm                           // Compensator
Tvd_c = syslin('c', T_c*Y(2,2));        // Compensated Gvd(s) transfer function
Tvg_c = syslin('c', Y(2,1)/(1 + T_c));  // Compensated Gvg(s) transfer function
Zout_c = syslin('c', Zout/(1 + T_c));

// Compensated phase margin and crossover frequency
[phi_c, fc_c] = p_margin(Tvd_c);

// Bode plots

// Analyzed transfer function
/*
 *  c = 1: Gvd(s)   - Control-to-Output
 *  c = 2: Gvg(s)   - Line-to-Output
 *  c = 3: Zout(s)  - Output impedance
 */ 
c = 2;

select c
  case 1
    clf(); bode([Tvd_u; Tvd_c], 0.01, 100000, ['Gvd_u(s)'; 'Gvd_c(s)']);
  case 2
    clf(); bode([Tvg_u; Tvg_c], 0.01, 100000, ['Gvg_u(s)'; 'Gvg_c(s)']);
  case 3
    clf(); bode([Zout_u; Zout_c], 0.01, 100000, ['Zout_u(s)'; 'Zout_c(s)']);
end

// ***************************** Simulation results ***************************/

// 1. Output voltage
subplot(2,1,1)
plot(vout_sim.time, vout_sim.values)
xlabel("Time (s)", "fontsize", 2)
ylabel("Voltage (V)", "fontsize", 2)
title("Boost converter - Output voltage", "fontsize", 2.5)

// 2. Output current
subplot(2,1,2)
plot(Iload_sim.time, Iload_sim.values, 'b', Iload_sim.time, vout_sim.values / Rload, 'r')
xlabel("Time (s)", "fontsize", 2)
ylabel("Current (A)", "fontsize", 2)
title("Boost converter - Output current", "fontsize", 2.5)
legend(["Iload perturbation"; "Iload"], "in_lower_right")



