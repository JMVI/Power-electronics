// -----------------------------------------------------------------------------
// Copyright (C) 2020 Jaime M. Villegas I. [jaime7592@gmail.com]
// -----------------------------------------------------------------------------
// Filename      : Buck_ss.m
// Description   : Small-signal analysis of non-ideal Buck Converter. Modeling,
//                 open-loop and closed loop analysis (voltage control)
// Version       : 01.00
// Revision      : 00
// Last modified : 07/05/2020
// References    : R.W. Erickson, D. Maksimovic. "Fundamentals of Power
//                 Electronics". 2nd ed. (2004)
// -----------------------------------------------------------------------------
clear
clc

/*************************** Converter parameters ****************************/

// Converter quiescent parameters
Vg = 28;            // Input voltage
Rload = 3;          // Load resistance
D = 0.536;          // Duty cycle
I = Vg*D/Rload;     // Output current
V = D*Vg;           // Output voltage

// Diode
Rd = 10e-3;  // Diode on-state resistance
Vd = 0.4;    // Diode forward voltage

// Transistor
Rt = 20e-3;    // Transistor on-state resistance

// Inductor
L = 50e-6;    // Inductor value
RL = 0.01;    // Inductor resistance

// Capacitor
Cout = 500e-6 // Capacitor value

// *************************** Simulation parameters *************************/

// 1. Simulation time
tsim = 10e-3;
tstep = 100e-9;

// 2. Line voltage and load current perturbations
Vg_stp = 30;
tstp_Vg = 0.8*tsim;

Iload_stp = 5;
tstp_Iload = 0.5*tsim;

/***************************** Converter model *******************************/

// State-space model
a1 = [-(RL+D*Rt+(1-D)*Rd)/L, -1/L, 0];
a2 = [1/Cout, -1/(Rload*Cout), 0];
a3 = [-D*(RL+D*Rt+(1-D)*Rd)/L, -D/L, 0];

A = [a1; a2; a3];
B = [D/L, (Vg+Rd*I+Vd-Rt*I)/L, 0; 0, 0, 1; D^2/L, D*(Vg+Rd*I+Vd-Rt*I)/L, 1];
C = [1, 0, 0; 0, 1, 0; 0, 0, 1];

// Polynomial variable
s = poly(0,"s");
Y = C*inv(s*eye(3,3) - A)*B;

// Transfer functions
Gid = syslin('c', Y(1,2));    // iL(s)/d(s)
Gvg = syslin('c', Y(2,1));    // v(s)/vg(s)
Gvd = syslin('c', Y(2,2));    // v(s)/d(s)
Gig = syslin('c', Y(3,2));    // iL(s)/vg(s)

// Output impedance
Zout = ( 1/(s*L) + s*Cout + 1/Rload  )^(-1);

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
phi = 52;
fc = 5000;

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
c = 3;

select c
  case 1
    clf(); bode([Tvd_u; Tvd_c], 0.01, 10000, ['Gvd_u(s)'; 'Gvd_c(s)']);
  case 2
    clf(); bode([Tvg_u; Tvg_c], 0.01, 10000, ['Gvg_u(s)'; 'Gvg_c(s)']);
  case 3
    clf(); bode([Zout_u; Zout_c], 0.01, 10000, ['Zout_u(s)'; 'Zout_c(s)']);
end

// ***************************** Simulation results ***************************/

// 1. Output voltage
subplot(2,1,1)
plot(vout_sim.time, vout_sim.values)
xlabel("Time (s)", "fontsize", 2)
ylabel("Voltage (V)", "fontsize", 2)
title("Buck converter - Output voltage", "fontsize", 2.5)

// 2. Output current
subplot(2,1,2)
plot(Iload_sim.time, Iload_sim.values, 'b', Iload_sim.time, vout_sim.values / Rload, 'r')
xlabel("Time (s)", "fontsize", 2)
ylabel("Current (A)", "fontsize", 2)
title("Buck converter - Output current", "fontsize", 2.5)
legend(["Iload perturbation"; "Iload"], "in_lower_right")



