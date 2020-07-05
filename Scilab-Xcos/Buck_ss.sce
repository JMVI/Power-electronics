// -----------------------------------------------------------------------------
// Copyright (C) 2020 Jaime M. Villegas I. [jaime7592@gmail.com]
// -----------------------------------------------------------------------------
// Filename      : Buck_ss.m
// Description   : Small-signal analysis of non-ideal Buck Converter. Modeling,
//                 open-loop and closed loop analysis
// Version       : 01.00
// Revision      : 00
// Last modified : 07/02/2020
// References    : R.W. Erickson, D. Maksimovic. "Fundamentals of Power
//                 Electronics". 2nd ed. (2004)
// -----------------------------------------------------------------------------
clear
clc

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

// State-space model
a1 = [-(RL+D*Rt+(1-D)*Rd)/L, -1/L, 0];
a2 = [1/Cout, -1/(Rload*Cout), 0];
a3 = [-D*(RL+D*Rt+(1-D)*Rd)/L, -D/L, 0];

A = [a1; a2; a3];
B = [D/L, (Vg+Rd*I+Vd-Rt*I)/L; 0, 0; D^2/L, D*(Vg+Rd*I+Vd-Rt*I)/L];
C = [1, 0, 0; 0, 1, 0; 0, 0, 1];

// Polynomial variable
s = poly(0,"s");
Y = C*inv(s*eye(3,3) - A)*B;

// Transfer functions
i_d   = syslin('c', Y(1,2));
v_vg  = syslin('c', Y(2,1));
v_d   = syslin('c', Y(2,2));
ig_vg = syslin('c', Y(3,2));

// Closed-loop parameters
Vm = 4;       // PWM modulator gain
Vref = 5;     // Reference
H = Vref/V;   // Sensor gain

// Uncompensated loop gain
Tv_u = syslin('c', H*Y(2,2)/Vm);        // Transfer function
Ncoeff_Tv_u = coeff(Tv_u.num);          // Numerator coefficients
Dcoeff_Tv_u = coeff(Tv_u.den);          // Denominator coefficients
Tvu0 = Ncoeff_Tv_u(1)/Dcoeff_Tv_u(1);   // Low frequency gain
w0 = sqrt(Dcoeff_Tv_u(1));              // Resonant frequency
f0 = w0/(2*%pi);

// Uncompensated phase margin and crossover frequency
[phi_u, fc_u] = p_margin(Tv_u);

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
Tv_c = syslin('c', Gc*H*Y(2,2)/Vm);

// Compensated phase margin and crossover frequency
[phi_c, fc_c] = p_margin(Tv_c);

// Bode plot
clf(); bode(Tv_c, 0.01, 10000);



