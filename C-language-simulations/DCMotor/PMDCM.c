/* -----------------------------------------------------------------------------
 * Copyright (C) 2021 Jaime M. Villegas I. <jaime7592@gmail.com>
 * -----------------------------------------------------------------------------
 * Filename      : PMDCM.c
 * Description   : Permanent Magnet DC Motor simulation. Closed-loop analysis
 *                 with cascade current and angular speed controller.
 * Version       : 01.00
 * Revision      : 00
 * Last modified : 06/04/2021
 * -----------------------------------------------------------------------------
 */

//----------------------------------------------------------------------------//
//                                Header files                                //
//----------------------------------------------------------------------------//

#include<stdio.h>
#include<stdlib.h>
#include<stdbool.h>
#include<math.h>

//----------------------------------------------------------------------------//
//                            General definitions                             //
//----------------------------------------------------------------------------//

// Simulation parameters
#define SIM_NUM_STEPS     100000
#define SIM_STEP_TIME     1e-3

// PMDC Motor parameters
const double Ra = 1.45;
const double La = 5.4e3;
const double Km = 0.5791;
const double Bm = 0.0023;
const double Jm = 2.189e-4;

// Speed reference (rad/s)
const double wref = 160;

// Load torque perturbation
const double T_LOAD = 0.0;
double TL = 0;
double t_stepTL = (SIM_NUM_STEPS / 2) * SIM_STEP_TIME;

// Input voltage
double Va = 0;  
            
// PMDC Motor State Variables
typedef enum
{
  /*0*/ PMDCM_IA = 0,    // Angular speed
  /*1*/ PMDCM_W,         // Armature current
  /*2*/ PMDCM_VAR_NUM    // Total number of varibles
}
PMDCM_VAR;

// State-space constants
double k_i1 = 0;
double k_i2 = 0;
double k_i3 = 0;
double k_w1 = 0;
double k_w2 = 0;
double k_w3 = 0;

// Initial conditions
double x1_0 = 0;       // Initial armature current
double x2_0 = 0;       // Initial angular speed

// PI Controllers

// Margin phase
double phi_MP = 0.8727;

// 1. Speed controller
double Kp_w = 0.0;
double Ki_w = 0.0;
const double wc_GcW = 100;

// 2. Current controller
double Kp_ia = 0.0;
double Ki_ia = 0.0;
const double wc_GcIa = 1000;

//----------------------------------------------------------------------------//
//                      Private functions prototypes                          //
//----------------------------------------------------------------------------//

/**
@brief  Initializes state-equations constants and parameters
@param  None
@retval None
*/
void PMDCM_init();

/**
@brief  ODE solver using Euler method
@param  y:  Output state vector
        t0: Initial instant of time
        t1: Next instant of time
        x:  Input state vector
        dx: Derivative of state vector
@retval None
*/
void ODE_solve_Euler(double* y, double t0, double t1, double* x);

/**
@brief  ODE solver using Runge-Kutta 45 method
@param  y:  Output state vector
        t0: Initial instant of time
        t1: Next instant of time
        x:  Input state vector
        dx: Derivative of state vector
@retval None
*/
void ODE_solve_RK45(double* y, double t0, double t1, double* x);

/**
@brief  Updates state-equations values
@param  sX:    Derivative of state vector
         X:    State vector
        Va:    Armature voltage
        Tload: Load torque
@retval None
*/
void PMDCM_state_eq(double* sX, double* X, double Va, double Tload);

/**
@brief  PI speed controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Speed_Controller(double err, double t0, double t1);

/**
@brief  PI current controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Current_Controller(double err, double t0, double t1);

//----------------------------------------------------------------------------//
//                               Main function                                //
//----------------------------------------------------------------------------//

int main()
{
  double data[SIM_NUM_STEPS][PMDCM_VAR_NUM];   // Data matrix
  unsigned int i = 0;                          // Iterator
  
  double x[PMDCM_VAR_NUM]  = {x1_0, x2_0};      // State vector
  double dx[PMDCM_VAR_NUM] = {0};               // Derivative of state vector
  double y[PMDCM_VAR_NUM]  = {0};               // Output vector
  
  FILE* fp = NULL;                             // File pointer
  
  /************************* Initialization cycle *****************************/
  
  // Initialize plant constants
  PMDCM_init();
  
  /*************************** Simulation cycle *******************************/
  
  for(i = 0; i < SIM_NUM_STEPS; i++)
  {
    ODE_solve_Euler(y, (double)(i * SIM_STEP_TIME),
                    (double)( (i + 1)*SIM_STEP_TIME ), x);
    
    // Updates state vector              
    x[PMDCM_IA] = y[PMDCM_IA];
    x[PMDCM_W]  = y[PMDCM_W];
  
    // Adds state variables to data output matrix
    data[i][PMDCM_IA] = y[PMDCM_IA];
    data[i][PMDCM_W]  = y[PMDCM_W];
  }
  
  /******************************* End cycle **********************************/
  
  // Output data file opening
  fp = fopen("sim_dat.txt", "w+t");
  
  if(fp == NULL)
  {
    printf("ERROR IN OPENING SIMULATION FILE OPERATION");
    exit(-1);
  }
  
  // Writes data output in file
  for(i = 0; i < SIM_NUM_STEPS; i++)
  {
    fprintf(fp, "%f %f %f\n", data[i][PMDCM_IA], data[i][PMDCM_W], Va);
  }
  
  // Closes file
  fclose(fp);
  
  return 0;
}

/**
@brief  Initializes state-equations constants and parameters
@param  None
@retval None
*/
void PMDCM_init()
{ 
  // State-space constants
  k_i1 = - Ra/La;
  k_i2 = - Km/La;
  k_i3 = 1/La;
  k_w1 = Km/Jm;
  k_w2 = - Bm/Jm;
  k_w3 = - 1/Jm;
  
  // Controller constants
  
  // 1. Current controller
  Kp_ia = (La/Ra) * (wc_GcIa * Ra);
  Ki_ia = wc_GcIa * Ra;

  // 2. Speed controller
  Ki_w = (Jm/Km) * exp( 2*log10(wc_GcW) 
                 - log( sqrt( pow(wc_GcW, 2)/pow(wc_GcW/tan(phi_MP), 2) + 1) ) );
                 
  Kp_w = Ki_w * tan(phi_MP) / wc_GcW;
}

/**
@brief  ODE solver using Euler method
@param  y:  Output state vector
        t0: Initial instant of time
        t1: Next instant of time
        x:  Input state vector
        dx: Derivative of state vector
@retval None
*/
void ODE_solve_Euler(double* y, double t0, double t1, double* x)
{
  double dx[PMDCM_VAR_NUM];   // Derivative of x
  double iaref = 0;           // Current reference
  double err_w = 0;           // Speed error signal
  double err_ia = 0;          // Current error signal
  
  // Computes input signal
  err_w = wref - y[PMDCM_W];
  iaref = PI_Speed_Controller(err_w, t0, t1);
  err_ia = iaref - y[PMDCM_IA];
  Va = PI_Current_Controller(err_ia, t0, t1);
  
  // Applies load torque perturbation
  if(t0 == t_stepTL)
  {
    TL = T_LOAD;
  }
  
  // Updates state variables
  PMDCM_state_eq(dx, x, Va, TL);
  
  // Calculates step solution
  y[PMDCM_IA] = x[PMDCM_IA] + dx[PMDCM_IA] * (t1 - t0);
  y[PMDCM_W]  = x[PMDCM_W]  + dx[PMDCM_W]  * (t1 - t0);
}

/**
@brief  ODE solver using Runge-Kutta 45 method
@param  y:  Output state vector
        t0: Initial instant of time
        t1: Next instant of time
        x:  Input state vector
        dx: Derivative of state vector
@retval None
*/
void ODE_solve_RK45(double* y, double t0, double t1, double* x)
{
  double iaref = 0;           // Current reference
  double err_w = 0;           // Speed error signal
  double err_ia = 0;          // Current error signal
  
  // Auxiliary variables for Runge - Kutta method
  double deltaT = t1 - t0;
  
  double k1[PMDCM_VAR_NUM] = {0};
  double k2[PMDCM_VAR_NUM] = {0};
  double k3[PMDCM_VAR_NUM] = {0};
  double k4[PMDCM_VAR_NUM] = {0};
  
  double xt[PMDCM_VAR_NUM] = {0};
  
  // Computes input signal
  err_w = wref - y[PMDCM_W];
  iaref = PI_Speed_Controller(err_w, t0, t1);
  err_ia = iaref - y[PMDCM_IA];
  Va = PI_Current_Controller(err_ia, t0, t1);
  
  // Applies load torque perturbation
  if(t0 == t_stepTL)
  {
    TL = T_LOAD;
  }
  
  PMDCM_state_eq(k1, x, Va, TL);

  xt[PMDCM_IA] = x[PMDCM_IA] + k1[PMDCM_IA] * deltaT / 2.0;
  xt[PMDCM_W]  = x[PMDCM_W]  + k1[PMDCM_W]  * deltaT / 2.0;
  
  PMDCM_state_eq(k2, xt, Va, TL);
  
  xt[PMDCM_IA] = x[PMDCM_IA] + k2[PMDCM_IA] * deltaT / 2.0;
  xt[PMDCM_W]  = x[PMDCM_W]  + k2[PMDCM_W]  * deltaT / 2.0;
  
  PMDCM_state_eq(k3, xt, Va, TL);
  
  xt[PMDCM_IA] = x[PMDCM_IA] + k3[PMDCM_IA] * deltaT;
  xt[PMDCM_W]  = x[PMDCM_W]  + k3[PMDCM_W]  * deltaT;
  
  PMDCM_state_eq(k4, xt, Va, TL);
  
  // Calculates step solution
  y[PMDCM_IA] = x[PMDCM_IA] + (k1[PMDCM_IA] 
                               + k2[PMDCM_IA] * 2 
                               + k3[PMDCM_IA] * 2 
                               + k4[PMDCM_IA] ) 
                               * deltaT / 6;
  
  y[PMDCM_W] = x[PMDCM_W] + (k1[PMDCM_W] 
                               + k2[PMDCM_W] * 2 
                               + k3[PMDCM_W] * 2 
                               + k4[PMDCM_W] ) 
                               * deltaT / 6;
}

/**
@brief  Updates state-equations values
@param  sX:    Derivative of state vector
         X:    State vector
        Va:    Armature voltage
        Tload: Load torque
@retval None
*/
void PMDCM_state_eq(double* sX, double* X, double Va, double Tload)
{ 
  // sX1
  sX[PMDCM_IA] = k_i1 * X[PMDCM_IA] + k_i2 * X[PMDCM_W] + k_i3 * Va;
  
  // sX2
  sX[PMDCM_W] = k_w1 * X[PMDCM_IA] + k_w2 * X[PMDCM_W] + k_w3 * Tload;
}

/**
@brief  PI speed controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Speed_Controller(double err, double t0, double t1)
{
  static double sumError = 0;    // Cumulated error (integrative term)
  
  // Integrative error
  sumError += err * (t1 - t0);
  
  return Kp_w*err + Ki_w*sumError;
}

/**
@brief  PI current controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Current_Controller(double err, double t0, double t1)
{
  static double sumError = 0;    // Cumulated error (integrative term)
  
  // Integrative error
  sumError += err * (t1 - t0);
  
  return Kp_ia*err + Ki_ia*sumError;
}

