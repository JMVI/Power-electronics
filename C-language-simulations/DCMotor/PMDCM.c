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
#define SIM_NUM_STEPS     50000
#define SIM_STEP_TIME     10e-6

// PMDC Motor parameters
const double Ra = 1.45;
const double La = 5.4e3;
const double Km = 0.5791;
const double Bm = 0.023;
const double Jm = 2.189e-4;

// Speed reference (rad/s)
const double wref = 160;

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

// 1. Current controller
double Kp_ia = 0.0;
double Ki_ia = 0.0;
const double wc_GcIa = 1000;

// 2. Speed controller
double Kp_w = 0.0;
double Ki_w = 0.0;
const double wc_GcW = 100;

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
void ODE_solve_Euler(double* y, double t0, double t1, double* x, double* dx);

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
@brief  PI current controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Current_Controller(double err, double t0, double t1);

/**
@brief  PI speed controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Speed_Controller(double err, double t0, double t1);

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
    //PMDCM_state_eq
    
    ODE_solve_Euler(y, 
                    (double)(i * SIM_STEP_TIME),
                    (double)( (i + 1)*SIM_STEP_TIME ), 
                    x,
                    dx);
    
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
    fprintf(fp, "%f %f\n", data[i][PMDCM_IA], data[i][PMDCM_W]);
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
  
  // TODO: Add computation of Gc_w using pole cancelation method
  Kp_w = 0.0;
  Ki_w = 0.0;
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
void ODE_solve_Euler(double* y, double t0, double t1, double* x, double* dx)
{   
  // Calculates step solution
  y[PMDCM_IA] = x[PMDCM_IA] + dx[PMDCM_IA] * (t1 - t0);
  y[PMDCM_W]  = x[PMDCM_W] + dx[PMDCM_W] * (t1 - t0);
}

/**
@brief  Updates state-equations values
@param  sX:    Derivative of state vector
         X:    State vector
        Va:    Armature voltage
        Tload: Load torque
@retval None
*/
void PMDCM_state_eq(double* sX, double* X, double Va, double Tload);
{ 
  // sX1
  sX[PMDCM_IA] = k_i1 * X[PMDCM_IA] + k_i2 * X[PMDCM_W] + k_i3 * Va;
  
  // sX2
  sX[PMDCM_W] = k_w1 * X[PMDCM_IA] + k_w2 * X[PMDCM_W] + k_w3 * Tload;
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
  static double err_1 = 0;       // Last error value
  
  // Integrative error
  sumError += err * (t1 - t0);
  
  // Stores last error value
  err_1 = err;
  
  return Kp_ia*err + Ki_ia*sumError;
}

/**
@brief  PI speed controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PI_Speed_Controller(double err, double t0, double t1);
{
  static double sumError = 0;    // Cumulated error (integrative term)
  static double err_1 = 0;       // Last error value
  
  // Integrative error
  sumError += err * (t1 - t0);
  
  // Stores last error value
  err_1 = err;
  
  return Kp_w*err + Ki_w*sumError;
}


