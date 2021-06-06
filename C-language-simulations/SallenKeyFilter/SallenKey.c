/* -----------------------------------------------------------------------------
 * Copyright (C) 2021 Jaime M. Villegas I. <jaime7592@gmail.com>
 * -----------------------------------------------------------------------------
 * Filename      : SallenKey.c
 * Description   : 2nd Order Sallen-Key Filter simulation using Euler's Method.
 * Version       : 01.00
 * Revision      : 00
 * Last modified : 05/31/2021
 * -----------------------------------------------------------------------------
 */

//----------------------------------------------------------------------------//
//                                Header files                                //
//----------------------------------------------------------------------------//

#include<stdio.h>
#include<stdlib.h>
#include <stdbool.h>

//----------------------------------------------------------------------------//
//                            General definitions                             //
//----------------------------------------------------------------------------//

// Simulation parameters
#define SIM_NUM_STEPS     50000
#define SIM_STEP_TIME     10e-6

// Activate closed-loop control
#define CLOSED_LOOP_ON    true

/***** Sallen-Key circuit parameters *****/

// Circuit resistances
const double R1 = 10e3;
const double R2 = 10e3;
const double R3 = 30e3;
const double R4 = 51e3;

// Circuit capacitances
const double C1 = 470e-9;
const double C2 = 470e-9;

// Voltage reference
const double Vref = 25;

// Sallen-Key State Variables
typedef enum
{
  /*0*/ SK_VC1 = 0,    // Voltage across capacitor 1
  /*1*/ SK_X,          // Derivative of Voltage across capacitor 1
  /*2*/ SK_VAR_NUM     // Total number of varibles
}
SALLEN_KEY_VAR;

// State-space constants
double k1 = 0;
double k2 = 0;

// Initial conditions
double x1_0 = 0;       // Initial voltage across capacitor 1
double x2_0 = 0;       // Initial derivative of Voltage across capacitor 1

// PID Controller
// double Kp = 7.86, Ki = 2015.4 , Kd = 0.075;
double Kp = 0.15, Ki = 25.0, Kd = 0.0;

//----------------------------------------------------------------------------//
//                      Private functions prototypes                          //
//----------------------------------------------------------------------------//

/**
@brief  Initializes state-equations constants and parameters
@param  None
@retval None
*/
void Sallen_Key_init();

/**
@brief  ODE solver using Euler method
@param  y:  Output state vector
        t0: Initial instant of time
        t1: Next instant of time
        x:  Input state vector
@retval None
*/
void ODE_solve_Euler(double* y, double t0, double t1, double* x);

/**
@brief  Updates state-equations values
@param  sX: Derivative of state vector
         X: State vector
@retval None
*/
void Sallen_Key_state_eq(double* sX, double* X, double In);

/**
@brief  PID controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PID_Controller(double err, double t0, double t1);

//----------------------------------------------------------------------------//
//                               Main function                                //
//----------------------------------------------------------------------------//

int main()
{
  double data[SIM_NUM_STEPS];          // Data vector
  unsigned int i = 0;                  // Iterator
  
  double x[SK_VAR_NUM] = {x1_0, x2_0}; // State vector
  double y[SK_VAR_NUM] = {0};          // Output vector
  
  FILE* fp = NULL;                     // File pointer
  
  /************************* Initialization cycle *****************************/
  
  // Initialize plant constants
  Sallen_Key_init();
  
  /*************************** Simulation cycle *******************************/
  
  for(i = 0; i < SIM_NUM_STEPS; i++)
  {
    ODE_solve_Euler(y, 
                    (double)(i * SIM_STEP_TIME),
                    (double)( (i + 1)*SIM_STEP_TIME ), 
                    x);
    
    // Updates state vector              
    x[SK_VC1] = y[SK_VC1];
    x[SK_X]   = y[SK_X];
    
    // Adds capacitor voltage to data output vector
    data[i] = y[SK_VC1];
  }
  
  /******************************* End cycle **********************************/
  
  // Output data file opening
  fp = fopen("sim_dat.dat", "w+b");
  
  if(fp == NULL)
  {
    printf("ERROR IN OPENING SIMULATION FILE OPERATION");
    exit(-1);
  }
  
  // Writes data output in file
  for(i = 0; i < SIM_NUM_STEPS; i++)
  {
    fwrite(&data[i], sizeof(double), 1, fp);
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
void Sallen_Key_init()
{
  // Setting up OPAMP gain
  double G = 1 + R4 / R3;
  
  // State-space constants
  k1 = ( -1 / (R1 * R2 * C1 * C2) );
  k2 = ( (R1 + R2) * C1 + (1 - G) * R1 * C2 ) * k1;
}

/**
@brief  ODE solver using Euler method
@param  y:  Output state vector
        t0: Initial instant of time
        t1: Next instant of time
        x:  Input state vector
@retval None
*/
void ODE_solve_Euler(double* y, double t0, double t1, double* x)
{
  double dx[SK_VAR_NUM]; // Derivative of x
  double u = 0;          // Input signal
  double err = 0;
  
  if(CLOSED_LOOP_ON == true)
  {
    err = Vref - y[SK_VC1];
    u = PID_Controller(err, t0, t1);
  }
  else
  {
    u = Vref;
  }
  
  // Updates state variables
  Sallen_Key_state_eq(dx, x, u);
  
  // Calculates step solution
  y[SK_VC1] = x[SK_VC1] + dx[SK_VC1] * (t1 - t0);
  y[SK_X]   = x[SK_X] + dx[SK_X] * (t1 - t0);
}

/**
@brief  Updates state-equations values
@param  sX: Derivative of state vector
         X: State vector
@retval None
*/
void Sallen_Key_state_eq(double* sX, double* X, double In)
{
  // sX1
  sX[SK_VC1] = X[SK_X];
  
  // sX2
  sX[SK_X] = k1 * X[SK_VC1] + k2 * X[SK_X] - k1 * In;
}

/**
@brief  PID controller
@param  err: Error signal
         t0: Initial instant of time
         t1: Next instant of time
@retval Control signal
*/
double PID_Controller(double err, double t0, double t1)
{
  static double sumError = 0;    // Cumulated error (integrative term)
  static double diffError = 0;   // Differential error (derivative term)
  static double err_1 = 0;       // Last error value
  
  // Integrative error
  sumError += err * (t1 - t0);
  
  // Derivative error
  diffError = (err - err_1) / (t1 - t0);
  
  // Stores last error value
  err_1 = err;
  
  return Kp*err + Ki*sumError + Kd*diffError;
}

