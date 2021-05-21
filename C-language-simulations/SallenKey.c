/* -----------------------------------------------------------------------------
 * Copyright (C) 2021 Jaime M. Villegas I. <jaime7592@gmail.com>
 * -----------------------------------------------------------------------------
 * Filename      : SallenKey.c
 * Description   : 2nd Order Sallen-Key Filter simulation using Euler's Method.
 * Version       : 01.00
 * Revision      : 00
 * Last modified : 05/20/2021
 * -----------------------------------------------------------------------------
 */

//----------------------------------------------------------------------------//
//                                Header files                                //
//----------------------------------------------------------------------------//

#include<stdio.h>
#include<stdlib.h>

//----------------------------------------------------------------------------//
//                            General definitions                             //
//----------------------------------------------------------------------------//

// Simulation parameters
#define SIM_NUM_STEPS     5000
#define SIM_STEP_TIME     100e-6

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
const double Vref = 10;

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

//----------------------------------------------------------------------------//
//                      Private functions prototypes                          //
//----------------------------------------------------------------------------//

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
@brief  Initializes state-equations constants and parameters
@param  None
@retval None
*/
void Sallen_Key_init();

/**
@brief  Updates state-equations values
@param  sX: Derivative of state vector
         X: State vector
@retval None
*/
void Sallen_Key_state_eq(double* sX, double* X);

//----------------------------------------------------------------------------//
//                               Main function                                //
//----------------------------------------------------------------------------//

int main()
{
  double data[SIM_NUM_STEPS];
  unsigned int i = 0;
  
  double x[SK_VAR_NUM] = {x1_0, x2_0};
  double y[SK_VAR_NUM] = {0};
  
  /*
  FILE* fp = NULL;
  fp = fopen("sim_data.dat", "w+");
  
  
  if(fp = NULL)
  {
    printf("ERROR IN OPENING SIMULATION FILE OPERATION");
    exit(-1);
  }
  */
  Sallen_Key_init();
  
  for(i = 0; i < SIM_NUM_STEPS; i++)
  {
    ODE_solve_Euler(y, (double)(i*SIM_STEP_TIME), (double)( (i + 1)*SIM_STEP_TIME ), x);
    x[0] = y[0];
    x[1] = y[1];
    
    //printf("%lf - %lf\n", y[0], y[1]);
  }
  
  return 0;
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
  double dev[2];
  
  Sallen_Key_state_eq(dev, x);
  
  y[0] = x[0] + dev[0]*(t1-t0);
  y[1] = x[1] + dev[1]*(t1-t0);
}

/**
@brief  Initializes state-equations constants and parameters
@param  None
@retval None
*/
void Sallen_Key_init()
{
  // Setting up OPAMP gain
  double G = 1 + R4/R3;
  
  // State-space constants
  k1 = ( -1 / (R1*R2*C1*C2) );
  k2 = ( (R1 + R2)*C1 + (1 - G)*R1*C2 ) * k1;
}

/**
@brief  Updates state-equations values
@param  sX: Derivative of state vector
         X: State vector
@retval None
*/
void Sallen_Key_state_eq(double* sX, double* X)
{
  // sX1
  sX[SK_VC1] = X[SK_X];
  
  // sX2
  sX[SK_X] = k1*X[SK_VC1] + k2*X[SK_X] - k1*Vref;
}

