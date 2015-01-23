/*

Authors:  Michael Petretti (michael@petretti.de)
          Björn Hagemeister
          @ University of Freiburg, Germany, 2015.


 * sfuntmpl_basic.c: Basic 'C' template for a level 2 S-function.
 *
 * Copyright 1990-2013 The MathWorks, Inc.
 */

/*
 * You must specify the S_FUNCTION_NAME as the name of your S-function
 * (i.e. replace sfuntmpl_basic with the name of your S-function).
 */

#define S_FUNCTION_NAME  steuerung
#define S_FUNCTION_LEVEL 2

/*
 * Need to include simstruc.h for the definition of the SimStruct and
 * its associated macro definitions.
 */
#include "simstruc.h"
#include "mex.h"
#include "../AutomaticShiftingMatlab/usedMethods.h"



static void mdlInitializeSizes(SimStruct *S) {
  // ssSetNumDiscStates(S, 1);
  ssSetNumSFcnParams(S, 0);
  if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
    return; /* Parameter mismatch will be reported by Simulink */
  }
  if (!ssSetNumInputPorts(S, 1)) return;
  ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
  ssSetInputPortDirectFeedThrough(S, 0, 1);
  if (!ssSetNumOutputPorts(S,1)) return;
  ssSetOutputPortWidth(S, 0, DYNAMICALLY_SIZED);
  ssSetNumSampleTimes(S, 1);
  /* Take care when specifying exception free code - see sfuntmpl.doc */
  ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
  
  initMethod();
}

static void mdlInitializeSampleTimes(SimStruct *S) {
  ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
  ssSetOffsetTime(S, 0, 0.0);
}

static void mdlOutputs(SimStruct *S, int_T tid) {
  InputRealPtrsType uZwi = ssGetInputPortRealSignalPtrs(S,0);

  y = ssGetOutputPortRealSignal(S, 0);      // output
  u = uZwi[0];   							// input
  // stepLength = u[4];
  mainIt();
 // for (int i = 0; i < 6; i++) {
//	y[i] = u[i];
	//}
}

static void mdlUpdate(SimStruct *S, int_T tid)
{

    y = ssGetOutputPortRealSignal(S,0);
    // int_T width = ssGetOutputPortWidth(S,0);
    u = (const real_T*) ssGetInputPortSignal(S,0);
    x = ssGetRealDiscStates(S);
    //InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S,0);

}

static void mdlTerminate(SimStruct *S){}

#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include "simulink.c" /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif 