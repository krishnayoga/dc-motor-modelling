#define S_FUNCTION_LEVEL    2
#define S_FUNCTION_NAME     pi_control
#include "simstruc.h"
#include <math.h>

#define U(element)    (*uPtrs[element]) /*Pointer to Input Port0*/

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumDiscStates(S, 10); //jumlah states dalam sistem
    if (!ssSetNumInputPorts(S, 1)) //jumlah input port
        return;
    ssSetInputPortWidth(S, 0, 1); //lebar input port
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);
    if (!ssSetNumOutputPorts(S, 1)) //jumlah output port
        return;
    ssSetOutputPortWidth(S, 0, 1); //lebar output port
    ssSetNumSampleTimes(S, 1);
    ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE
                     | SS_OPTION_DISCRETE_VALUED_OUTPUT));
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, 0.001); //sample time 0.001 sekon 
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T            *X0      = ssGetRealDiscStates(S); 
    int_T             nXStates = ssGetNumDiscStates(S); 
    InputRealPtrsType uPtrs    = ssGetInputPortRealSignalPtrs(S, 0); 
    int_T             i; 

    //reset seluruh nilai states agar bernilai 0.0
    for (i = 0; i < nXStates; i++) 
    {
        X0[i] = 0.0;
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T *Y = ssGetOutputPortRealSignal(S, 0); 
    real_T *X = ssGetRealDiscStates(S); 
    real_T u_k; 

    u_k  = X[2]; 
    Y[0] = u_k;
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    real_T            *X    = ssGetRealDiscStates(S); 
    InputRealPtrsType uPtrs = ssGetInputPortRealSignalPtrs(S, 0);

    real_T            dt = 0.0001;
    real_T            r_k, w_k, teta_k, u_k, y_k, e_k, e_k_prv, xe_k_new, xe_k_prv, d_ek;
    real_T            h, kp, ki;

    //nilai PI controller
    kp = 0.0002; 
    ki = 2.5;
    
    e_k_prv  = X[0]; //error sebelumnya
    xe_k_prv = X[1]; //integral error sebelumnya

    e_k      = U(0); //error
    xe_k_new = xe_k_prv + dt * (e_k + e_k_prv) / 2;  //integral error
    u_k      = kp * e_k + ki * xe_k_new; //sinyal kendali

    X[0] = e_k;
    X[1] = xe_k_new;
    X[2] = u_k;
}

static void mdlTerminate(SimStruct *S)
{
}   /*Keep this function empty since no memory is allocated*/

#ifdef MATLAB_MEX_FILE
/* Is this file being compiled as a MEX-file? */
#include "simulink.c" /*MEX-file interface mechanism*/
#else
#include "cg_sfun.h"  /*Code generation registration function*/
#endif
