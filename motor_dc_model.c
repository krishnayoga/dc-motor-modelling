#define S_FUNCTION_LEVEL    2
#define S_FUNCTION_NAME     motor_dc_model
#include "simstruc.h"    
#include <math.h>

#define U(element)    (*uPtrs0[element])

static void mdlInitializeSizes(SimStruct *S) 
{
    ssSetNumContStates(S, 3); //jumlah states dalam sistem       

    if (!ssSetNumInputPorts(S, 1)) //jumlah input port
        return;
    ssSetInputPortWidth(S, 0, 2); //lebar input port
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortOverWritable(S, 0, 1);

    if (!ssSetNumOutputPorts(S, 1)) //jumlah output port
        return;
    ssSetOutputPortWidth(S, 0, 4); //lebar output port

    ssSetNumSampleTimes(S, 1);

    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, CONTINUOUS_SAMPLE_TIME); //sample time 0.0 sekon
    ssSetOffsetTime(S, 0, 0.0);
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
    real_T *X0     = ssGetContStates(S);
    int_T  nStates = ssGetNumContStates(S);
    int_T  i;

    //reset seluruh nilai states agar bernilai 0.0
    for (i = 0; i < nStates; i++)
    {
        X0[i] = 0.0;
    }
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T            *Y     = ssGetOutputPortRealSignal(S, 0);
    real_T            *X     = ssGetContStates(S);
    InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S, 0);

    real_T            teta;      //posisi sudut
    real_T            tetadot;   //kecepatan sudut
    real_T            i;         //arus
    real_T            Kt = 1.28; //konstanta torsi

    i       = X[0];
    tetadot = X[1];
    teta    = X[2];
    Y[0]    = i;
    Y[1]    = tetadot;
    Y[2]    = teta;
    Y[3]    = Kt * i;
}

#define MDL_DERIVATIVES
static void mdlDerivatives(SimStruct *S)
{
    real_T            *dX    = ssGetdX(S);
    real_T            *X     = ssGetContStates(S);
    InputRealPtrsType uPtrs0 = ssGetInputPortRealSignalPtrs(S, 0);

    real_T i, tetadot, teta;            //state variables (x)
    real_T idot, tetadotdot, tetadot2;  //xdot
    real_T J, b, K, R, L;               //konstanta-konstanta
    real_T v, TL;                       //input

    //motor parameter
    J = 0.02215;
    b = 0.002953;
    K = 1.28;
    R = 11.2;
    L = 0.1215;

    v  = U(0); //input W Reference
    TL = U(1); //input Load

    //update state data
    i       = X[0];
    tetadot = X[1];
    teta    = X[2];

    //persamaan motor DC
    idot       = -((R / L) * i) - ((K / L) * tetadot) + ((1 / L) * v);
    tetadotdot = ((K / J) * i) - ((b / J) * tetadot) - ((-1 / J) * TL);
    tetadot2   = tetadot;

    //update state data yang telah diintegralkan
    dX[0] = idot;
    dX[1] = tetadotdot;
    dX[2] = tetadot2;
}


static void mdlTerminate(SimStruct *S)
{
}

#ifdef  MATLAB_MEX_FILE

#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
