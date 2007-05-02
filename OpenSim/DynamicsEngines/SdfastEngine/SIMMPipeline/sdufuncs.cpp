// sdufuncs.cpp
//
// This file contains functions called by SD/Fast whose only role is to call back to the
// SdfastEngine.  A back-pointer to the engine that has linked to these functions is
// kept here.  Note that this assumes that the SD/Fast model compiled using this sdfast.cpp
// file is only linked to one engine.

#include <iostream>
#include <OpenSim/DynamicsEngines/SdfastEngine/SdfastEngine.h>

#ifndef WIN32
	#define SDFAST_DLL_API
#else
	#define SDFAST_DLL_API __declspec(dllexport)
#endif

using namespace OpenSim;

// This is a back-pointer to the engine that has linked to these sdfast functions.  It can be used for callbacks.
SdfastEngine *engine = NULL;

extern "C" {

SDFAST_DLL_API void setSdfastEngineInstance(SdfastEngine *aEngine)
{
	if(engine) std::cerr << "WARNING!!! Attempting to link the same sdfast model dll to multiple SdfastEngines" << std::endl;
	engine = aEngine;
}	

int sduforce(double t, double q[], double u[]) { return engine->sduforce(t, q, u); }
int sdumotion(double t, double q[], double u[]) { return engine->sdumotion(t, q, u); }
void sduperr(double t, double q[], double errors[]) { engine->sduperr(t, q, errors); } 
void sduverr(double t, double q[], double u[], double errors[]) { engine->sduverr(t, q, u, errors); }
void sduaerr(double t, double q[], double u[], double udot[], double errors[]) { engine->sduaerr(t, q, u, udot, errors); }
void sduconsfrc(double t, double q[], double u[], double mults[]) { engine->sduconsfrc(t, q, u, mults); }

}
