/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
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
