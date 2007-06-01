// RegisterTypes_SimbodyEngine.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimbodyEngine.h"
#include "SimbodyEngine.h"
#include "SimbodyBody.h"
#include "SimbodyJoint.h"
#include "SimbodyCoordinate.h"
#include "SimbodySpeed.h"

using namespace std;
using namespace OpenSim;

static osimSimbodyEngineInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
OSIMSIMBODYENGINE_API void RegisterTypes_SimbodyEngine()
{
	cout<<"RegisterTypes_SimbodyEngine\n";

	Object::RegisterType( SimbodyEngine() );
	Object::RegisterType( SimbodyBody() );
	Object::RegisterType( SimbodyJoint() );
	Object::RegisterType( SimbodyCoordinate() );
	Object::RegisterType( SimbodySpeed() );
}

osimSimbodyEngineInstantiator::osimSimbodyEngineInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimbodyEngineInstantiator::registerDllClasses() 
{ 
        RegisterTypes_SimbodyEngine(); 
} 
    
