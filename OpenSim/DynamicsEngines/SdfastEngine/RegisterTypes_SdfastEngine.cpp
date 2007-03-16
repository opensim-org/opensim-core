// RegisterTypes_SdfastEngine.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/Object.h>
#include "RegisterTypes_SdfastEngine.h"
#include "SdfastEngine.h"
#include "SdfastBody.h"
#include "SdfastJoint.h"
#include "SdfastCoordinate.h"
#include "SdfastSpeed.h"

using namespace std;
using namespace OpenSim;

static SdfastEngineInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
SDFAST_ENGINE_API void RegisterTypes_SdfastEngine()
{
	cout<<"RegisterTypes_SdfastEngine\n";

	Object::RegisterType( SdfastEngine() );
	Object::RegisterType( SdfastBody() );
	Object::RegisterType( SdfastJoint() );
	Object::RegisterType( SdfastCoordinate() );
	Object::RegisterType( SdfastSpeed() );
}

SdfastEngineInstantiator::SdfastEngineInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void SdfastEngineInstantiator::registerDllClasses() 
{ 
        RegisterTypes_SdfastEngine(); 
} 
    
