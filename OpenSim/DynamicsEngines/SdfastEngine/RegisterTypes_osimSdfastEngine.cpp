// RegisterTypes_SdfastEngine.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSdfastEngine.h"
#include "SdfastEngine.h"
#include "SdfastBody.h"
#include "SdfastJoint.h"
#include "SdfastCoordinate.h"
#include "SdfastSpeed.h"

using namespace std;
using namespace OpenSim;

static osimSdfastEngineInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
OSIMSDFASTENGINE_API void RegisterTypes_SdfastEngine()
{
	cout<<"RegisterTypes_SdfastEngine\n";

	Object::RegisterType( SdfastEngine() );
	Object::RegisterType( SdfastBody() );
	Object::RegisterType( SdfastJoint() );
	Object::RegisterType( SdfastCoordinate() );
	Object::RegisterType( SdfastSpeed() );
}

osimSdfastEngineInstantiator::osimSdfastEngineInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSdfastEngineInstantiator::registerDllClasses() 
{ 
        RegisterTypes_SdfastEngine(); 
} 
    
