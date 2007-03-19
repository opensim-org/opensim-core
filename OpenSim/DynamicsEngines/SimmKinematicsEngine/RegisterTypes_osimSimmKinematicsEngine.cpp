// RegisterTypes_osimSimmKinematicsEngine.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimmKinematicsEngine.h"
#include "SimmBody.h"
#include "SimmCoordinate.h"
#include "SimmJoint.h"
#include "SimmKinematicsEngine.h"
#include "SimmRotationDof.h"
#include "SimmTranslationDof.h"

using namespace std;
using namespace OpenSim;

static osimSimmKinematicsEngineInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimSimmKinematicsEngine library.
 */
OSIMSIMMKINEMATICSENGINE_API void RegisterTypes_osimSimmKinematicsEngine()
{
	//cout<<"RegisterTypes_osimSimmKinematicsEngine\n";

	Object::RegisterType( SimmBody() );
	Object::RegisterType( SimmCoordinate() );
	Object::RegisterType( SimmJoint() );
	Object::RegisterType( SimmKinematicsEngine() );
	Object::RegisterType( SimmRotationDof() );
	Object::RegisterType( SimmTranslationDof() );
}

osimSimmKinematicsEngineInstantiator::osimSimmKinematicsEngineInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimmKinematicsEngineInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimSimmKinematicsEngine(); 
} 
    
