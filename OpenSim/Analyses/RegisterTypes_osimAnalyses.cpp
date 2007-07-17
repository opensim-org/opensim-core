// RegisterTypes_Analyses.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimAnalyses.h"
#include "Kinematics.h"
#include "Actuation.h"
#include "PointKinematics.h"
#include "BodyKinematics.h"
#include "ActuatorGeneralizedForces.h"
#include "GeneralizedForces.h"
#include "MuscleAnalysis.h"
#include "InverseDynamics.h"



using namespace OpenSim;
using namespace std;

static suAnalysesInstantiator instantiator; 
     
//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Analyses library.
 */
OSIMANALYSES_API void RegisterTypes_suAnalyses()
{
	Object::RegisterType( Kinematics() );
	Object::RegisterType( Actuation() );
	Object::RegisterType( PointKinematics() );
	Object::RegisterType( BodyKinematics() );
	Object::RegisterType( ActuatorGeneralizedForces() );
   Object::RegisterType( GeneralizedForces() );
	Object::RegisterType( MuscleAnalysis() );
	Object::RegisterType( InverseDynamics() );
}

suAnalysesInstantiator::suAnalysesInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void suAnalysesInstantiator::registerDllClasses() 
{ 
        RegisterTypes_suAnalyses(); 
} 
