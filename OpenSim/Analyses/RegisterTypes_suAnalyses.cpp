// RegisterTypes_Analyses.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/Object.h>
#include "RegisterTypes_suAnalyses.h"
#include "Kinematics.h"
#include "Actuation.h"
#include "PointKinematics.h"
#include "BodyKinematics.h"
#include "ActuatorGeneralizedForces.h"

#include "InvestigationPerturbation.h"
#include "InvestigationForward.h"



using namespace OpenSim;
using namespace std;

static suAnalysesInstantiator instantiator; 
     
//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the Analyses library.
 */
SUANALYSES_API void RegisterTypes_suAnalyses()
{
	//cout<<"RegisterTypes_suAnalyses\n";

	Object::RegisterType( InvestigationPerturbation() );
	Object::RegisterType( InvestigationForward() );

	Object::RegisterType( Kinematics() );
	Object::RegisterType( Actuation() );
	Object::RegisterType( PointKinematics() );
	Object::RegisterType( BodyKinematics() );
	Object::RegisterType( ActuatorGeneralizedForces() );
}

suAnalysesInstantiator::suAnalysesInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void suAnalysesInstantiator::registerDllClasses() 
{ 
        RegisterTypes_suAnalyses(); 
} 
