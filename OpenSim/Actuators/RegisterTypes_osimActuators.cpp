// RegisterTypes.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimActuators.h"

#include "GeneralizedForceAtv.h"
#include "JointMoment.h"
#include "LinearSetPoint.h"
#include "PolynomialSetPoint.h"
#include "SimmDarrylMuscle.h"
#include "SimmZajacHill.h"
#include "Torque.h"

using namespace OpenSim;
using namespace std;

static osimActuatorsInstantiator instantiator;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
OSIMACTUATORS_API void RegisterTypes_osimActuators()
{
	//cout<<"RegisterTypes_osimActuators\n";

	Object::RegisterType( GeneralizedForceAtv() );
	Object::RegisterType( JointMoment() );
	Object::RegisterType( LinearSetPoint() );
	Object::RegisterType( PolynomialSetPoint() );
	Object::RegisterType( SimmDarrylMuscle() );
	Object::RegisterType( SimmZajacHill() );
	Object::RegisterType( Torque() );

}

osimActuatorsInstantiator::osimActuatorsInstantiator()
{
       registerDllClasses();
}

void osimActuatorsInstantiator::registerDllClasses()
{
       RegisterTypes_osimActuators();
}
