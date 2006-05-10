// RegisterTypes.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/Object.h>
#include "RegisterTypes_rdActuators.h"
#include "GeneralizedForceAtv.h"
#include "LinearSetPoint.h"
#include "PolynomialSetPoint.h"
#include "JointMoment.h"



using namespace OpenSim;
using namespace std;

static rdActuatorsInstantiator instantiator;


//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
RDACTUATORS_API void RegisterTypes_rdActuators()
{
	cout<<"RegisterTypes_rdActuators\n";

	Object::RegisterType( GeneralizedForceAtv() );
	Object::RegisterType( LinearSetPoint() );
	Object::RegisterType( PolynomialSetPoint() );
	Object::RegisterType( JointMoment() );
}

rdActuatorsInstantiator::rdActuatorsInstantiator()
{
       registerDllClasses();
}

void rdActuatorsInstantiator::registerDllClasses()
{
       RegisterTypes_rdActuators();
}