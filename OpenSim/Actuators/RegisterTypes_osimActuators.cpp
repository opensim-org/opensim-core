// RegisterTypes.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimActuators.h"

#include "CoordinateActuator.h"
#include "PointActuator.h"
#include "TorqueActuator.h"
#include "PointToPointActuator.h"
#include "Thelen2003Muscle.h"
#include "Schutte1993Muscle.h"
#include "Delp1990Muscle.h"
#include "CoordinateLimitForce.h"
#include "SpringGeneralizedForce.h"

#include "RigidTendonMuscle.h"

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

	Object::RegisterType( CoordinateActuator() );
	Object::RegisterType( PointActuator() );
	Object::RegisterType( TorqueActuator() );
	Object::RegisterType( PointToPointActuator() );

	Object::RegisterType( Thelen2003Muscle() );
	Object::RegisterType( Schutte1993Muscle() );
	Object::RegisterType( Delp1990Muscle() );
	Object::RegisterType( CoordinateLimitForce() );
	Object::RegisterType( SpringGeneralizedForce() );
	Object::RegisterType( RigidTendonMuscle() );

	// OLD Versions
	//Associate an instance with old name to help deserialization
	// This has to be done after the new Type is registered
	Object::RenameType("GeneralizedForce", CoordinateActuator());
	Object::RenameType("Force", PointActuator());
	Object::RenameType("Torque", TorqueActuator());
}

osimActuatorsInstantiator::osimActuatorsInstantiator()
{
       registerDllClasses();
}

void osimActuatorsInstantiator::registerDllClasses()
{
       RegisterTypes_osimActuators();
}
