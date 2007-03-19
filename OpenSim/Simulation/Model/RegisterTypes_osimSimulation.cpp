// RegisterTypes_osimSimulation.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimulation.h"
#include "ContactForceSet.h"
#include "Force.h"
#include "AnalysisSet.h"
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>

#include "AbstractModel.h"
#include "ActuatorSet.h"
#include "BodyScale.h"
#include "BodyScaleSet.h"
#include "BodySet.h"
#include "BoneSet.h"
#include "CoordinateSet.h"
#include "DofSet.h"
#include "JointSet.h"
#include "MarkerSet.h"
#include "PolyObject.h"
#include "Marker.h"
#include "SpeedSet.h"
#include <OpenSim/Simulation/Wrap/MuscleWrap.h>
#include <OpenSim/Simulation/Wrap/MuscleWrapSet.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/Simulation/Wrap/WrapTorus.h>
#include <OpenSim/Simulation/Wrap/WrapObjectSet.h>

using namespace std;
using namespace OpenSim;

static osimSimulationInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the osimSimulation library.
 */
OSIMSIMULATION_API void RegisterTypes_osimSimulation()
{
	//cout<<"RegisterTypes_osimSimulation\n";

	Object::RegisterType( ControlSet() );
	Object::RegisterType( ControlConstant() );
	Object::RegisterType( ControlLinear() );
	Object::RegisterType( ControlLinearNode() );
	Object::RegisterType( Force() );
	Object::RegisterType( AnalysisSet() );

	Object::RegisterType( AbstractModel() );
	Object::RegisterType( ActuatorSet() );
	Object::RegisterType( BodyScale() );
	Object::RegisterType( BodyScaleSet() );
	Object::RegisterType( BodySet() );
	Object::RegisterType( BoneSet() );
	Object::RegisterType( ContactForceSet() );
	Object::RegisterType( CoordinateSet() );
	Object::RegisterType( DofSet() );
	Object::RegisterType( JointSet() );
	Object::RegisterType( MarkerSet() );
	Object::RegisterType( PolyObject() );
	Object::RegisterType( Marker() );
	Object::RegisterType( SpeedSet() );
	Object::RegisterType( MuscleWrap() );
	Object::RegisterType( MuscleWrapSet() );
	Object::RegisterType( WrapCylinder() );
	Object::RegisterType( WrapEllipsoid() );
	Object::RegisterType( WrapSphere() );
	Object::RegisterType( WrapTorus() );
	Object::RegisterType( WrapObjectSet() );
}

osimSimulationInstantiator::osimSimulationInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void osimSimulationInstantiator::registerDllClasses() 
{ 
        RegisterTypes_osimSimulation(); 
} 
    
