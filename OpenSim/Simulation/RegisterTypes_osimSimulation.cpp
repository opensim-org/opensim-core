// RegisterTypes_osimSimulation.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Common/Object.h>
#include "RegisterTypes_osimSimulation.h"

#include "Model/ContactForceSet.h"
#include "Model/Force.h"
#include "Model/AnalysisSet.h"
#include "Model/AbstractModel.h"
#include "Model/ActuatorSet.h"
#include "Model/BodyScale.h"
#include "Model/BodyScaleSet.h"
#include "Model/BodySet.h"
#include "Model/BoneSet.h"
#include "Model/CoordinateSet.h"
#include "Model/DofSet.h"
#include "Model/JointSet.h"
#include "Model/Marker.h"
#include "Model/MarkerSet.h"
#include "Model/MusclePoint.h"
#include "Model/MuscleViaPoint.h"
#include "Model/PolyObject.h"
#include "Model/SpeedSet.h"
#include "Model/VisibleMarker.h"
#include "Control/ControlSet.h"
#include "Control/ControlConstant.h"
#include "Control/ControlLinear.h"
#include "Wrap/MuscleWrap.h"
#include "Wrap/MuscleWrapSet.h"
#include "Wrap/WrapCylinder.h"
#include "Wrap/WrapEllipsoid.h"
#include "Wrap/WrapSphere.h"
#include "Wrap/WrapTorus.h"
#include "Wrap/WrapObjectSet.h"

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
	Object::RegisterType( Marker() );
	Object::RegisterType( MarkerSet() );
	Object::RegisterType( MusclePoint() );
	Object::RegisterType( MuscleViaPoint() );
	Object::RegisterType( PolyObject() );
	Object::RegisterType( SpeedSet() );
	Object::RegisterType( VisibleMarker() );

	Object::RegisterType( ControlSet() );
	Object::RegisterType( ControlConstant() );
	Object::RegisterType( ControlLinear() );
	Object::RegisterType( ControlLinearNode() );

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
    
