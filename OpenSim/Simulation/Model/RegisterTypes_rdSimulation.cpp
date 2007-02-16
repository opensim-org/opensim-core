// RegisterTypes_rdSimulation.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/Object.h>
#include "RegisterTypes_rdSimulation.h"
#include "ContactForceSet.h"
#include "Force.h"
#include "Torque.h"
#include "Muscle.h"
#include "GeneralizedForce.h"
#include "AnalysisSet.h"
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>

#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/ActuatorSet.h>
#include <OpenSim/Simulation/SIMM/BodyScale.h>
#include <OpenSim/Simulation/SIMM/BodyScaleSet.h>
#include <OpenSim/Simulation/SIMM/BodySet.h>
#include <OpenSim/Simulation/SIMM/BoneSet.h>
#include <OpenSim/Simulation/SIMM/CoordinateSet.h>
#include <OpenSim/Simulation/SIMM/DofSet.h>
#include <OpenSim/Simulation/SIMM/IKTaskSet.h>
#include <OpenSim/Simulation/SIMM/IKCoordinateTask.h>
#include <OpenSim/Simulation/SIMM/IKMarkerTask.h>
#include <OpenSim/Simulation/SIMM/JointSet.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include <OpenSim/Simulation/SIMM/MuscleWrap.h>
#include <OpenSim/Simulation/SIMM/MuscleWrapSet.h>
#include <OpenSim/Simulation/SIMM/PolyObject.h>
#include <OpenSim/Simulation/SIMM/SimmBody.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/SIMM/SimmDarrylMuscle.h>
#include <OpenSim/Simulation/SIMM/SimmJoint.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarker.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerFrame.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPair.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPairSet.h>
#include <OpenSim/Simulation/SIMM/SimmMeasurement.h>
#include <OpenSim/Simulation/SIMM/SimmMeasurementSet.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionEvent.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleGroup.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePointSet.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePointSet.h>
#include <OpenSim/Simulation/SIMM/SimmPoint.h>
#include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
#include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
#include <OpenSim/Simulation/SIMM/SimmZajacHill.h>
#include <OpenSim/Simulation/SIMM/SpeedSet.h>
#include <OpenSim/Simulation/SIMM/WrapCylinder.h>
#include <OpenSim/Simulation/SIMM/WrapEllipsoid.h>
#include <OpenSim/Simulation/SIMM/WrapSphere.h>
#include <OpenSim/Simulation/SIMM/WrapTorus.h>
#include <OpenSim/Simulation/SIMM/WrapObjectSet.h>

using namespace std;
using namespace OpenSim;

static rdSimulationInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
RDSIMULATION_API void RegisterTypes_rdSimulation()
{
	//cout<<"RegisterTypes_rdSimulation\n";

	Object::RegisterType( ControlSet() );
	Object::RegisterType( ControlConstant() );
	Object::RegisterType( ControlLinear() );
	Object::RegisterType( ControlLinearNode() );
	Object::RegisterType( Torque() );
	Object::RegisterType( Force() );
	Object::RegisterType( GeneralizedForce() );
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
	Object::RegisterType( IKTaskSet() );
	Object::RegisterType( IKCoordinateTask() );
	Object::RegisterType( IKMarkerTask() );
	Object::RegisterType( JointSet() );
	Object::RegisterType( MarkerSet() );
	Object::RegisterType( MuscleWrap() );
	Object::RegisterType( MuscleWrapSet() );
	Object::RegisterType( PolyObject() );
	Object::RegisterType( SimmBody() );
	Object::RegisterType( SimmCoordinate() );
	Object::RegisterType( SimmDarrylMuscle() );
	Object::RegisterType( SimmJoint() );
	Object::RegisterType( SimmKinematicsEngine() );
	Object::RegisterType( SimmMarker() );
	Object::RegisterType( SimmMarkerData() );
	Object::RegisterType( SimmMarkerFrame() );
	Object::RegisterType( SimmMarkerPair() );
	Object::RegisterType( SimmMarkerPairSet() );
	Object::RegisterType( SimmMeasurement() );
	Object::RegisterType( SimmMeasurementSet() );
	Object::RegisterType( SimmMotionData() );
	Object::RegisterType( SimmMotionEvent() );
	Object::RegisterType( SimmMuscleGroup() );
	Object::RegisterType( SimmMusclePoint() );
	Object::RegisterType( SimmMusclePointSet() );
	Object::RegisterType( SimmMuscleViaPoint() );
	Object::RegisterType( SimmMusclePointSet() );
	Object::RegisterType( SimmPoint() );
	Object::RegisterType( SimmRotationDof() );
	Object::RegisterType( SimmTranslationDof() );
	Object::RegisterType( SimmZajacHill() );
	Object::RegisterType( SpeedSet() );
	Object::RegisterType( WrapCylinder() );
	Object::RegisterType( WrapEllipsoid() );
	Object::RegisterType( WrapSphere() );
	Object::RegisterType( WrapTorus() );
	Object::RegisterType( WrapObjectSet() );
}

rdSimulationInstantiator::rdSimulationInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void rdSimulationInstantiator::registerDllClasses() 
{ 
        RegisterTypes_rdSimulation(); 
} 
    
