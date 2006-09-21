// RegisterTypes_rdSimulation.cpp
// author: Frank C. Anderson
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <string>
#include <iostream>
#include <OpenSim/Tools/Object.h>
#include "RegisterTypes_rdSimulation.h"
#include "ActuatorSet.h"
#include "ContactForceSet.h"
#include "Force.h"
#include "Torque.h"
#include "Muscle.h"
#include "GeneralizedForce.h"
#include "AnalysisSet.h"
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Simulation/Control/ControlConstant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Simulation/SIMM/SimmBody.h>
#include <OpenSim/Simulation/SIMM/BodyScale.h>
#include <OpenSim/Simulation/SIMM/BodyScaleSet.h>
#include <OpenSim/Simulation/SIMM/SimmBone.h>
#include <OpenSim/Simulation/SIMM/Constant.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinateSet.h>
#include <OpenSim/Simulation/SIMM/SimmGenericModelParams.h>
#include <OpenSim/Simulation/SIMM/SimmIKParams.h>
#include <OpenSim/Simulation/SIMM/SimmIKTrialParams.h>
#include <OpenSim/Simulation/SIMM/SimmIKTrialParamsSet.h>
#include <OpenSim/Simulation/SIMM/SimmJoint.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarker.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPair.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPairSet.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerPlacementParams.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmMeasurement.h>
#include <OpenSim/Simulation/SIMM/SimmMeasurementSet.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmMotionEvent.h>
#include <OpenSim/Simulation/SIMM/SimmMuscle.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleGroup.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleGroupSet.h>
#include <OpenSim/Simulation/SIMM/SimmMuscleViaPoint.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePoint.h>
#include <OpenSim/Simulation/SIMM/SimmMusclePointSet.h>
#include <OpenSim/Simulation/SIMM/SimmPoint.h>
#include <OpenSim/Simulation/SIMM/SimmRotationDof.h>
#include <OpenSim/Simulation/SIMM/SimmScalingParams.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmTranslationDof.h>
#include <OpenSim/Simulation/SIMM/SimmDofSet.h>



using namespace OpenSim;
using namespace std;

static rdSimulationInstantiator instantiator; 

//_____________________________________________________________________________
/**
 * The purpose of this routine is to register all class types exported by
 * the rdSimulation library.
 */
RDSIMULATION_API void RegisterTypes_rdSimulation()
{
	cout<<"RegisterTypes_rdSimulation\n";

	Object::RegisterType( ControlSet() );
	Object::RegisterType( ControlConstant() );
	Object::RegisterType( ControlLinear() );
	Object::RegisterType( ControlLinearNode() );

	Object::RegisterType( ActuatorSet() );
	Object::RegisterType( Force() );
	Object::RegisterType( GeneralizedForce() );
	Object::RegisterType( Torque() );
	//Object::RegisterType( Muscle() );

	Object::RegisterType( ContactForceSet() );

	Object::RegisterType( AnalysisSet() );
	Object::RegisterType( SimmBody() );
	Object::RegisterType( BodyScale() );
	Object::RegisterType( BodyScaleSet() );
	Object::RegisterType( SimmBone() );
	Object::RegisterType( Constant() );
	Object::RegisterType( SimmCoordinate() );
	Object::RegisterType( SimmCoordinateSet() );
	Object::RegisterType( SimmGenericModelParams() );
	Object::RegisterType( SimmIKParams() );
	Object::RegisterType( SimmIKTrialParams() );
	Object::RegisterType( SimmIKTrialParamsSet() );
	Object::RegisterType( SimmJoint() );
	Object::RegisterType( SimmKinematicsEngine() );
	Object::RegisterType( SimmMarker() );
	Object::RegisterType( SimmMarkerPair() );
	Object::RegisterType( SimmMarkerPairSet() );
	Object::RegisterType( SimmMarkerPlacementParams() );
	Object::RegisterType( SimmMarkerSet() );
	Object::RegisterType( SimmMeasurement() );
	Object::RegisterType( SimmMeasurementSet() );
	Object::RegisterType( SimmModel() );
	Object::RegisterType( SimmMotionEvent() );
	Object::RegisterType( SimmMuscle() );
	Object::RegisterType( SimmMuscleGroup() );
	Object::RegisterType( SimmMuscleGroupSet() );
	Object::RegisterType( SimmMuscleViaPoint() );
	Object::RegisterType( SimmMusclePoint() );
	Object::RegisterType( SimmMusclePointSet() );
	Object::RegisterType( SimmPoint() );
	Object::RegisterType( SimmRotationDof() );
	Object::RegisterType( SimmScalingParams() );
	Object::RegisterType( SimmSubject() );
	Object::RegisterType( SimmTranslationDof() );
	Object::RegisterType( SimmDofSet() );

	//Object::RegisterType( IntegCallbackSet() );
	//Object::RegisterType( DerivCallbackSet() );

}

rdSimulationInstantiator::rdSimulationInstantiator() 
{ 
        registerDllClasses(); 
} 
    
void rdSimulationInstantiator::registerDllClasses() 
{ 
        RegisterTypes_rdSimulation(); 
} 
    
