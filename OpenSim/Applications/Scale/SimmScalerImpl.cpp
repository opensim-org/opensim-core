#include <OpenSim/Tools/Array.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/SQP/rdFSQP.h>
#include <OpenSim/Simulation/Model/AbstractDynamicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include "SimmScalerImpl.h"



using namespace OpenSim;
using namespace std;

SimmScalerImpl::SimmScalerImpl(Model &aModel):
ScalerInterface(aModel)
{
}

bool SimmScalerImpl::scaleModel(const ScaleSet& aScaleSet, bool aPreserveMassDist, double aFinalMass)
{
	// GET UNSCALED ACTUATOR LENGTHS



	// SCALE THE MUSCLE ATTACHMENT POINTS
	// Here we know we're in SIMM implementation so we may convert to the concrete simm* classes
	// Though it would be better to keep using the Model, AbstractDynamicsEngine and add whatever methods (e.g. getBodies())
	// and abstract classes (e.g. suBody and suJoint). This's all academic though until we have another 
	// kinematics engine -Ayman 2/06 
	SimmModel& sModel = dynamic_cast<SimmModel&> (_theModel);
	
	// Scale the muscles
	SimmMuscle* sm;
	int i;
	for (i = 0; i < sModel.getNumberOfMuscles(); i++)
	{
		if (sm = dynamic_cast<SimmMuscle*>(sModel.getMuscle(i)))
			sm->scale(aScaleSet);
	}

	// SCALE THE BODY SEGMENTAL PARMETERS
	// Scale the rest (KinematicsEngine stuff)
	sModel.getSimmKinematicsEngine().scale(aScaleSet, aPreserveMassDist, aFinalMass);


	// COMPUTE THE NEW SCALED ACTUATOR LENGTHS



	// COMPUTE THE LENGTH SCALE FACTORS FOR EACH OF THE ACTUATORS
	// factor = scaled_length / unscaled_length



	// SCALE THE LENGTH PARAMETERS FOR THE ACTUATORS
	// Each actuator should do this internally because different actuator types may
	// need to scale different things.


	return true;
}
