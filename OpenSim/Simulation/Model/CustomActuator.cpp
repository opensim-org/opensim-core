// CustomActuator.cpp
// Author: Peter Eastman
/*
 * Copyright (c) 2009, Stanford University. All rights reserved. 
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "CustomActuator.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <sstream>
#include <OpenSim/Simulation/Model/Model.h>
#include "SimTKsimbody.h"
#include "OpenSimForceSubsystem.h"
#include "ForceAdapter.h"
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

CustomActuator::CustomActuator()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
CustomActuator::CustomActuator(const CustomActuator &aAct) :
    Actuator(aAct)
{
}

/**
 * Create a SimTK::Force which implements this CustomForce.
 */
void CustomActuator::createSystem(SimTK::MultibodySystem& system) const
{
    ForceAdapter* adapter = new ForceAdapter(*this, _model->getSimbodyEngine());
    SimTK::Force::Custom force(_model->updUserForceSubsystem(), adapter);

	 // Beyond the const Component get the index so we can access the SimTK::Force later
	CustomActuator* mutableThis = const_cast<CustomActuator *>(this);
	mutableThis->_index = force.getForceIndex();
	// Add this line if exposing named states 
	mutableThis->_model->addModelComponent(this);

}

//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double CustomActuator::computePotentialEnergy(const SimTK::State& state) const
{
	return 0.0;
}

//-----------------------------------------------------------------------------
// METHODS TO APPLY FORCES AND TORQUES
//-----------------------------------------------------------------------------
void CustomActuator::applyForce(const SimTK::State &s, const OpenSim::Body &aBody, 
								const Vec3& aForce, Vector_<SpatialVec> &bodyForces) const
{
	_model->getMatterSubsystem().getMobilizedBody(SimTK::MobilizedBodyIndex(aBody.getIndex())).applyBodyForce(s, SimTK::SpatialVec(Vec3(0), aForce), bodyForces);
}

void CustomActuator::applyForceToPoint(const SimTK::State &s, const OpenSim::Body &aBody, const Vec3& aPoint, 
									   const Vec3& aForce, Vector_<SpatialVec> &bodyForces) const
{
	_model->getMatterSubsystem().addInStationForce(s, SimTK::MobilizedBodyIndex(aBody.getIndex()), 
												   aPoint, aForce, bodyForces);
}

void CustomActuator::applyTorque(const SimTK::State &s, const OpenSim::Body &aBody, 
								 const Vec3& aTorque, Vector_<SpatialVec> &bodyForces) const
{
	_model->getMatterSubsystem().addInBodyTorque(s, SimTK::MobilizedBodyIndex(aBody.getIndex()),
												 aTorque, bodyForces);
}

void CustomActuator::applyGeneralizedForce(const SimTK::State &s, const Coordinate &aCoord, 
										   double aForce, Vector &mobilityForces) const
{
	_model->getMatterSubsystem().addInMobilityForce(s, SimTK::MobilizedBodyIndex(aCoord.getBodyIndex()), 
									SimTK::MobilizerUIndex(aCoord.getMobilityIndex()), aForce, mobilityForces);
}

