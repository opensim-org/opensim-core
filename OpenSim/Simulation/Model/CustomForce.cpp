// CustomForce.cpp
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
#include "CustomForce.h"
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

CustomForce::CustomForce() :
	_bodyForces(NULL),
	_mobilityForces(NULL),
	_matter(NULL),
	_state(NULL)
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aActuator Actuator to copy.
 */
CustomForce::CustomForce(const CustomForce &aForce) :
    Force(aForce),
	_bodyForces(NULL),
	_mobilityForces(NULL),
	_matter(NULL),
	_state(NULL)
{
}

/**
 * Create a SimTK::Force which implements this CustomForce.
 */
void CustomForce::createSystem(SimTK::MultibodySystem& system) const
{
    ForceAdapter* adapter = new ForceAdapter(*this, _model->getSimbodyEngine());
    SimTK::Force::Custom force(_model->updUserForceSubsystem(), adapter);

	 // Beyond the const Component get the index so we can access the SimTK::Force later
	CustomForce* mutableThis = const_cast<CustomForce *>(this);
	mutableThis->_index = force.getForceIndex();
}


//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double CustomForce::computePotentialEnergy(const SimTK::State& state) const
{
	return 0.0;
}

//-----------------------------------------------------------------------------
// METHODS TO APPLY FORCES AND TORQUES
//-----------------------------------------------------------------------------
void CustomForce::applyForce(const OpenSim::Body &aBody, const Vec3& aForce) const
{
	if (_state == NULL)
		throw Exception("Called applyForce() from outside computeForce()");
	_model->getMatterSubsystem().getMobilizedBody(SimTK::MobilizedBodyIndex(aBody.getIndex())).applyBodyForce(*_state, SimTK::SpatialVec(Vec3(0), aForce), *_bodyForces);
}

void CustomForce::applyForceToPoint(const OpenSim::Body &aBody, const Vec3& aPoint, const Vec3& aForce) const
{
	if (_state == NULL) printf(" CustomForce::applyForceToPoint state= %x \n", _state );

//		throw Exception("Called applyForceToPoint() from outside computeForce()");
	_model->getMatterSubsystem().addInStationForce(*_state, SimTK::MobilizedBodyIndex(aBody.getIndex()), aPoint, aForce, *_bodyForces);
}

void CustomForce::applyTorque(const OpenSim::Body &aBody, const Vec3& aTorque) const
{
	if (_state == NULL)
		throw Exception("Called applyTorque() from outside computeForce()");
	_model->getMatterSubsystem().addInBodyTorque(*_state, SimTK::MobilizedBodyIndex(aBody.getIndex()), aTorque, *_bodyForces);
}

void CustomForce::applyGeneralizedForce(const Coordinate &aCoord, double aForce) const
{
	if (_state == NULL)
		throw Exception("Called applyGeneralizedForce() from outside computeForce()");
     _model->getMatterSubsystem().addInMobilityForce(*_state, SimTK::MobilizedBodyIndex(aCoord.getBodyIndex()), SimTK::MobilizerUIndex(aCoord.getMobilityIndex()), aForce, *_mobilityForces);
}

//-----------------------------------------------------------------------------
// INTERFACE USED BY FORCEADAPTER
//-----------------------------------------------------------------------------
void CustomForce::computeForce(const SimbodyEngine& engine, const SimTK::State& state, SimTK::Vector_<SimTK::SpatialVec>& bodyForces, SimTK::Vector& mobilityForces) const
{
	_matter = &engine._model->getMatterSubsystem();
	_state = &state;
	_bodyForces = &bodyForces;
	_mobilityForces = &mobilityForces;

	computeForce(state);

	_matter = NULL;
	_state = NULL;
	_bodyForces = NULL;
	_mobilityForces = NULL;
}
