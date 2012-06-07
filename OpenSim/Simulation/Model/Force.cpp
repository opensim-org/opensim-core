// Force.cpp
// Author: Peter Eastman, Ajay Seth
/*
 * Copyright (c) 2009 Stanford University. All rights reserved. 
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

#include "Force.h"
#include "Model.h"
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/Model/ForceAdapter.h>


using namespace SimTK;

namespace OpenSim {

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
Force::Force()
{
	setNull();
	constructProperties();
}

//_____________________________________________________________________________
// Copy constructor.
Force::Force(const Force& source) : Super(source)
{
	copyData(source);
}

//_____________________________________________________________________________
// Copy assignment.
Force& Force::operator=(const Force& source)
{
	if (&source != this) {
	    Super::operator=(source);
	    copyData(source);
    }
	return *this;
}
//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
// Perform copy construction or assignment on local data members; base class
// (including properties) has already been copied.
void Force::copyData(const Force& source)
{
    copyProperty_isDisabled(source);
	// A copy is no longer a live Force with an underlying SimTK::Force. The
	// system must be created, at which time the Force will be assigned an index
	// corresponding to a valid system SimTK::Force.
	_index.invalidate();
}

//_____________________________________________________________________________
// Set the data members of this Force to their null values.
void Force::setNull()
{
    // the lone data member (_index) knows how to initialize itself
}

//_____________________________________________________________________________
// Define properties.
void Force::constructProperties()
{
	constructProperty_isDisabled(false);
}

// Create an underlying SimTK::Force to represent the OpenSim::Force in the 
// computational system.  Create a SimTK::Force::Custom by default.
void Force::addToSystem(SimTK::MultibodySystem& system) const
{
	Super::addToSystem(system);

	ForceAdapter* adapter = new ForceAdapter(*this);
    SimTK::Force::Custom force(_model->updForceSubsystem(), adapter);

	 // Beyond the const Component get the index so we can access the SimTK::Force later
	Force* mutableThis = const_cast<Force *>(this);
	mutableThis->_index = force.getForceIndex();
}


void Force::initStateFromProperties(SimTK::State& s) const
{
	Super::initStateFromProperties(s);

	SimTK::Force& simForce = _model->updForceSubsystem().updForce(_index);

	// Otherwise we have to change the status of the constraint
	if(get_isDisabled())
		simForce.disable(s);
	else
		simForce.enable(s);

}

void Force::setPropertiesFromState(const SimTK::State& state)
{
	Super::setPropertiesFromState(state);

    set_isDisabled(isDisabled(state));
}


//_____________________________________________________________________________
/**
 * Set whether or not this Force is disabled.
 * Simbody multibody system instance is realized every time the isDisabled
 * changes, BUT multiple sets to the same value have no cost.
 *
 * @param isDisabled If true the force is disabled; if false the Force is enabled.
 */
void Force::setDisabled(SimTK::State& s, bool isDisabled) 
{
	if(_index.isValid()){
		SimTK::Force& simtkForce = _model->updForceSubsystem().updForce(_index);
		if(isDisabled)
			simtkForce.disable(s);
		else
			simtkForce.enable(s);
	}
	set_isDisabled(isDisabled);
}

bool Force::isDisabled(const SimTK::State& s) const
{
	if(_index.isValid()){
		SimTK::Force& simtkForce = _model->updForceSubsystem().updForce(_index);
		return simtkForce.isDisabled(s);
	}
	return get_isDisabled();
}

//-----------------------------------------------------------------------------
// ABSTRACT METHODS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
double Force::computePotentialEnergy(const SimTK::State& state) const
{
	return 0.0;
}

//-----------------------------------------------------------------------------
// METHODS TO APPLY FORCES AND TORQUES
//-----------------------------------------------------------------------------
void Force::applyForceToPoint(const SimTK::State &s, const OpenSim::Body &aBody, const Vec3& aPoint, 
									const Vec3& aForce, Vector_<SpatialVec> &bodyForces) const
{
	_model->getMatterSubsystem().addInStationForce(s, SimTK::MobilizedBodyIndex(aBody.getIndex()), 
												   aPoint, aForce, bodyForces);
}

void Force::applyTorque(const SimTK::State &s, const OpenSim::Body &aBody, 
							  const Vec3& aTorque, Vector_<SpatialVec> &bodyForces) const
{
	_model->getMatterSubsystem().addInBodyTorque(s, SimTK::MobilizedBodyIndex(aBody.getIndex()),
												 aTorque, bodyForces);
}

void Force::applyGeneralizedForce(const SimTK::State &s, const Coordinate &aCoord, 
										double aForce, Vector &mobilityForces) const
{
	_model->getMatterSubsystem().addInMobilityForce(s, SimTK::MobilizedBodyIndex(aCoord.getBodyIndex()), 
									SimTK::MobilizerUIndex(aCoord.getMobilityIndex()), aForce, mobilityForces);
}


} // end of namespace OpenSim
