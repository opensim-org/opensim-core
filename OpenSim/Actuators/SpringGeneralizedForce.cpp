// SpringGeneralizedForce.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//	AUTHORS:  Frank C. Anderson
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


//==============================================================================
// INCLUDES
//==============================================================================
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/Joint.h>
#include "SpringGeneralizedForce.h"

using namespace OpenSim;
using namespace std;


//==============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//==============================================================================

//_____________________________________________________________________________
// This also serves as the default constructor.
SpringGeneralizedForce::SpringGeneralizedForce(const string& coordinateName)
{
	setNull();
    constructProperties();

    if (!coordinateName.empty())
	    setProperty_coordinate(coordinateName);
}

//_____________________________________________________________________________
// Set the data members of this force to their null values.
void SpringGeneralizedForce::setNull()
{
	// no data members that need initializing
}

	
//_____________________________________________________________________________
//
/**
 * Set the data members of this force to their null values.
 */
void SpringGeneralizedForce::constructProperties()
{
	constructProperty_coordinate();
	constructProperty_stiffness(0.0);
	constructProperty_rest_length(0.0);
	constructProperty_viscosity(0.0);
}

//_____________________________________________________________________________
/**
 * setup sets the _model pointer to proper value
 * _coordinate is actually set inside _createSystem
 */
void SpringGeneralizedForce::setup(Model& model)
{
	Super::setup(model);

    _coord = &model.updCoordinateSet().get(getProperty_coordinate());
}

//==============================================================================
// GET AND SET
//==============================================================================
//-----------------------------------------------------------------------------
// REST LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the rest length of the spring.
 *
 * @param aRestLength Rest length of the spring.
 */
void SpringGeneralizedForce::
setRestLength(double aRestLength)
{
	setProperty_rest_length(aRestLength);
}
//_____________________________________________________________________________
/**
 * Get the rest length of the spring.
 *
 * @return Rest length of the spring.
 */
double SpringGeneralizedForce::
getRestLength() const
{
	return getProperty_rest_length();
}

//-----------------------------------------------------------------------------
// VISCOSITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the viscosity of the spring.  Normally the viscosity should be a
 * positive number.  Negative viscosities will put energy into the system
 * rather than apply a damping force.
 *
 * @param aViscosity Viscosity of the spring.
 */
void SpringGeneralizedForce::
setViscosity(double aViscosity)
{
	setProperty_viscosity(aViscosity);
}
//_____________________________________________________________________________
/**
 * Get the viscosity of the spring.
 *
 * @return Stiffness of the spring.
 */
double SpringGeneralizedForce::
getViscosity() const
{
	return getProperty_viscosity();
}

//-----------------------------------------------------------------------------
// STIFFNESS
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the stiffness of the spring.  Normally the stiffness is a positive
 * quantity.  Negative stiffnessess will result in an unstable system- the
 * force will push away from the rest length instead of pulling toward it.
 *
 * @param aStiffness Stiffness of the spring force.
 */
void SpringGeneralizedForce::
setStiffness(double aStiffness)
{
	setProperty_stiffness(aStiffness);
}
//_____________________________________________________________________________
/**
 * Get the stiffness of the force.
 *
 * @return Stiffness of the force.
 */
double SpringGeneralizedForce::
getStiffness() const
{

	return getProperty_stiffness();
}


//==============================================================================
// COMPUTATIONS
//==============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the spring force to the
 * model.
 * Force applied = -stiffness * (_coordinateValue - restLength) 
 *                   - viscosity * _coordinateSpeed
 */
void SpringGeneralizedForce::computeForce(const SimTK::State& s, 
							      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							      SimTK::Vector& generalizedForces) const
{
	if(_model==NULL || _coord == NULL) return;

	// FORCE
	applyGeneralizedForce(s, *_coord, computeForceMagnitude(s), generalizedForces);
}
//_____________________________________________________________________________
/**
 * setup sets the actual Coordinate reference _coord
 */
 void  SpringGeneralizedForce::
createSystem(SimTK::MultibodySystem& system) const {
    Super::createSystem( system );

	if (_model) {
        SpringGeneralizedForce* mthis = 
            const_cast<SpringGeneralizedForce*>(this);
		mthis->_coord = &_model->updCoordinateSet().get(getProperty_coordinate());
    }
     
}
/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> SpringGeneralizedForce::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName()+"_Force");
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> SpringGeneralizedForce::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);

	values.append(computeForceMagnitude(state));
	return values;
};

/**
 * Given SimTK::State object Compute the (signed) magnitude of the force applied
 * along the _coordinate
 */
double SpringGeneralizedForce::
computeForceMagnitude(const SimTK::State& s) const
{
	double q = _coord->getValue(s);
	double speed =  _coord->getSpeedValue(s);
	double force = -getStiffness()*(q - getProperty_rest_length()) 
                        - getProperty_viscosity()*speed;
	return force;
}
