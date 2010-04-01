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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Common/PropertyDbl.h>
#include "SpringGeneralizedForce.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

using namespace OpenSim;
using namespace std;


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
SpringGeneralizedForce::~SpringGeneralizedForce()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
SpringGeneralizedForce::
SpringGeneralizedForce(string aCoordinateName) :
	CustomForce(),
	_coordName(aCoordinateName),
	_stiffness(_propStiffness.getValueDbl()),
	_restLength(_propRestLength.getValueDbl()),
	_viscosity(_propViscosity.getValueDbl()),
	_coord(NULL)
{
	// NULL
	setNull();

	if (_model) {
		_coord = &_model->updCoordinateSet().get(_coordName);
	} 
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aforce force to be copied.
 */
SpringGeneralizedForce::
SpringGeneralizedForce(const SpringGeneralizedForce &aForce) :
	CustomForce(aForce),
	_coordName(_propCoordinateName.getValueStr()),
	_stiffness(_propStiffness.getValueDbl()),
	_restLength(_propRestLength.getValueDbl()),
	_viscosity(_propViscosity.getValueDbl()),
	_coord(NULL)
{
	setNull();

	// MEMBER VARIABLES
	setStiffness(aForce.getStiffness());
	setRestLength(aForce.getRestLength());
	setViscosity(aForce.getViscosity());
}

//_____________________________________________________________________________
/**
 * Copy this force and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this force.
 */
Object* SpringGeneralizedForce::
copy() const
{
	SpringGeneralizedForce *force = new SpringGeneralizedForce(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this force to their null values.
 */
void SpringGeneralizedForce::
setNull()
{
	setType("SpringGeneralizedForce");
	setupProperties();

	// Defualt VALUES
	_stiffness = 0.0;
	_restLength = 0.0;
	_viscosity = 0.0;
}

	
//_____________________________________________________________________________
//
/**
 * Set the data members of this force to their null values.
 */
void SpringGeneralizedForce::
setupProperties()
{
	_propCoordinateName.setName("coordinate"); 
	_propertySet.append( &_propCoordinateName );

	_propStiffness.setName("stiffness");
	_propStiffness.setValue(0.0);
	_propertySet.append( &_propStiffness );

	_propRestLength.setName("rest_length");
	_propRestLength.setValue(0.0);
	_propertySet.append( &_propRestLength );

	_propViscosity.setName("viscosity");
	_propViscosity.setValue(0.0);
	_propertySet.append( &_propViscosity );
}


//=============================================================================
// OPERATORS
//=============================================================================
//-----------------------------------------------------------------------------
// ASSIGNMENT
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return  Reference to the altered object.
 */
SpringGeneralizedForce& SpringGeneralizedForce::
operator=(const SpringGeneralizedForce &aForce)
{
	// BASE CLASS
	CustomForce::operator =(aForce);

	// MEMBER VARIABLES
	setStiffness(aForce.getStiffness());
	setRestLength(aForce.getRestLength());
	setViscosity(aForce.getViscosity());

	return(*this);
}

//_____________________________________________________________________________
/**
 * setup sets the actual Coordinate reference _coord
 */
void SpringGeneralizedForce::
setup(Model& aModel)
{
	Force::setup( aModel);

	if (_model) {
		_coord = &_model->updCoordinateSet().get(_coordName);
	}
}

//=============================================================================
// GET AND SET
//=============================================================================
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
	_restLength = aRestLength;
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
	return(_restLength);
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
	_viscosity = aViscosity;
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
	return(_viscosity);
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
	_stiffness = aStiffness;
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

	return _stiffness;
}


//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the spring force to the
 * model.
 */
void SpringGeneralizedForce::computeForce(const SimTK::State& s, 
							      SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							      SimTK::Vector& generalizedForces) const
{
	if(_model==NULL || _coord == NULL) return;

	// FORCE
	double q = _coord->getValue(s);
	double speed =  _coord->getSpeedValue(s);
	double force = -getStiffness()*(q - _restLength) - _viscosity*speed;

	applyGeneralizedForce(s, *_coord, force, generalizedForces);
}
