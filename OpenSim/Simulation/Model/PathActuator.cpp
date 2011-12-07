// PathActuator.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2011, Stanford University. All rights reserved. 
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

/* 
 * Author: Ajay Seth
 */


//=============================================================================
// INCLUDES
//=============================================================================
#include "PathActuator.h"
#include "Model.h"
#include "PointForceDirection.h"

using namespace OpenSim;
using namespace std;


//=============================================================================
// STATICS
//=============================================================================


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Destructor.
 */
PathActuator::~PathActuator()
{
}
//_____________________________________________________________________________
/**
 * Default constructor.
 */
PathActuator::PathActuator() :	Actuator(),
	_pathProp(PropertyObj("", GeometryPath())),
	_path((GeometryPath&)_pathProp.getValueObj()),
	_optimalForce(_propOptimalForce.getValueDbl())
{
	// NULL
	setNull();
}
//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aForce Force to be copied.
 */
PathActuator::PathActuator(const PathActuator &aPathActuator) :
	Actuator(aPathActuator),
	_pathProp(PropertyObj("", GeometryPath())),
	_path((GeometryPath&)_pathProp.getValueObj()),
	_optimalForce(_propOptimalForce.getValueDbl())
{
	setNull();
	copyData(aPathActuator);
}
//_____________________________________________________________________________
/**
 * Copy this actuator and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this actuator.
 */
Object* PathActuator::
copy() const
{
	PathActuator *force = new PathActuator(*this);
	return force;
}


//=============================================================================
// CONSTRUCTION
//=============================================================================
//_____________________________________________________________________________
/**
 * Set the data members of this actuator to their null values.
 */
void PathActuator::setNull()
{
	setType("PathActuator");
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathActuator::setupProperties()
{
	_pathProp.setName("GeometryPath");
	_propertySet.append(&_pathProp);

	_propOptimalForce.setName("optimal_force");
	_propOptimalForce.setValue(1.0);
	_propertySet.append( &_propOptimalForce );
}

//_____________________________________________________________________________
/**
 * Copy the member data of the specified actuator.
 */
void PathActuator::copyData(const PathActuator &aPathActuator)
{
	// MEMBER VARIABLES
	_path = aPathActuator._path;
	setOptimalForce(aPathActuator.getOptimalForce());
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
 * @return  PathActuator with contents of aPathActuator.
 */
PathActuator& PathActuator::operator=(const PathActuator &aPathActuator)
{
	// BASE CLASS
	Actuator::operator =(aPathActuator);

	copyData(aPathActuator);

	return(*this);
}


//=============================================================================
// GET AND SET
//=============================================================================
//-----------------------------------------------------------------------------
// OPTIMAL FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the optimal force of the force.
 *
 * @param aOptimalForce Optimal force.
 */
void PathActuator::setOptimalForce(double aOptimalForce)
{
	_optimalForce = aOptimalForce;
}

//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double PathActuator::getOptimalForce() const
{
	return(_optimalForce);
}

//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Get the length of the path actuator. This is a convenience function that 
 * calls the underlying path object for its length.
 *
 * @return Current length of the actuator's path.
 */
double PathActuator::getLength(const SimTK::State& s) const
{
	return _path.getLength(s);
}
//_____________________________________________________________________________
/**
 * Get the speed of actuator along its path.
 *
 * @return path lengthening speed.
 */
double PathActuator::getLengtheningSpeed(const SimTK::State& s) const
{
	return(_path.getLengtheningSpeed(s));
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PathActuator::getStress( const SimTK::State& s) const
{
	return fabs(getForce(s)/_optimalForce); 
}


//_____________________________________________________________________________
/**
 * Add a Path point to the _path of the actuator. The new point is appended 
 * to the end of the current path
 *
 */
void PathActuator::addNewPathPoint(
		 const std::string& proposedName, 
		 OpenSim::Body& aBody, 
		 const SimTK::Vec3& aPositionOnBody) {
	// Create new PathPoint
	PathPoint* newPathPoint =_path.appendNewPathPoint(proposedName, aBody, aPositionOnBody);
	// Set offset/position on owner body
	newPathPoint->setName(proposedName);
	for (int i=0; i<3; i++)	// Use interface that does not depend on state
		newPathPoint->setLocationCoord(i, aPositionOnBody[i]);
}

//=============================================================================
// COMPUTATIONS
//=============================================================================
//_____________________________________________________________________________
/**
 * Compute all quantities necessary for applying the actuator force to the
 * model.
 */
double PathActuator::computeActuation( const SimTK::State& s ) const
{
	if(_model==NULL)
		return 0.0;

	// FORCE
	return( getControl(s) * _optimalForce );
}


//=============================================================================
// APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the actuator force along path wrapping over and connecting rigid bodies
 */
void PathActuator::computeForce( const SimTK::State& s, 
							   SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							   SimTK::Vector& mobilityForces) const
{
	if(_model==NULL) return;

	// compute path's lengthening speed if necessary
	double speed = _path.getLengtheningSpeed(s);

	// the lengthening speed of this actutor is the "speed" of the actuator used to compute power
	setSpeed(s, speed);

	double force =0;
	if( isForceOverriden(s) ) {
		force = computeOverrideForce(s);
	} else {
		force = computeActuation(s);
	}

	// the force of this actuator used to compute power
    setForce(s,  force );

	OpenSim::Array<PointForceDirection*> PFDs;
	_path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), force*PFDs[i]->direction(), bodyForces);
	}
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double PathActuator::computeMomentArm(SimTK::State& s, Coordinate& aCoord) const
{
	return _path.computeMomentArm(s, aCoord);
}

//_____________________________________________________________________________
/**
 * Perform some setup functions that happen after the
 * object has been deserialized or copied.
 *
 * @param aModel OpenSim model containing this PathActuator.
 */
void PathActuator::setup(Model& aModel)
{
	// Specify underlying ModelComponents prior to calling base::setup() to automatically 
	// propogate setup to subcomponents. Subsequent createSystem() will also be automatically
	// propogated.
	includeAsSubComponent(&_path);
	Actuator::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

	_path.setOwner(this);
}


//=============================================================================
// XML
//=============================================================================
//-----------------------------------------------------------------------------
// UPDATE FROM XML NODE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Update this object based on its XML node.
 *
 * This method simply calls Object::updateFromXMLNode() and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void PathActuator::updateFromXMLNode()
{
	Actuator::updateFromXMLNode();
	setOptimalForce(_optimalForce);
}	

/** 
 * Methods to query a Force for the value actually applied during simulation
 * The names of the quantities (column labels) is returned by this first function
 * getRecordLabels()
 */
OpenSim::Array<std::string> PathActuator::getRecordLabels() const {
	OpenSim::Array<std::string> labels("");
	labels.append(getName());
	return labels;
}
/**
 * Given SimTK::State object extract all the values necessary to report forces, application location
 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
 */
OpenSim::Array<double> PathActuator::getRecordValues(const SimTK::State& state) const {
	OpenSim::Array<double> values(1);
	values.append(getForce(state));
	return values;
};

//_____________________________________________________________________________
/**
 * Get the visible object used to represent the muscle.
 */
VisibleObject* PathActuator::getDisplayer() const
{ 
	return getGeometryPath().getDisplayer(); 
}

//_____________________________________________________________________________
/**
 * Update the visible object used to represent the muscle.
 */
void PathActuator::updateDisplayer(const SimTK::State& s)
{
	_path.updateDisplayer(s);
}

//=============================================================================
// SCALING
//=============================================================================
//_____________________________________________________________________________
/**
 * Perform computations that need to happen before the muscle is scaled.
 * For this object, that entails calculating and storing the muscle-tendon
 * length in the current body position.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void PathActuator::preScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.preScale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Scale the muscle based on XYZ scale factors for each body.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 * @return Whether muscle was successfully scaled or not.
 */
void PathActuator::scale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.scale(s, aScaleSet);
}

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * For this object, that entails updating the muscle path. Derived classes
 * should probably also scale or update some of the force-generating
 * properties.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void PathActuator::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.postScale(s, aScaleSet);
}