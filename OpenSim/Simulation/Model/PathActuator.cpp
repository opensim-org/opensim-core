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
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
// default destructor, copy constructor, copy assignment

//_____________________________________________________________________________
/**
 * Default constructor.
 */
PathActuator::PathActuator()
{
	setNull();
    constructProperties();
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
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void PathActuator::constructProperties()
{
	constructProperty_GeometryPath(GeometryPath());
	constructProperty_optimal_force(1.0);
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
	setProperty_optimal_force(aOptimalForce);
}

//_____________________________________________________________________________
/**
 * Get the optimal force of the force.
 *
 * @return Optimal force.
 */
double PathActuator::getOptimalForce() const
{
	return getProperty_optimal_force();
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
	return getGeometryPath().getLength(s);
}
//_____________________________________________________________________________
/**
 * Get the speed of actuator along its path.
 *
 * @return path lengthening speed.
 */
double PathActuator::getLengtheningSpeed(const SimTK::State& s) const
{
	return getGeometryPath().getLengtheningSpeed(s);
}
//_____________________________________________________________________________
/**
 * Get the stress of the force.
 *
 * @return Stress.
 */
double PathActuator::getStress( const SimTK::State& s) const
{
	return fabs(getForce(s)/getProperty_optimal_force()); 
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
	PathPoint* newPathPoint = updGeometryPath()
        .appendNewPathPoint(proposedName, aBody, aPositionOnBody);
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
	return( getControl(s) * getProperty_optimal_force() );
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

	const GeometryPath &path = getGeometryPath();

	// compute path's lengthening speed if necessary
	double speed = path.getLengtheningSpeed(s);

	// the lengthening speed of this actutor is the "speed" of the actuator 
    // used to compute power
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
	path.getPointForceDirections(s, &PFDs);

	for (int i=0; i < PFDs.getSize(); i++) {
		applyForceToPoint(s, PFDs[i]->body(), PFDs[i]->point(), 
                          force*PFDs[i]->direction(), bodyForces);
	}
	for(int i=0; i < PFDs.getSize(); i++)
		delete PFDs[i];
}

/**
 * Compute the moment-arm of this muscle about a coordinate.
 */
double PathActuator::computeMomentArm(SimTK::State& s, Coordinate& aCoord) const
{
	return getGeometryPath().computeMomentArm(s, aCoord);
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
	GeometryPath &path = updGeometryPath();

	// Specify underlying ModelComponents prior to calling base::setup() to 
    // automatically propagate setup to subcomponents. Subsequent createSystem()
    // will also be automatically propagated.
	includeAsSubComponent(&path);
	Super::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

	path.setOwner(this);
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
 * This method simply calls Object::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) and then calls
 * a few methods in this class to ensure that variable members have been
 * set in a consistent manner.
 */
void PathActuator::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	updGeometryPath().setOwner(this);
	Super::updateFromXMLNode(aNode, versionNumber);
}	

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
	updGeometryPath().updateDisplayer(s);
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
	updGeometryPath().preScale(s, aScaleSet);
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
	updGeometryPath().scale(s, aScaleSet);
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
	updGeometryPath().postScale(s, aScaleSet);
}