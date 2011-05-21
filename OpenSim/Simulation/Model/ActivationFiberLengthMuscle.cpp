// ActivationFiberLengthMuscle.cpp
// Author: Peter Loan, Jeff Reinbolt
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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
#include <OpenSim/Common/XMLDocument.h>
#include <OpenSim/Common/XMLNode.h>
#include "ActivationFiberLengthMuscle.h"
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "ConditionalPathPoint.h"
#include "PointForceDirection.h"
#include "GeometryPath.h"
#include <OpenSim/Simulation/Wrap/PathWrapPoint.h>
#include <OpenSim/Simulation/Wrap/WrapResult.h>
#include <OpenSim/Simulation/Wrap/PathWrap.h>
#include <OpenSim/Simulation/Model/CoordinateSet.h>
#include "Model.h"
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Common/DebugUtilities.h>
#include "SimTKsimbody.h"

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

const int ActivationFiberLengthMuscle::STATE_ACTIVATION = 0;
const int ActivationFiberLengthMuscle::STATE_FIBER_LENGTH = 1;

static int counter=0;
//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ActivationFiberLengthMuscle::ActivationFiberLengthMuscle() : Muscle(),
	_defaultActivation(0),
	_defaultFiberLength(0)
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ActivationFiberLengthMuscle::~ActivationFiberLengthMuscle()
{
	VisibleObject* disp;
	if ((disp = getDisplayer())){
		 // Free up allocated geometry objects
		disp->freeGeometry();
	}
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle ActivationFiberLengthMuscle to be copied.
 */
ActivationFiberLengthMuscle::ActivationFiberLengthMuscle(const ActivationFiberLengthMuscle &aMuscle) : Muscle(aMuscle),
	_defaultActivation(aMuscle._defaultActivation),
	_defaultFiberLength(aMuscle._defaultFiberLength)
{
	setNull();
	setupProperties();
	copyData(aMuscle);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one ActivationFiberLengthMuscle to another.
 *
 * @param aMuscle ActivationFiberLengthMuscle to be copied.
 */
void ActivationFiberLengthMuscle::copyData(const ActivationFiberLengthMuscle &aMuscle)
{
	Muscle::copyData(aMuscle);
}

//_____________________________________________________________________________
/**
 * Set the data members of this ActivationFiberLengthMuscle to their null values.
 */
void ActivationFiberLengthMuscle::setNull()
{
	setType("ActivationFiberLengthMuscle");
}

//_____________________________________________________________________________
/**
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
void ActivationFiberLengthMuscle::updateFromXMLNode()
{
	// Call base class now assuming _node has been corrected for current version
	Muscle::updateFromXMLNode();
}

//_____________________________________________________________________________
/**
 * Perform set up functions after model has been deserialized or copied.
 *
 * @param aModel The model containing this muscle.
 */
void ActivationFiberLengthMuscle::setup(Model& aModel)
{
	Muscle::setup(aModel);

	// _model will be NULL when objects are being registered.
	if (_model == NULL)
		return;

    setNumStateVariables(2);

	_stateVariableSuffixes[STATE_ACTIVATION]="activation";
	_stateVariableSuffixes[STATE_FIBER_LENGTH]="fiber_length";
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this acuator.
 */
 void ActivationFiberLengthMuscle::createSystem(SimTK::MultibodySystem& system) const
{
	PathActuator::createSystem(system);

	ActivationFiberLengthMuscle* mutableThis = const_cast<ActivationFiberLengthMuscle *>(this);

	mutableThis->setNumStateVariables(_stateVariableSuffixes.getSize());
	mutableThis->addStateVariables(_stateVariableSuffixes);
	mutableThis->addCacheVariable<SimTK::Vector>("state_derivatives", SimTK::Vector(getNumStateVariables()), SimTK::Stage::Dynamics);

	mutableThis->_model->addModelComponent(this);
 }

 void ActivationFiberLengthMuscle::initState( SimTK::State& s) const
{
    Actuator::initState(s);

	ActivationFiberLengthMuscle* mutableThis = const_cast<ActivationFiberLengthMuscle *>(this);

	// keep track of the index for the first state variable derivatives in the cache 
	mutableThis->_zIndex = getZIndex(_stateVariableSuffixes[0]);

	setActivation(s, _defaultActivation);
	setFiberLength(s, _defaultFiberLength);
}

void ActivationFiberLengthMuscle::setDefaultsFromState(const SimTK::State& state)
{
	Actuator::setDefaultsFromState(state);
    _defaultActivation = getActivation(state);
    _defaultFiberLength = getFiberLength(state);
}

double ActivationFiberLengthMuscle::getDefaultActivation() const {
    return _defaultActivation;
}
void ActivationFiberLengthMuscle::setDefaultActivation(double activation) {
    _defaultActivation = activation;
}
double ActivationFiberLengthMuscle::getDefaultFiberLength() const {
    return _defaultFiberLength;
}
void ActivationFiberLengthMuscle::setDefaultFiberLength(double length) {
    _defaultFiberLength = length;
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ActivationFiberLengthMuscle::setupProperties()
{
}

//_____________________________________________________________________________
/**
 * Set the name of the muscle. This method overrides the one in Object
 * so that the path points can be [re]named accordingly.
 *
 * @param aName The new name of the muscle.
 */
void ActivationFiberLengthMuscle::setName(const string &aName)
{
	// base class
	Muscle::setName(aName);
}


//_____________________________________________________________________________
/**
 */
void ActivationFiberLengthMuscle::setNumStateVariables( int aNumStateVariables)
{
	_numStateVariables = aNumStateVariables;
	_stateVariableSuffixes.setSize(aNumStateVariables);
}

//_____________________________________________________________________________
/**
 * Get the name of a state variable, given its index.
 *
 * @param aIndex The index of the state variable to get.
 * @return The name of the state variable.
 */
string ActivationFiberLengthMuscle::getStateVariableName(int aIndex) const
{
	if(0<=aIndex && aIndex<_numStateVariables)
		return getName() + "." + _stateVariableSuffixes[aIndex];
	else {
		std::stringstream msg;
		msg << "Actuator::getStateVariableName: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " state variables.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

// STATES
int ActivationFiberLengthMuscle::getNumStateVariables() const
{
	ActivationFiberLengthMuscle* mutableThis = const_cast<ActivationFiberLengthMuscle *>(this);
    return mutableThis->updRep()->getNumStateVariablesAddedByModelComponent(); //+ numStatesOfUnderlyingComponent
}

//_____________________________________________________________________________
/**
 * Set the derivative of an actuator state, specified by index
 *
 * @param aIndex The index of the state to set.
 * @param aValue The value to set the state to.
 */
void ActivationFiberLengthMuscle::setStateVariableDeriv(const SimTK::State& s, int aIndex, double aValue) const {

	SimTK::Vector& stateDeriv =  updCacheVariable<SimTK::Vector>(s, "state_derivatives");
	if(0<=aIndex && aIndex<_numStateVariables) {
		stateDeriv[aIndex] = aValue;
	} else {
		std::stringstream msg;
		msg << "ActivationFiberLengthMuscle::setStateVariableDeriv: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
	markCacheVariableValid(s, "state_derivatives");
}

//_____________________________________________________________________________
/**
 * Get the derivative of an actuator state, by index.
 *
 * @param aIndex the index of the state to get.
 * @return The value of the state.
 */
double ActivationFiberLengthMuscle::getStateVariableDeriv(const SimTK::State& s, int aIndex) const
{
	const SimTK::Vector& stateDeriv = getCacheVariable<SimTK::Vector>(s, "state_derivatives");
	if(0<=aIndex && aIndex<_numStateVariables) {
        return( stateDeriv[aIndex] );
	} else {
		std::stringstream msg;
		msg << "MusclegetStateVariableDeriv: ERR- index out of bounds.\nActuator " 
		    << getName() << " of type " << getType() << " has " << getNumStateVariables() << " states.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}




//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @param aMuscle The muscle from which to copy its data
 * @return Reference to this object.
 */
ActivationFiberLengthMuscle& ActivationFiberLengthMuscle::operator=(const ActivationFiberLengthMuscle &aMuscle)
{
	// base class
	Actuator::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}


//=============================================================================
// GET
//=============================================================================
//_____________________________________________________________________________
/**
 * Get the length of the tendon.
 *
 * @return Current length of the tendon.
 */
double ActivationFiberLengthMuscle::getTendonLength(const SimTK::State& s) const
{
	return getLength(s) - getFiberLengthAlongTendon(s);
}
//_____________________________________________________________________________
/**
 * Get the length of the muscle fiber(s) along the tendon. This method
 * accounts for the pennation angle. 
 *
 * @return Current length of the muscle fiber(s) along the direction of
 * the tendon.
 */
double ActivationFiberLengthMuscle::getFiberLengthAlongTendon(const SimTK::State& s) const
{
	return getFiberLength(s) * cos(getPennationAngle(s));
}

//-----------------------------------------------------------------------------
// FORCE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Compute the force generated by the muscle fibers. This accounts for
 * pennation angle. That is, the fiber force is computed by dividing the
 * actuator force by the cosine of the pennation angle.
 *
 * @return Force in the muscle fibers.
 */
double ActivationFiberLengthMuscle::getFiberForce(const SimTK::State& s) const
{
	double force;
	double cos_penang = cos(getPennationAngle(s));
	if(fabs(cos_penang) < SimTK::Zero) {
		force = SimTK::NaN;
	} else {
		force = getForce(s) / cos_penang;
	}

	return force;
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers.
 *
 * @return Current active force of the muscle fibers.
 */
double ActivationFiberLengthMuscle::getActiveFiberForce(const SimTK::State& s) const
{
	return getFiberForce(s) - getPassiveFiberForce(s);
}
//_____________________________________________________________________________
/**
 * Get the active force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current active force of the muscle fibers along tendon.
 */
double ActivationFiberLengthMuscle::getActiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getActiveFiberForce(s) * cos(getPennationAngle(s));
}
//_____________________________________________________________________________
/**
 * Get the passive force generated by the muscle fibers along the direction
 * of the tendon.
 *
 * @return Current passive force of the muscle fibers along tendon.
 */
double ActivationFiberLengthMuscle::getPassiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getPassiveFiberForce(s) * cos(getPennationAngle(s));
}

//=============================================================================
// SCALING
//=============================================================================

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * For this object, that entails updating the muscle path. Derived classes
 * should probably also scale or update some of the force-generating
 * properties.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void ActivationFiberLengthMuscle::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	_path.postScale(s, aScaleSet);

	if (_path.getPreScaleLength(s) > 0.0)
		{
			double scaleFactor = getLength(s) / _path.getPreScaleLength(s);
			_optimalFiberLength *= scaleFactor;
			_tendonSlackLength *= scaleFactor;
			_path.setPreScaleLength(s, 0.0) ;
		}
}

//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------

//=============================================================================
// FORCE APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the muscle's force at its points of attachment to the bodies.
 */
void ActivationFiberLengthMuscle::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	Muscle::computeForce(s, bodyForces, generalizedForces);

	if( isForceOverriden(s) ) {
		// Also define the state derivatives, since realize acceleration will
		// ask for muscle derivatives, which will be integrated
		// in the case the force is being overridden, the states aren't being used
		// but a valid derivative cache entry is still required 
		SimTK::Vector& stateDerivs =  updCacheVariable<SimTK::Vector>(s, "state_derivatives");
		stateDerivs = 0.0;
		markCacheVariableValid(s, "state_derivatives");
    } 

}

//_____________________________________________________________________________
/**
 * Find the force produced by muscle under isokinetic conditions assuming
 * an infinitely stiff tendon.  That is, all the shortening velocity of the
 * actuator (the musculotendon unit) is assumed to be due to the shortening
 * of the muscle fibers alone.  This methods calls
 * computeIsometricForce and so alters the internal member variables of this
 * muscle.
 *
 *
 * Note that the current implementation approximates the effect of the
 * force-velocity curve.  It does not account for the shortening velocity
 * when it is solving for the equilibrium length of the muscle fibers.  And,
 * a generic representation of the force-velocity curve is used (as opposed
 * to the implicit force-velocity curve assumed by this model.
 *
 *
 * @param aActivation Activation of the muscle.
 * @return Isokinetic force generated by the actuator.
 * @todo Reimplement this methods with more accurate representation of the
 * force-velocity curve.
 */
double ActivationFiberLengthMuscle::computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation) const
{
	double isometricForce = computeIsometricForce(s, aActivation);

	double normalizedLength = getFiberLength(s) / _optimalFiberLength;
	double normalizedVelocity = -cos(_pennationAngleAtOptimal) * getLengtheningSpeed(s) / (_maxContractionVelocity * _optimalFiberLength);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,1.0,normalizedVelocity);

	return isometricForce * normalizedForceVelocity;
}

//_____________________________________________________________________________
//**
// * get the excitation value for this ActivationFiberLengthMuscle 
// */
double ActivationFiberLengthMuscle::getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}

void ActivationFiberLengthMuscle::equilibrate(SimTK::State& state) const
{
	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	_model->getMultibodySystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeEquilibrium(state);
}