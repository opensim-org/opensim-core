// ActivationFiberLengthMuscle.cpp
// Author: Peter Loan, Ajay Seth
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "ActivationFiberLengthMuscle.h"
#include "Model.h"


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

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
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
 * Connect properties to local pointers.
 */
void ActivationFiberLengthMuscle::setupProperties()
{
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

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this acuator.
 */
 void ActivationFiberLengthMuscle::createSystem(SimTK::MultibodySystem& system) const
{
	PathActuator::createSystem(system);

	ActivationFiberLengthMuscle* mutableThis = const_cast<ActivationFiberLengthMuscle *>(this);

	Array<string> stateVariables;
	stateVariables.setSize(2);
	stateVariables[0] = "activation";
	stateVariables[1] = "fiber_length";
	mutableThis->addStateVariables(stateVariables);
	mutableThis->addCacheVariable<SimTK::Vector>("state_derivatives", SimTK::Vector(getNumStateVariables()), SimTK::Stage::Dynamics);
	
	// Cache the computed active and passive muscle force
	// note the total muscle force is the tendon force and is already a cached variable of the actuator
	mutableThis->addCacheVariable<double>("activeForce", 0.0, SimTK::Stage::Velocity);
	mutableThis->addCacheVariable<double>("passiveForce", 0.0, SimTK::Stage::Velocity);

	mutableThis->_model->addModelComponent(this);
 }

 void ActivationFiberLengthMuscle::initState( SimTK::State& s) const
{
    Actuator::initState(s);

	ActivationFiberLengthMuscle* mutableThis = const_cast<ActivationFiberLengthMuscle *>(this);

	// keep track of the index for the first state variable derivatives in the cache 
	mutableThis->_zIndex = getZIndex("activation");

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
 * Get the name of a state variable, given its index.
 *
 * @param aIndex The index of the state variable to get.
 * @return The name of the state variable.
 */
string ActivationFiberLengthMuscle::getStateVariableName(int aIndex) const
{
	if(aIndex == 0)
		return getName() + ".activation";
	else if (aIndex == 1)
		return getName() + ".fiber_length";
	else {
		std::stringstream msg;
		msg << "Actuator::getStateVariableName: ERR- index out of bounds.\nActuator " 
			 << getName() << " of type " << getType() << " has " << getNumStateVariables() << " state variables.";
		throw( Exception(msg.str(),__FILE__,__LINE__) );
	}
}

// STATES
//_____________________________________________________________________________
/**
 * Set the derivative of an actuator state, specified by index
 *
 * @param aIndex The index of the state to set.
 * @param aValue The value to set the state to.
 */
void ActivationFiberLengthMuscle::setStateVariableDeriv(const SimTK::State& s, int aIndex, double aValue) const {

	SimTK::Vector& stateDeriv = updCacheVariable<SimTK::Vector>(s, "state_derivatives");
	if(0<=aIndex && aIndex<getNumStateVariables()) {
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
	if(0<=aIndex && aIndex<getNumStateVariables()) {
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
	Muscle::operator=(aMuscle);

	copyData(aMuscle);

	return(*this);
}


//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
double ActivationFiberLengthMuscle::getFiberLength(const SimTK::State& s) const {
	return getStateVariable(s, "fiber_length");
}
void ActivationFiberLengthMuscle::setFiberLength(SimTK::State& s, double fiberLength) const {
	setStateVariable(s, "fiber_length", fiberLength);
}
double ActivationFiberLengthMuscle::getFiberLengthDeriv(const SimTK::State& s) const {
	return getStateVariableDeriv(s, STATE_FIBER_LENGTH);
}
void ActivationFiberLengthMuscle::setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const {
	setStateVariableDeriv(s, STATE_FIBER_LENGTH, fiberLengthDeriv);
}
//_____________________________________________________________________________
/**
/**
 * Get the normalized length of the muscle fiber(s).  This is the current
 * fiber length(s) divided by the optimal fiber length.
 *
 * @param Current length of the muscle fiber(s).
 */
double ActivationFiberLengthMuscle::getNormalizedFiberLength(const SimTK::State& s) const
{
	return getFiberLength(s) / getOptimalFiberLength();
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
/**
 * Get the passive force generated by the muscle fibers.
 *
 * @param Current passive force of the muscle fiber(s).
 */
double ActivationFiberLengthMuscle::getPassiveFiberForce(const SimTK::State& s) const
{
	return getPassiveForce(s);
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
double ActivationFiberLengthMuscle::getPassiveForce( const SimTK::State& s) const {
    return getCacheVariable<double>(s, "passiveForce");
}
void ActivationFiberLengthMuscle::setPassiveForce(const SimTK::State& s, double force ) const {
    setCacheVariable<double>(s, "passiveForce", force);
}

double ActivationFiberLengthMuscle::getTendonForce(const SimTK::State& s) const {
	return getForce(s);
}
void ActivationFiberLengthMuscle::setTendonForce(const SimTK::State& s, double force) const {
	setForce(s, force);
}
double ActivationFiberLengthMuscle::getActivation(const SimTK::State& s) const {
	return getStateVariable(s, "activation");
}
void ActivationFiberLengthMuscle::setActivation(SimTK::State& s, double activation) const {
	setStateVariable(s, "activation", activation);
}
double ActivationFiberLengthMuscle::getActivationDeriv(const SimTK::State& s) const {
	return getStateVariableDeriv(s, STATE_ACTIVATION);
}
void ActivationFiberLengthMuscle::setActivationDeriv(const SimTK::State& s, double activationDeriv) const {
	setStateVariableDeriv(s, STATE_ACTIVATION, activationDeriv);
}
//_____________________________________________________________________________
//**
// * get the excitation value for this ActivationFiberLengthMuscle 
// */
double ActivationFiberLengthMuscle::getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}
//_____________________________________________________________________________
/**
 * Get the stress in this actuator.  It is calculated as the force divided
 * by the maximum isometric force (which is proportional to its area).
 */
double ActivationFiberLengthMuscle::getStress(const SimTK::State& s) const
{
	return getForce(s) / _maxIsometricForce;
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
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void ActivationFiberLengthMuscle::computeEquilibrium(SimTK::State& s) const
{
	double force = computeIsometricForce(s, getActivation(s));

	//cout<<getName()<<": isometric force = "<<force<<endl;
	//cout<<getName()<<": fiber length = "<<getFiberLength(s)<<endl;
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