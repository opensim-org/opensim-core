// ActivationFiberLengthMuscle_Deprecated.cpp
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
#include "ActivationFiberLengthMuscle_Deprecated.h"
#include "Model.h"


//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

const int ActivationFiberLengthMuscle_Deprecated::STATE_ACTIVATION = 0;
const int ActivationFiberLengthMuscle_Deprecated::STATE_FIBER_LENGTH = 1;

const string ActivationFiberLengthMuscle_Deprecated::STATE_ACTIVATION_NAME = "activation";
const string ActivationFiberLengthMuscle_Deprecated::STATE_FIBER_LENGTH_NAME = "fiber_length";


//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ActivationFiberLengthMuscle_Deprecated::ActivationFiberLengthMuscle_Deprecated() : Muscle(),
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
 * @param aMuscle ActivationFiberLengthMuscle_Deprecated to be copied.
 */
ActivationFiberLengthMuscle_Deprecated::ActivationFiberLengthMuscle_Deprecated(const ActivationFiberLengthMuscle_Deprecated &aMuscle) : Muscle(aMuscle),
	_defaultActivation(aMuscle._defaultActivation),
	_defaultFiberLength(aMuscle._defaultFiberLength)
{
	setNull();
	setupProperties();
}
//_____________________________________________________________________________
/**
 * Destructor.
 */
ActivationFiberLengthMuscle_Deprecated::~ActivationFiberLengthMuscle_Deprecated()
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
//_____________________________________________________________________________
//_____________________________________________________________________________
/**
 * Set the data members of this ActivationFiberLengthMuscle_Deprecated to their null values.
 */
void ActivationFiberLengthMuscle_Deprecated::setNull()
{
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void ActivationFiberLengthMuscle_Deprecated::setupProperties()
{
}


void ActivationFiberLengthMuscle_Deprecated::equilibrate(SimTK::State& state) const
{
	// Reasonable initial activation value
	setActivation(state, 0.01);
	setFiberLength(state, getOptimalFiberLength());
	_model->getMultibodySystem().realize(state, SimTK::Stage::Velocity);

	// Compute isometric force to get starting value
	// of _fiberLength.
	computeIsometricForce(state, getActivation(state));
}

//_____________________________________________________________________________
/**
 * allocate and initialize the SimTK state for this acuator.
 */
 void ActivationFiberLengthMuscle_Deprecated::createSystem(SimTK::MultibodySystem& system) const
{
	Muscle::createSystem(system);

	addStateVariable(STATE_ACTIVATION_NAME);
	addStateVariable(STATE_FIBER_LENGTH_NAME);

	double value = 0.0;
	addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", value, SimTK::Stage::Dynamics);
	addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", value, SimTK::Stage::Dynamics);
	
	// Cache the computed active and passive muscle force
	// note the total muscle force is the tendon force and is already a cached variable of the actuator
	addCacheVariable<double>("activeForce", 0.0, SimTK::Stage::Velocity);
	addCacheVariable<double>("passiveForce", 0.0, SimTK::Stage::Velocity);
 }

 void ActivationFiberLengthMuscle_Deprecated::initState( SimTK::State& s) const
{
    Muscle::initState(s);

	ActivationFiberLengthMuscle_Deprecated* mutableThis = const_cast<ActivationFiberLengthMuscle_Deprecated *>(this);

	setActivation(s, _defaultActivation);
	setFiberLength(s, _defaultFiberLength);
}

void ActivationFiberLengthMuscle_Deprecated::setDefaultsFromState(const SimTK::State& state)
{
	Muscle::setDefaultsFromState(state);

    _defaultActivation = getActivation(state);
    _defaultFiberLength = getFiberLength(state);
}

double ActivationFiberLengthMuscle_Deprecated::getDefaultActivation() const {
    return _defaultActivation;
}
void ActivationFiberLengthMuscle_Deprecated::setDefaultActivation(double activation) {
    _defaultActivation = activation;
}
double ActivationFiberLengthMuscle_Deprecated::getDefaultFiberLength() const {
    return _defaultFiberLength;
}
void ActivationFiberLengthMuscle_Deprecated::setDefaultFiberLength(double length) {
    _defaultFiberLength = length;
}

//_____________________________________________________________________________
/**
 * Get the name of a state variable, given its index.
 *
 * @param aIndex The index of the state variable to get.
 * @return The name of the state variable.
 */
Array<std::string> ActivationFiberLengthMuscle_Deprecated::getStateVariableNames() const
{
	Array<std::string> stateVariableNames = ModelComponent::getStateVariableNames();
	// Make state variable names unique to this muscle
	for(int i=0; i<stateVariableNames.getSize(); ++i){
		stateVariableNames[i] = getName()+"."+stateVariableNames[i];
	}
	return stateVariableNames;
}

// STATES
//_____________________________________________________________________________
/**
 * Set the derivative of an actuator state, specified by index
 *
 * @param aStateName The name of the state to set.
 * @param aValue The value to set the state to.
 */
void ActivationFiberLengthMuscle_Deprecated::setStateVariableDeriv(const SimTK::State& s, const std::string &aStateName, double aValue) const
{
	double& cacheVariable = updCacheVariable<double>(s, aStateName + "_deriv");
	cacheVariable = aValue;
	markCacheVariableValid(s, aStateName + "_deriv");
}

//_____________________________________________________________________________
/**
 * Get the derivative of an actuator state, by index.
 *
 * @param aStateName the name of the state to get.
 * @return The value of the state.
 */
double ActivationFiberLengthMuscle_Deprecated::getStateVariableDeriv(const SimTK::State& s, const std::string &aStateName) const
{
	return getCacheVariable<double>(s, aStateName + "_deriv");
}

//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state
 */
SimTK::Vector ActivationFiberLengthMuscle_Deprecated::computeStateVariableDerivatives(const SimTK::State &s) const
{
	SimTK::Vector derivs(getNumStateVariables());
	derivs[0] = getActivationDeriv(s);
	derivs[1] = getFiberLengthDeriv(s);
	return derivs;
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
ActivationFiberLengthMuscle_Deprecated& ActivationFiberLengthMuscle_Deprecated::operator=(const ActivationFiberLengthMuscle_Deprecated &aMuscle)
{
	// base class
	Muscle::operator=(aMuscle);

	return(*this);
}


//=============================================================================
// GET
//=============================================================================
//-----------------------------------------------------------------------------
// LENGTH
//-----------------------------------------------------------------------------
double ActivationFiberLengthMuscle_Deprecated::getFiberLength(const SimTK::State& s) const {
	return getStateVariable(s, STATE_FIBER_LENGTH_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setFiberLength(SimTK::State& s, double fiberLength) const {
	setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength);
}
double ActivationFiberLengthMuscle_Deprecated::getFiberLengthDeriv(const SimTK::State& s) const {
	return getStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const {
	setStateVariableDeriv(s, STATE_FIBER_LENGTH_NAME, fiberLengthDeriv);
}
//_____________________________________________________________________________
/**
/**
 * Get the normalized length of the muscle fiber(s).  This is the current
 * fiber length(s) divided by the optimal fiber length.
 *
 * @param Current length of the muscle fiber(s).
 */
double ActivationFiberLengthMuscle_Deprecated::getNormalizedFiberLength(const SimTK::State& s) const
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
double ActivationFiberLengthMuscle_Deprecated::getFiberLengthAlongTendon(const SimTK::State& s) const
{
	return getFiberLength(s) * cos(getPennationAngle(s));
}
//_____________________________________________________________________________
/**
 * Get the length of the tendon.
 *
 * @return Current length of the tendon.
 */
double ActivationFiberLengthMuscle_Deprecated::getTendonLength(const SimTK::State& s) const
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
double ActivationFiberLengthMuscle_Deprecated::getFiberForce(const SimTK::State& s) const
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
double ActivationFiberLengthMuscle_Deprecated::getActiveFiberForce(const SimTK::State& s) const
{
	return getFiberForce(s) - getPassiveFiberForce(s);
}
/**
 * Get the passive force generated by the muscle fibers.
 *
 * @param Current passive force of the muscle fiber(s).
 */
double ActivationFiberLengthMuscle_Deprecated::getPassiveFiberForce(const SimTK::State& s) const
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
double ActivationFiberLengthMuscle_Deprecated::getActiveFiberForceAlongTendon(const SimTK::State& s) const
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
double ActivationFiberLengthMuscle_Deprecated::getPassiveFiberForceAlongTendon(const SimTK::State& s) const
{
	return getPassiveFiberForce(s) * cos(getPennationAngle(s));
}
double ActivationFiberLengthMuscle_Deprecated::getPassiveForce( const SimTK::State& s) const {
    return getCacheVariable<double>(s, "passiveForce");
}
void ActivationFiberLengthMuscle_Deprecated::setPassiveForce(const SimTK::State& s, double force ) const {
    setCacheVariable<double>(s, "passiveForce", force);
}

double ActivationFiberLengthMuscle_Deprecated::getTendonForce(const SimTK::State& s) const {
	return getForce(s);
}
void ActivationFiberLengthMuscle_Deprecated::setTendonForce(const SimTK::State& s, double force) const {
	setForce(s, force);
}
double ActivationFiberLengthMuscle_Deprecated::getActivation(const SimTK::State& s) const {
	return getStateVariable(s, STATE_ACTIVATION_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setActivation(SimTK::State& s, double activation) const {
	setStateVariable(s, STATE_ACTIVATION_NAME, activation);
}
double ActivationFiberLengthMuscle_Deprecated::getActivationDeriv(const SimTK::State& s) const {
	return getStateVariableDeriv(s, STATE_ACTIVATION_NAME);
}
void ActivationFiberLengthMuscle_Deprecated::setActivationDeriv(const SimTK::State& s, double activationDeriv) const {
	setStateVariableDeriv(s, STATE_ACTIVATION_NAME, activationDeriv);
}
//_____________________________________________________________________________
//**
// * get the excitation value for this ActivationFiberLengthMuscle_Deprecated 
// */
double ActivationFiberLengthMuscle_Deprecated::getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}
//_____________________________________________________________________________
/**
 * Get the stress in this actuator.  It is calculated as the force divided
 * by the maximum isometric force (which is proportional to its area).
 */
double ActivationFiberLengthMuscle_Deprecated::getStress(const SimTK::State& s) const
{
	return getForce(s) / getPropertyValue<double>("max_isometric_force");
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
void ActivationFiberLengthMuscle_Deprecated::postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath &path = updPropertyValue<GeometryPath>("GeometryPath");

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
		{
			double scaleFactor = getLength(s) / path.getPreScaleLength(s);
			updPropertyValue<double>("optimal_fiber_length") *= scaleFactor;
			updPropertyValue<double>("tendon_slack_length") *= scaleFactor;
			path.setPreScaleLength(s, 0.0) ;
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
void ActivationFiberLengthMuscle_Deprecated::computeForce(const SimTK::State& s, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForces) const
{
	Muscle::computeForce(s, bodyForces, generalizedForces);

	if( isForceOverriden(s) ) {
		// Also define the state derivatives, since realize acceleration will
		// ask for muscle derivatives, which will be integrated
		// in the case the force is being overridden, the states aren't being used
		// but a valid derivative cache entry is still required
		int numStateVariables = getNumStateVariables();
		Array<std::string> stateVariableNames = getStateVariableNames();
		for (int i = 0; i < numStateVariables; ++i) {
			stateVariableNames[i] = stateVariableNames[i].substr(stateVariableNames[i].find('.') + 1);
			setStateVariableDeriv(s, stateVariableNames[i], 0.0);
		}
    } 

}

//_____________________________________________________________________________
/**
 * Compute the equilibrium states.  This method computes a fiber length
 * for the muscle that is consistent with the muscle's activation level.
 */
void ActivationFiberLengthMuscle_Deprecated::computeInitialFiberEquilibrium(SimTK::State& s) const
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
double ActivationFiberLengthMuscle_Deprecated::computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation) const
{
	double isometricForce = computeIsometricForce(s, aActivation);

	const double &optimalFiberLength = getPropertyValue<double>("optimal_fiber_length");
	const double &pennationAngleAtOptimal = getPropertyValue<double>("pennation_angle_at_optimal");
	const double &maxContractionVelocity = getPropertyValue<double>("max_contraction_velocity");

	double normalizedLength = getFiberLength(s) / optimalFiberLength;
	double normalizedVelocity = -cos(pennationAngleAtOptimal) * getLengtheningSpeed(s) / (maxContractionVelocity * optimalFiberLength);
	double normalizedForceVelocity = evaluateForceLengthVelocityCurve(1.0,1.0,normalizedVelocity);

	return isometricForce * normalizedForceVelocity;
}

SimTK::SystemYIndex ActivationFiberLengthMuscle_Deprecated::getStateVariableSystemIndex(const string &stateVariableName) const
{
	unsigned int start = stateVariableName.find(".");
	unsigned int end = stateVariableName.length();
	
	if(start == end)
		return ModelComponent::getStateVariableSystemIndex(stateVariableName);
	else{
		string localName = stateVariableName.substr(++start, end-start);
		return ModelComponent::getStateVariableSystemIndex(localName);
	}
}

//=============================================================================
// GENERIC NORMALIZED FORCE-LENGTH-VELOCIY PROPERTIES
//=============================================================================
//_____________________________________________________________________________
/**
 * Evaluate the normalized force-length-velocity curve for the Muscle_Deprecated.
 * A simple generic implementation is used here.  Derived classes should
 * override this method for more precise evaluation of the
 * force-length-velocity curve.
 *
 * @param aActivation Activation level of the Muscle_Deprecated.  1.0 is full activation;
 * 0.0 is no activation.
 * @param aNormalizedLength Normalized length of the Muscle_Deprecated fibers.  1.0 indicates
 * the Muscle_Deprecated fibers are at their optimal length.  Lnorm = L / Lo.
 * @param aNormalizedVelocity Normalized shortening velocity of the Muscle_Deprecated fibers.
 * Positive values indicate concentric contraction (shortening); negative values
 * indicate eccentric contraction (lengthening).  Normalized velocity is
 * the fiber shortening velocity divided by the maximum shortening velocity times
 * the optimal fiber length.  Vnorm = V / (Vmax*Lo).
 * @return Force normalized by the optimal force.
 */
double ActivationFiberLengthMuscle_Deprecated::evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity) const
{
	// force-length
	double fLength = exp(-17.33 * fabs(pow(aNormalizedLength-1.0,3)));

	// force-velocity
	double fVelocity = 1.8  -  1.8 / (1.0 + exp( (0.04 - aNormalizedVelocity)/0.18) );

	return aActivation * fLength * fVelocity;
}


//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * Muscle_Deprecated. Pennation angle increases as Muscle_Deprecated fibers shorten. The implicit
 * modeling assumption is that Muscle_Deprecateds have constant width.
 *
 * @param aFiberLength Current fiber length of Muscle_Deprecated.
 * @param aOptimalFiberLength Optimal fiber length of Muscle_Deprecated.
 * @param aInitialPennationAngle Pennation angle at optimal fiber length (in radians).
 * @return Current pennation angle (in radians).
 */
double ActivationFiberLengthMuscle_Deprecated::calcPennation( double aFiberLength, double aOptimalFiberLength,
											    double aInitialPennationAngle) const
{
	if (aFiberLength < SimTK::Eps)
		return 0.0;

   double value = aOptimalFiberLength * sin(aInitialPennationAngle) / aFiberLength;

   if ( isnan(value)  ) 
       return 0.0;
   else if (value <= 0.0 )
      return 0.0;
   else if (value >= 1.0)
		return SimTK_PI/2.0;
   else
      return asin(value);
}

//=============================================================================
// CALCULATIONS
//=============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
	normalized lengths, pennation angle, etc... */
void ActivationFiberLengthMuscle_Deprecated::calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{
	double norm_muscle_tendon_length = getLength(s) / getOptimalFiberLength();
	
	mli.fiberLength = getStateVariable(s, STATE_FIBER_LENGTH_NAME);
	
	mli.pennationAngle = calcPennation(mli.fiberLength, getOptimalFiberLength(), getPennationAngleAtOptimalFiberLength());

	mli.cosPennationAngle = cos(mli.pennationAngle);
	mli.tendonLength = getLength(s)-mli.fiberLength*mli.cosPennationAngle;
	
	mli.normFiberLength = mli.fiberLength/getOptimalFiberLength();
	mli.normTendonLength = norm_muscle_tendon_length - mli.normFiberLength * mli.cosPennationAngle;
	mli.tendonStrain = (mli.tendonLength/getTendonSlackLength()-1.0);

	mli.fiberActiveForceLengthMultiplier = calcActiveForce(s, mli.normFiberLength);
	mli.fiberPassiveForceLengthMultiplier = calcPassiveForce(s, mli.normFiberLength);

	mli.musclePotentialEnergy = 0;
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
	normalized velocities, pennation angular velocity, etc... */
void ActivationFiberLengthMuscle_Deprecated::calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
	fvi.fiberVelocity = getFiberLengthDeriv(s);
	fvi.normFiberVelocity = fvi.fiberVelocity/(getOptimalFiberLength()*getMaxContractionVelocity());
}

/* calculate muscle's active and passive force-length, force-velocity, 
	tendon force, relationships and their related values */
void ActivationFiberLengthMuscle_Deprecated::calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
	const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
	const double &maxIsometricForce = getMaxIsometricForce();

	double tendonForce = getForce(s);
	mdi.normTendonForce = tendonForce/maxIsometricForce;
	
	mdi.passiveFiberForce = mli.fiberPassiveForceLengthMultiplier * maxIsometricForce;
	
	mdi.activation = getStateVariable(s, STATE_ACTIVATION_NAME);

	mdi.activeFiberForce =  tendonForce/mli.cosPennationAngle - mdi.passiveFiberForce;
}