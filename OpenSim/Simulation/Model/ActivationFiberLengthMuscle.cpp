// ActivationFiberLengthMuscle.cpp
// Author: Ajay Seth
/*
 * Copyright (c)  2012, Stanford University. All rights reserved. 
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
#include "ActivationFiberLengthMuscle.h"
#include "Model.h"

using namespace std;
using namespace OpenSim;
using SimTK::Vec3;


//==============================================================================
// STATIC INITIALIZATION
//==============================================================================
const string ActivationFiberLengthMuscle::STATE_ACTIVATION_NAME = "activation";
const string ActivationFiberLengthMuscle::STATE_FIBER_LENGTH_NAME = "fiber_length";


//==============================================================================
// CONSTRUCTORS
//==============================================================================
// Uses default (compiler-generated) destructor, copy constructor, and copy
// assignment operator.

//_____________________________________________________________________________
// Default constructor.
ActivationFiberLengthMuscle::ActivationFiberLengthMuscle()
{
	constructProperties();
}

//_____________________________________________________________________________
// Allocate and initialize properties.
void ActivationFiberLengthMuscle::constructProperties()
{
	constructProperty_default_activation(1.0);
	constructProperty_default_fiber_length(getOptimalFiberLength());
}

//_____________________________________________________________________________
// Allocate Simbody System resources for this actuator.
 void ActivationFiberLengthMuscle::createSystem(SimTK::MultibodySystem& system) const
{
	Super::createSystem(system);   // invoke superclass implementation

	addStateVariable(STATE_ACTIVATION_NAME);
	// Fiber length should be a position stage state variable.
	// That is setting the fiber length should force position and above
	// dependent cache to be reevaluated. Problem with doing this now
	// is that there are no position level Z variables and making it 
	// a Q (multibody position coordinate) will invalidate the whole
	// multibody position realization which is overkill and would
	// also wipe out the muscle path, which we do not want to 
	// reevaluate over and over.
	addStateVariable(STATE_FIBER_LENGTH_NAME);//, SimTK::Stage::Velocity);

	double value = 0.0;
	addCacheVariable(STATE_ACTIVATION_NAME+"_deriv", value, SimTK::Stage::Dynamics);
	addCacheVariable(STATE_FIBER_LENGTH_NAME+"_deriv", value, SimTK::Stage::Dynamics);
 }

 void ActivationFiberLengthMuscle::initState( SimTK::State& s) const
{
    Super::initState(s);   // invoke superclass implementation

	ActivationFiberLengthMuscle* mutableThis = const_cast<ActivationFiberLengthMuscle *>(this);

	setActivation(s, getDefaultActivation());
	setFiberLength(s, getDefaultFiberLength());
}

void ActivationFiberLengthMuscle::setDefaultsFromState(const SimTK::State& state)
{
	Super::setDefaultsFromState(state);    // invoke superclass implementation

    setDefaultActivation(getStateVariable(state, STATE_ACTIVATION_NAME));
    setDefaultFiberLength(getStateVariable(state, STATE_FIBER_LENGTH_NAME));
}

double ActivationFiberLengthMuscle::getDefaultActivation() const {
    return getProperty_default_activation();
}
void ActivationFiberLengthMuscle::setDefaultActivation(double activation) {
    setProperty_default_activation(activation);
}
double ActivationFiberLengthMuscle::getDefaultFiberLength() const {
    return getProperty_default_fiber_length();
}
void ActivationFiberLengthMuscle::setDefaultFiberLength(double length) {
    setProperty_default_fiber_length(length);
}

//_____________________________________________________________________________
/**
 * Get the name of a state variable, given its index.
 *
 * @param aIndex The index of the state variable to get.
 * @return The name of the state variable.
 */
Array<std::string> ActivationFiberLengthMuscle::getStateVariableNames() const
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
 * Set the derivative of an actuator state, specified by name
 *
 * @param aStateName The name of the state to set.
 * @param aValue The value to set the state to.
 */
void ActivationFiberLengthMuscle::
setStateVariableDeriv(const SimTK::State& s, const std::string &aStateName, 
                      double aValue) const
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
double ActivationFiberLengthMuscle::
getStateVariableDeriv(const SimTK::State& s, const std::string &aStateName) const
{
	return getCacheVariable<double>(s, aStateName + "_deriv");
}

//_____________________________________________________________________________
/**
 * Compute the derivatives of the muscle states.
 *
 * @param s  system state
 */
SimTK::Vector ActivationFiberLengthMuscle::
computeStateVariableDerivatives(const SimTK::State &s) const
{
	SimTK::Vector derivs(getNumStateVariables());
	derivs[0] = getActivationRate(s);
	derivs[1] = getFiberVelocity(s);
	//derivs.dump("ActivationFiberLengthMuscle::derivs");
	return derivs;
}


//==============================================================================
// GET
//==============================================================================
//-----------------------------------------------------------------------------
// STATE VALUES
//-----------------------------------------------------------------------------

void ActivationFiberLengthMuscle::setActivation(SimTK::State& s, double activation) const
{
	setStateVariable(s, STATE_ACTIVATION_NAME, activation);
}

void ActivationFiberLengthMuscle::setFiberLength(SimTK::State& s, double fiberLength) const
{
	setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength);
	// NOTE: This is a temporary measure since we were forced to allocate
	// fiber length as a Dynamics stage dependent state variable.
	// In order to force the recalculation of the length cache we have to 
	// invalidate the length info whenever fiber length is set.
	markCacheVariableInvalid(s,"lengthInfo");
    markCacheVariableInvalid(s,"velInfo");
    markCacheVariableInvalid(s,"dynamicsInfo");
    
}

double ActivationFiberLengthMuscle::getActivationRate(const SimTK::State& s) const
{
	return calcActivationRate(s);
}


//==============================================================================
// SCALING
//==============================================================================

//_____________________________________________________________________________
/**
 * Perform computations that need to happen after the muscle is scaled.
 * For this object, that entails updating the muscle path. Derived classes
 * should probably also scale or update some of the force-generating
 * properties.
 *
 * @param aScaleSet XYZ scale factors for the bodies.
 */
void ActivationFiberLengthMuscle::
postScale(const SimTK::State& s, const ScaleSet& aScaleSet)
{
	GeometryPath& path = updProperty_GeometryPath();

	path.postScale(s, aScaleSet);

	if (path.getPreScaleLength(s) > 0.0)
		{
			double scaleFactor = getLength(s) / path.getPreScaleLength(s);
			updProperty_optimal_fiber_length() *= scaleFactor;
			updProperty_tendon_slack_length() *= scaleFactor;
			path.setPreScaleLength(s, 0.0) ;
		}
}

//--------------------------------------------------------------------------
// COMPUTATIONS
//--------------------------------------------------------------------------
void ActivationFiberLengthMuscle::computeInitialFiberEquilibrium(SimTK::State& s) const
{
	// Reasonable initial activation value
	//cout << getName() << "'s activation is: " << getActivation(s) << endl;
	//cout << getName() << "'s fiber-length is: " << getFiberLength(s) << endl;
	if(getActivation(s) < 0.01){
		setActivation(s, 0.01);
	}
	//cout << getName() << "'s NEW activation is: " << getActivation(s) << endl;
	setFiberLength(s, getOptimalFiberLength());
	_model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
	double force = computeIsometricForce(s, getActivation(s));
	//cout << getName() << "'s Equilibrium activation is: " << getActivation(s) << endl;
	//cout << getName() << "'s Equilibrium fiber-length is: " << getFiberLength(s) << endl;
}

//==============================================================================
// FORCE APPLICATION
//==============================================================================
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
double ActivationFiberLengthMuscle::
computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, 
                                                    double aActivation) const
{
	double isometricForce = computeIsometricForce(s, aActivation);

	const double &optimalFiberLength = getProperty_optimal_fiber_length();
	const double &pennationAngleAtOptimal = getProperty_pennation_angle_at_optimal();
	const double &maxContractionVelocity = getProperty_max_contraction_velocity();

	double normalizedLength = getFiberLength(s) / optimalFiberLength;
	double normalizedVelocity = -cos(pennationAngleAtOptimal) 
        * getLengtheningSpeed(s) / (maxContractionVelocity * optimalFiberLength);
	
	// force-length
	double fLength = exp(-17.33 * fabs(pow(normalizedLength-1.0,3)));

	// force-velocity
	double fVelocity = 1.8  -  1.8 / (1.0 + exp( (0.04 - normalizedVelocity)/0.18) );

	double normalizedForceVelocity = aActivation * fLength * fVelocity;

	return isometricForce * normalizedForceVelocity;
}

SimTK::SystemYIndex ActivationFiberLengthMuscle::
getStateVariableSystemIndex(const string &stateVariableName) const
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