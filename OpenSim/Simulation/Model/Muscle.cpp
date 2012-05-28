// Muscle.cpp
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
#include "Muscle.h"

#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <OpenSim/Simulation/SimbodyEngine/SimbodyEngine.h>
#include "ConditionalPathPoint.h"
#include "PointForceDirection.h"
#include "GeometryPath.h"

#include "Model.h"


#include <OpenSim/Common/XMLDocument.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;
using SimTK::Vec3;

static int counter=0;
//=============================================================================
// CONSTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
Muscle::Muscle()
{
	constructProperties();
	// override the value of default _minControl, _maxControl
	setMinControl(0.0);
	setMaxControl(1.0);
}

//_____________________________________________________________________________
// Override default implementation by object to intercept and fix the XML node
// underneath the model to match current version.
void Muscle::updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber)
{
	if ( versionNumber < XMLDocument::getLatestVersion()) {
		if (Object::getDebugLevel()>=1)
			cout << "Updating Muscle object to latest format..." << endl;
		
		if (versionNumber <= 20301){
			SimTK::Xml::element_iterator pathIter = 
                                            aNode.element_begin("GeometryPath");
			if (pathIter != aNode.element_end()) {
				XMLDocument::renameChildNode(*pathIter, "MusclePointSet", "PathPointSet");
				XMLDocument::renameChildNode(*pathIter, "MuscleWrapSet", "PathWrapSet");
            } else { // There was no GeometryPath, just MusclePointSet
				XMLDocument::renameChildNode(aNode, "MusclePointSet", "PathPointSet");
				XMLDocument::renameChildNode(aNode, "MuscleWrapSet", "PathWrapSet");
				// Now create a "GeometryPath" node and move MusclePointSet & MuscleWrapSet under it
				SimTK::Xml::Element myPathElement("GeometryPath");
				SimTK::Xml::element_iterator  pathPointSetIter = aNode.element_begin("PathPointSet");
				SimTK::Xml::Node moveNode = aNode.removeNode(pathPointSetIter);
				myPathElement.insertNodeAfter(myPathElement.element_end(),moveNode);
				SimTK::Xml::element_iterator  pathWrapSetIter = aNode.element_begin("PathWrapSet");
				moveNode = aNode.removeNode(pathWrapSetIter);
				myPathElement.insertNodeAfter(myPathElement.element_end(),moveNode);
				aNode.insertNodeAfter(aNode.element_end(), myPathElement);
            }
			XMLDocument::renameChildNode(aNode, "pennation_angle", "pennation_angle_at_optimal");
        }
	}
	// Call base class now assuming aNode has been corrected for current version
	Super::updateFromXMLNode(aNode, versionNumber);
}


//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Muscle::constructProperties()
{
	constructProperty_max_isometric_force(1000.0);
	constructProperty_optimal_fiber_length(0.1);
	constructProperty_tendon_slack_length(0.2);
	constructProperty_pennation_angle_at_optimal(0.0);
	constructProperty_max_contraction_velocity(10.0);
	constructProperty_ignore_tendon_compliance(false);
	constructProperty_ignore_activation_dynamics(false);
}


//--------------------------------------------------------------------------
// MUSCLE PARAMETERS GETTERS AND SETTERS
//--------------------------------------------------------------------------
double Muscle::getMaxIsometricForce() const
{   return get_max_isometric_force(); }

double Muscle::getOptimalFiberLength() const
{   return get_optimal_fiber_length(); }

double Muscle::getTendonSlackLength() const 
{   return get_tendon_slack_length(); }

double Muscle::getPennationAngleAtOptimalFiberLength() const 
{   return get_pennation_angle_at_optimal(); }

double Muscle::getMaxContractionVelocity() const 
{   return get_max_contraction_velocity(); }

void Muscle::setMaxIsometricForce(double aMaxIsometricForce)
{   set_max_isometric_force(aMaxIsometricForce); }

void Muscle::setOptimalFiberLength(double aOptimalFiberLength) 
{   set_optimal_fiber_length(aOptimalFiberLength); }

void Muscle::setTendonSlackLength(double aTendonSlackLength) 
{   set_tendon_slack_length(aTendonSlackLength); }

void Muscle::setPennationAngleAtOptimalFiberLength(double aPennationAngle)
{   set_pennation_angle_at_optimal(aPennationAngle); }

void Muscle::setMaxContractionVelocity(double aMaxContractionVelocity) 
{   set_max_contraction_velocity(aMaxContractionVelocity); }


//=============================================================================
// ModelComponent Interface Implementation
//=============================================================================
void Muscle::setup(Model &aModel)
{
	Super::setup(aModel);

	_muscleWidth = getOptimalFiberLength()
                    * sin(getPennationAngleAtOptimalFiberLength());

	_maxIsometricForce = getMaxIsometricForce();
	_optimalFiberLength = getOptimalFiberLength();
	_pennationAngleAtOptimal = getPennationAngleAtOptimalFiberLength();
	_tendonSlackLength = getTendonSlackLength();
}

// Add Muscle's contributions to the underlying system
 void Muscle::createSystem(SimTK::MultibodySystem& system) const
{
	Super::createSystem(system);

	addModelingOption("ignore_tendon_compliance", 1);
	addModelingOption("ignore_activation_dynamics", 1);
	
	// Cache the calculated values for this muscle categorized by their realization stage 
	addCacheVariable<Muscle::MuscleLengthInfo>
       ("lengthInfo", MuscleLengthInfo(), SimTK::Stage::Position);
	addCacheVariable<Muscle::FiberVelocityInfo>
       ("velInfo", FiberVelocityInfo(), SimTK::Stage::Velocity);
	addCacheVariable<Muscle::MuscleDynamicsInfo>
       ("dynamicsInfo", MuscleDynamicsInfo(), SimTK::Stage::Dynamics);
 }

void Muscle::setDefaultsFromState(const SimTK::State& state)
{
    Super::setDefaultsFromState(state);

    set_ignore_tendon_compliance(getIgnoreTendonCompliance(state));
	set_ignore_activation_dynamics(getIgnoreActivationDynamics(state));
}

void  Muscle::initState(SimTK::State& state) const {
    Super::initState(state);

    setIgnoreTendonCompliance(state, 
        get_ignore_tendon_compliance());
    setIgnoreActivationDynamics(state, 
        get_ignore_activation_dynamics());
}

// Get/set runtime flag to ignore tendon compliance when computing muscle 
// dynamics.
bool Muscle::getIgnoreTendonCompliance(const SimTK::State& s) const
{
	return (getModelingOption(s, "ignore_tendon_compliance") > 0);
}

void Muscle::setIgnoreTendonCompliance(SimTK::State& s, bool ignore) const
{
	setModelingOption(s, "ignore_tendon_compliance", int(ignore));
}


/* get/set flag to activation dynamics when computing muscle dynamics  */
bool Muscle::getIgnoreActivationDynamics(const SimTK::State& s) const
{
	return (getModelingOption(s, "ignore_activation_dynamics") > 0);
}

void Muscle::setIgnoreActivationDynamics(SimTK::State& s, bool ignore) const
{
	setModelingOption(s, "ignore_activation_dynamics", int(ignore));
}



//=============================================================================
// GET values of interest from calculations
//=============================================================================
//_____________________________________________________________________________
//**
// * get the excitation value for this ActivationFiberLengthMuscle 
// */
double Muscle::getExcitation( const SimTK::State& s) const {
    return( getControl(s) );
}


/* get the activation level of the muscle, which modulates the active force of the muscle 
	and has a normalized (0 to 1) value */
double Muscle::getActivation(const SimTK::State& s) const
{
	const Muscle::MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.activation;
}

/* get the current working fiber length (m) for the muscle */
double Muscle::getFiberLength(const SimTK::State& s) const 
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.fiberLength; 
}

/* get the current pennation angle (radians) between the fiber and tendon at the current fiber length  */
double Muscle::getPennationAngle(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.pennationAngle; 
}

/* get the current tendon length (m)  given the current joint angles and fiber length */
double Muscle::getTendonLength(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.tendonLength; 
}

/* get the current normalized fiber length (fiber_length/optimal_fiber_length) */
double Muscle::getNormalizedFiberLength(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.normFiberLength; 
}

/* get the current fiber length (m) projected (*cos(pennationAngle)) onto the tendon direction */
double Muscle::getFiberLengthAlongTendon(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.fiberLength*mli.cosPennationAngle; 
}

/* get the current tendon strain (delta_l/lo is dimensionless)  */
double Muscle::getTendonStrain(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.tendonStrain;
}

/* the potential energy (J) stored in the fiber due to its parallel elastic element */
double Muscle::getFiberPotentialEnergy(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.fiberPotentialEnergy;
}

/* the potential energy (J) stored in the tendon */	
double Muscle::getTendonPotentialEnergy(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.tendonPotentialEnergy;
}

/* the total potential energy (J) stored in the muscle */	
double Muscle::getMusclePotentialEnergy(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.musclePotentialEnergy;
}

/* get the passive fiber (parallel elastic element) force multiplier */
double Muscle::getPassiveForceMultiplier(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.fiberPassiveForceLengthMultiplier;
}

/* get the active fiber (contractile element) force multiplier due to current fiber length */
double Muscle::getActiveForceLengthMultiplier(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.fiberActiveForceLengthMultiplier;
}

/* get current fiber velocity (m/s) positive is lengthening */
double Muscle::getFiberVelocity(const SimTK::State& s) const
{
	const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
	return fvi.fiberVelocity; 
}

/* get normalized fiber velocity (fiber_length/s / max_contraction_velocity) */
double Muscle::getNormalizedFiberVelocity(const SimTK::State& s) const
{
	const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
	return fvi.normFiberVelocity; 
}

/* get the current fiber velocity (m/s) projected onto the tendon direction */
double Muscle::getFiberVelocityAlongTendon(const SimTK::State& s) const 
{
	const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return fvi.fiberVelocity*mli.cosPennationAngle; 
}

/* get the tendon velocity (m/s) */
double Muscle::getTendonVelocity(const SimTK::State& s) const
{
	const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
	return fvi.tendonVelocity; 
}

/* get the dimensionless factor resulting from the fiber's force-velocity curve */
double Muscle::getForceVelocityMultiplier(const SimTK::State& s) const
{
	const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
	return fvi.fiberForceVelocityMultiplier; 
}


/* get pennation angular velocity (radians/s) */
double Muscle::getPennationAngularVelocity(const SimTK::State& s) const
{
	const FiberVelocityInfo& fvi = getFiberVelocityInfo(s);
	return fvi.pennationAngularVelocity; 
}

/* get the current fiber force (N) applied to the tendon */
double Muscle::getFiberForce(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.activeFiberForce+mdi.passiveFiberForce; 
}

/* get the current active fiber force (N) due to activation*force_length*force_velocity relationships */
double Muscle::getActiveFiberForce(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.activeFiberForce; 
}

/* get the current passive fiber force (N) passive_force_length relationship */
double Muscle::getPassiveFiberForce(const SimTK::State& s) const 
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.passiveFiberForce;
}

/* get the current active fiber force (N) projected onto the tendon direction */
double Muscle::getActiveFiberForceAlongTendon(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mdi.activeFiberForce*mli.cosPennationAngle;
}

/* get the current passive fiber force (N) projected onto the tendon direction */
double Muscle::getPassiveFiberForceAlongTendon(const SimTK::State& s) const 
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mdi.passiveFiberForce*mli.cosPennationAngle;
}

/* get the current tendon force (N) applied to bones */
double Muscle::getTendonForce(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return getMaxIsometricForce()*mdi.normTendonForce;
}

/* get the current fiber stiffness (N/m) defined as the partial derivative
	of fiber force w.r.t. fiber length */
double Muscle::getFiberStiffness(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.fiberStiffness;
}

/* get the current tendon stiffness (N/m) defined as the partial derivative
	of tendon force w.r.t. tendon length */
double Muscle::getTendonStiffness(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.tendonStiffness;
}

/* get the current muscle stiffness (N/m) defined as the partial derivative
	of muscle force w.r.t. muscle length */
double Muscle::getMuscleStiffness(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.muscleStiffness;
}

/* get the current fiber power (W) */
double Muscle::getFiberPower(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.fiberPower;
}

/* get the current tendon power (W) */
double Muscle::getTendonPower(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.tendonPower;
}

/* get the current muscle power (W) */
double Muscle::getMusclePower(const SimTK::State& s) const
{
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);
	return mdi.musclePower;
}


void Muscle::setExcitation(SimTK::State& s, double excitaion) const
{
	setControls(SimTK::Vector(1, excitaion), _model->updControls(s));
}

/* Access to muscle calculation data structures */
const Muscle::MuscleLengthInfo& Muscle::getMuscleLengthInfo(const SimTK::State& s) const
{
	if(!isCacheVariableValid(s,"lengthInfo")){
		MuscleLengthInfo &umli = updMuscleLengthInfo(s);
		calcMuscleLengthInfo(s, umli);
		markCacheVariableValid(s,"lengthInfo");
		// don't bother fishing it out of the cache since 
		// we just calculated it and still have a handle on it
		return umli;
	}
	return getCacheVariable<MuscleLengthInfo>(s, "lengthInfo");
}

Muscle::MuscleLengthInfo& Muscle::updMuscleLengthInfo(const SimTK::State& s) const
{
	return updCacheVariable<MuscleLengthInfo>(s, "lengthInfo");
}

const Muscle::FiberVelocityInfo& Muscle::
getFiberVelocityInfo(const SimTK::State& s) const
{
	if(!isCacheVariableValid(s,"velInfo")){
		FiberVelocityInfo& ufvi = updFiberVelocityInfo(s);
		calcFiberVelocityInfo(s, ufvi);
		markCacheVariableValid(s,"velInfo");
		// don't bother fishing it out of the cache since 
		// we just calculated it and still have a handle on it
		return ufvi;
	}
	return getCacheVariable<FiberVelocityInfo>(s, "velInfo");
}

Muscle::FiberVelocityInfo& Muscle::
updFiberVelocityInfo(const SimTK::State& s) const
{
	return updCacheVariable<FiberVelocityInfo>(s, "velInfo");
}

const Muscle::MuscleDynamicsInfo& Muscle::
getMuscleDynamicsInfo(const SimTK::State& s) const
{
	if(!isCacheVariableValid(s,"dynamicsInfo")){
		MuscleDynamicsInfo& umdi = updMuscleDynamicsInfo(s);
		calcMuscleDynamicsInfo(s, umdi);
		markCacheVariableValid(s,"dynamicsInfo");
		// don't bother fishing it out of the cache since 
		// we just calculated it and still have a handle on it
		return umdi;
	}
	return getCacheVariable<MuscleDynamicsInfo>(s, "dynamicsInfo");
}
Muscle::MuscleDynamicsInfo& Muscle::
updMuscleDynamicsInfo(const SimTK::State& s) const
{
	return updCacheVariable<MuscleDynamicsInfo>(s, "dynamicsInfo");
}

//_____________________________________________________________________________
/**
 * Get the stress in this muscle actuator.  It is calculated as the force 
 * divided by the maximum isometric force (which is proportional to its area).
 */
double Muscle::getStress(const SimTK::State& s) const
{
	return getForce(s) / getMaxIsometricForce();
}


//=============================================================================
// CALCULATIONS
//=============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
	normalized lengths, pennation angle, etc... */
void Muscle::calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{
	throw Exception("ERROR- "+getConcreteClassName()
        + "::calcMuscleLengthInfo() NOT IMPLEMENTED.");
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
	normalized velocities, pennation angular velocity, etc... */
void Muscle::calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
	throw Exception("ERROR- "+getConcreteClassName()
        + "::calcFiberVelocityInfo() NOT IMPLEMENTED.");
}

/* calculate muscle's active and passive force-length, force-velocity, 
	tendon force, relationships and their related values */
void Muscle::calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
	throw Exception("ERROR- "+getConcreteClassName()
        + "::calcMuscleDynamicsInfo() NOT IMPLEMENTED.");
}

//=============================================================================
// FORCE APPLICATION
//=============================================================================
//_____________________________________________________________________________
/**
 * Apply the muscle's force at its points of attachment to the bodies.
 */
void Muscle::computeForce(const SimTK::State& s, 
						  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
						  SimTK::Vector& generalizedForces) const
{
	Super::computeForce(s, bodyForces, generalizedForces); // Calls compute actuation.

	// NOTE: Force could be negative, in particular during CMC, when the optimizer is computing
	// gradients, it will setForce(+1) and setForce(-1) to compute the derivative with respect to force.
	if (getForce(s) < -SimTK::SqrtEps) {
		string msg = getConcreteClassName()
            + "::computeForce, muscle force < 0 for muscle '" 
            + getName() +"' ";
		cout << msg << " at time = " << s.getTime() << endl;
		//throw Exception(msg);
    }
}

double Muscle::computePotentialEnergy(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	return mli.musclePotentialEnergy;
}


void Muscle::updateGeometry(const SimTK::State& s)
{
	updGeometryPath().updateGeometry(s);
}


//_____________________________________________________________________________
/**
 * Utility function to calculate the current pennation angle in a
 * muscle. Pennation angle increases as muscle fibers shorten. The implicit
 * modeling assumption is that muscles have constant width.
 *
 * @param s State
 * @return Current pennation angle (in radians).
 */
/*
 double Muscle::calcPennationAngle(const SimTK::State &s) const
{
	double aFiberLength = getFiberLength(s);
	if (aFiberLength < SimTK::Eps){
		cout << "Muscle::calcPennationAngle() ERRROR- fiber length is zero." << endl;
		return SimTK::NaN;
	}
	
	double value = _muscleWidth/aFiberLength;

	if(value >= 1.0){
		cout << "Muscle::calcPennationAngle() ERRROR- pennation at 90 degrees." << endl;
		return SimTK_PI/2.0;
	}
   else
      return asin(value);
}
*/