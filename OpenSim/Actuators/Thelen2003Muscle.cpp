// Thelen2003Muscle.cpp
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
#include "Thelen2003Muscle.h"
#include <OpenSim/Common/SimmMacros.h>
#include <OpenSim/Common/DebugUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
Thelen2003Muscle::Thelen2003Muscle() :
   ActivationFiberLengthMuscle()
{
	setNull();
	setupProperties();
}

//_____________________________________________________________________________
/**
 * Constructor.
 */
Thelen2003Muscle::Thelen2003Muscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle) :
   ActivationFiberLengthMuscle()
{
	setNull();
	setupProperties();
	setName(aName);
	setMaxIsometricForce(aMaxIsometricForce);
	setOptimalFiberLength(aOptimalFiberLength);
	setTendonSlackLength(aTendonSlackLength);
	setPennationAngleAtOptimalFiberLength(aPennationAngle);
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
Thelen2003Muscle::~Thelen2003Muscle()
{
}

//_____________________________________________________________________________
/**
 * Copy constructor.
 *
 * @param aMuscle Thelen2003Muscle to be copied.
 */
Thelen2003Muscle::Thelen2003Muscle(const Thelen2003Muscle &aMuscle) :
   ActivationFiberLengthMuscle(aMuscle)
{
	setNull();
	setupProperties();
	copyData(aMuscle);
}

//_____________________________________________________________________________
/**
 * Copy this muscle point and return a pointer to the copy.
 * The copy constructor for this class is used.
 *
 * @return Pointer to a copy of this Thelen2003Muscle.
 */
Object* Thelen2003Muscle::copy() const
{
	Thelen2003Muscle *musc = new Thelen2003Muscle(*this);
	return(musc);
}

//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================
//_____________________________________________________________________________
/**
 * Copy data members from one Thelen2003Muscle to another.
 *
 * @param aMuscle Thelen2003Muscle to be copied.
 */
void Thelen2003Muscle::copyData(const Thelen2003Muscle &aMuscle)
{
	setPropertyValue("activation_time_constant", aMuscle.getPropertyValue<double>("activation_time_constant"));
	setPropertyValue("deactivation_time_constant", aMuscle.getPropertyValue<double>("deactivation_time_constant"));
	setPropertyValue("Vmax", aMuscle.getPropertyValue<double>("Vmax"));
	setPropertyValue("Vmax0", aMuscle.getPropertyValue<double>("Vmax0"));
	setPropertyValue("FmaxTendonStrain", aMuscle.getPropertyValue<double>("FmaxTendonStrain"));
	setPropertyValue("FmaxMuscleStrain", aMuscle.getPropertyValue<double>("FmaxMuscleStrain"));
	setPropertyValue("KshapeActive", aMuscle.getPropertyValue<double>("KshapeActive"));
	setPropertyValue("KshapePassive", aMuscle.getPropertyValue<double>("KshapePassive"));
	setPropertyValue("damping", aMuscle.getPropertyValue<double>("damping"));
	setPropertyValue("Af", aMuscle.getPropertyValue<double>("Af"));
	setPropertyValue("Flen", aMuscle.getPropertyValue<double>("Flen"));
}

//_____________________________________________________________________________
/**
 * Set the data members of this Thelen2003Muscle to their null values.
 */
void Thelen2003Muscle::setNull()
{
	setType("Thelen2003Muscle");
}

//_____________________________________________________________________________
/**
 * Connect properties to local pointers.
 */
void Thelen2003Muscle::setupProperties()
{
	addProperty<double>("activation_time_constant",
		"time constant for ramping up of muscle activation",
		0.01);
	addProperty<double>("deactivation_time_constant",
		"time constant for ramping down of muscle activation",
		0.04);
	addProperty<double>("Vmax",
		"maximum contraction velocity at full activation in fiber lengths per second",
		10.0);
	addProperty<double>("Vmax0",
		"maximum contraction velocity at low activation in fiber lengths per second",
		5.0);
	addProperty<double>("FmaxTendonStrain",
		"tendon strain due to maximum isometric muscle force",
		0.033);
	addProperty<double>("FmaxMuscleStrain",
		"passive muscle strain due to maximum isometric muscle force",
		0.6);
	addProperty<double>("KshapeActive",
		"shape factor for Gaussian active muscle force-length relationship",
		0.5);
	addProperty<double>("KshapePassive",
		"exponential shape factor for passive force-length relationship",
		4.0);
	addProperty<double>("damping",
		"passive damping in the force-velocity relationship",
		0.05);
	addProperty<double>("Af",
		"force-velocity shape factor",
		0.3);
	addProperty<double>("Flen",
		"maximum normalized lengthening force",
		1.8);
}

//=============================================================================
// OPERATORS
//=============================================================================
//_____________________________________________________________________________
/**
 * Assignment operator.
 *
 * @return Reference to this object.
 */
Thelen2003Muscle& Thelen2003Muscle::operator=(const Thelen2003Muscle &aMuscle)
{
	// BASE CLASS
	ActivationFiberLengthMuscle::operator=(aMuscle);
	copyData(aMuscle);

	return(*this);
}


//=============================================================================
// GET
//=============================================================================

//-----------------------------------------------------------------------------
// ACTIVATION TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant for ramping up of muscle force.
 *
 * @param aActivationTimeConstant The time constant for ramping up of muscle force.
 * @return Whether the time constant was successfully changed.
 */
void Thelen2003Muscle::setActivationTimeConstant(double aActivationTimeConstant)
{
	setPropertyValue("activation_time_constant", aActivationTimeConstant);
}

//-----------------------------------------------------------------------------
// DEACTIVATION TIME
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the time constant for ramping down of muscle force.
 *
 * @param aDeactivationTimeConstant The time constant for ramping down of muscle force.
 * @return Whether the time constant was successfully changed.
 */
void Thelen2003Muscle::setDeactivationTimeConstant(double aDeactivationTimeConstant)
{
	setPropertyValue("deactivation_time_constant", aDeactivationTimeConstant);
}


//-----------------------------------------------------------------------------
// MAX CONTRACTION VELOCITY
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum contraction velocity of the fibers, in optimal fiber lengths per second.
 *
 * @param aVmax The maximum contraction velocity of the fibers, in optimal fiber lengths per second.
 * @return Whether the maximum contraction velocity was successfully changed.
 */
void Thelen2003Muscle::setVmax(double aVmax)
{
	setPropertyValue("Vmax", aVmax);
}

//-----------------------------------------------------------------------------
// MAX CONTRACTION VELOCITY AT LOW ACTIVATION
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum contraction velocity at low activation of the fibers, in optimal fiber lengths per second.
 *
 * @param aVmax The maximum contraction velocity at low activation of the fibers, in optimal fiber lengths per second.
 * @return Whether the maximum contraction velocity was successfully changed.
 */
void Thelen2003Muscle::setVmax0(double aVmax0)
{
	setPropertyValue("Vmax0", aVmax0);
}

//-----------------------------------------------------------------------------
// TENDON STRAIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the tendon strain due to maximum isometric muscle force.
 *
 * @param aFmaxTendonStrain The tendon strain due to maximum isometric muscle force.
 * @return Whether the tendon strain was successfully changed.
 */
void Thelen2003Muscle::setFmaxTendonStrain(double aFmaxTendonStrain)
{
	setPropertyValue("FmaxTendonStrain", aFmaxTendonStrain);
}

//-----------------------------------------------------------------------------
// MUSCLE STRAIN
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the passive muscle strain due to maximum isometric muscle force.
 *
 * @param aFmaxMuscleStrain The passive muscle strain due to maximum isometric muscle force.
 * @return Whether the passive muscle strain was successfully changed.
 */
void Thelen2003Muscle::setFmaxMuscleStrain(double aFmaxMuscleStrain)
{
	setPropertyValue("FmaxMuscleStrain", aFmaxMuscleStrain);
}

//-----------------------------------------------------------------------------
// SHAPE FACTOR ACTIVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the shape factor for Gaussian active muscle force-length relationship.
 *
 * @param aKShapeActive The shape factor for Gaussian active muscle force-length relationship.
 * @return Whether the shape factor was successfully changed.
 */
void Thelen2003Muscle::setKshapeActive(double aKShapeActive)
{
	setPropertyValue("KshapeActive", aKShapeActive);
}

//-----------------------------------------------------------------------------
// SHAPE FACTOR PASSIVE
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the shape factor for Gaussian passive muscle force-length relationship.
 *
 * @param aKshapePassive The shape factor for Gaussian passive muscle force-length relationship.
 * @return Whether the shape factor was successfully changed.
 */
void Thelen2003Muscle::setKshapePassive(double aKshapePassive)
{
	setPropertyValue("KshapePassive", aKshapePassive);
}

//-----------------------------------------------------------------------------
// DAMPING
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the damping factor related to maximum contraction velocity.
 *
 * @param aDamping The damping factor related to maximum contraction velocity.
 * @return Whether the damping factor was successfully changed.
 */
void Thelen2003Muscle::setDamping(double aDamping)
{
	setPropertyValue("damping", aDamping);
}

//-----------------------------------------------------------------------------
// FORCE-VELOCITY SHAPE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the force-velocity shape factor.
 *
 * @param aAf The force-velocity shape factor.
 * @return Whether the shape factor was successfully changed.
 */
void Thelen2003Muscle::setAf(double aAf)
{
	setPropertyValue("Af", aAf);
}

//-----------------------------------------------------------------------------
// FORCE-VELOCITY SHAPE FACTOR
//-----------------------------------------------------------------------------
//_____________________________________________________________________________
/**
 * Set the maximum normalized lengthening force.
 *
 * @param aFlen The maximum normalized lengthening force.
 * @return Whether the maximum normalized lengthening force was successfully changed.
 */
void Thelen2003Muscle::setFlen(double aFlen)
{
	setPropertyValue("Flen", aFlen);
}

//=============================================================================
// COMPUTATION
//=============================================================================
double Thelen2003Muscle::computeActuation(const SimTK::State& s) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);

	double tendonForce = getMaxIsometricForce()*mdi.normTendonForce;
	setForce(s, tendonForce);

	return tendonForce;
}


//=============================================================================
// CALCULATIONS
//=============================================================================
/* calculate muscle's position related values such fiber and tendon lengths,
	normalized lengths, pennation angle, etc... */
void Thelen2003Muscle::calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const
{
	double norm_muscle_tendon_length = getLength(s) / getOptimalFiberLength();
	
	mli.fiberLength = getStateVariable(s, STATE_FIBER_LENGTH_NAME);
	
	if (mli.fiberLength < SimTK::Eps)
		mli.pennationAngle = 0.0;
	else{
		double value = getOptimalFiberLength() * sin(getPennationAngleAtOptimalFiberLength()) / mli.fiberLength;
		if ( isnan(value)  ) 
			mli.pennationAngle = 0.0;
		else if (value <= 0.0 )
			mli.pennationAngle = 0.0;
		else if (value >= 1.0)
			mli.pennationAngle = SimTK_PI/2.0;
		else
			mli.pennationAngle = asin(value);
	}

	mli.cosPennationAngle = cos(mli.pennationAngle);
	mli.tendonLength = getLength(s)-mli.fiberLength*mli.cosPennationAngle;
	
	mli.normFiberLength = mli.fiberLength/getOptimalFiberLength();
	mli.normTendonLength = norm_muscle_tendon_length - mli.normFiberLength * mli.cosPennationAngle;
	mli.tendonStrain = (mli.tendonLength/getTendonSlackLength()-1.0);

	mli.forceLengthMultiplier = calcActiveForce(s, mli.normFiberLength);
	mli.passiveForceMultiplier = calcPassiveForce(s, mli.normFiberLength);
}

/* calculate muscle's velocity related values such fiber and tendon velocities,
	normalized velocities, pennation angular velocity, etc... */
void Thelen2003Muscle::calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const
{
	const MuscleLengthInfo& mli = getMuscleLengthInfo(s);
	const MuscleDynamicsInfo& mdi = getMuscleDynamicsInfo(s);

	const double &optimalFiberLength = getOptimalFiberLength();

	// Maximum contraction velocity is an activation scaled value
	double Vmax = getPropertyValue<double>("Vmax");
	const double &vmax0 = getPropertyValue<double>("Vmax0");
	if (mdi.activation < 1.0) {
		Vmax = vmax0 + mdi.activation*(Vmax-vmax0);
	}
	Vmax = Vmax*optimalFiberLength;

	/* If pennation equals 90 degrees, fiber length equals muscle width and fiber
	* velocity goes to zero.  Pennation will stay at 90 until tendon starts to
	* pull, then "stiff tendon" approximation is used to calculate approximate
	* fiber velocity.
	*/
	if (abs(mli.cosPennationAngle) < SimTK::SqrtEps) {
		if (abs(mdi.normTendonForce) < SimTK::SqrtEps) {
			fvi.normFiberVelocity = fvi.fiberVelocity = 0.0;
		} else { // assume rigid tendon
			fvi.fiberVelocity = getLengtheningSpeed(s);
			fvi.normFiberVelocity = fvi.fiberVelocity/(optimalFiberLength*getMaxContractionVelocity());
		}
	} else {
		double velocity_dependent_force = mdi.normTendonForce / mli.cosPennationAngle - mli.passiveForceMultiplier;
		fvi.normFiberVelocity = calcFiberVelocity(s, mdi.activation, mli.forceLengthMultiplier, velocity_dependent_force);
		fvi.fiberVelocity = fvi.normFiberVelocity * Vmax;
	}

	fvi.forceVelocityMultiplier = mdi.activeFiberForce/(getMaxIsometricForce() *mdi.activation*mli.forceLengthMultiplier);

}

/* calculate muscle's active and passive force-length, force-velocity, 
	tendon force, relationships and their related values */
void Thelen2003Muscle::calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const
{
	const MuscleLengthInfo &mli = getMuscleLengthInfo(s);
	const double &maxIsometricForce = getMaxIsometricForce();

	mdi.normTendonForce = calcTendonForce(s, mli.normTendonLength);
	
	mdi.passiveFiberForce = mli.passiveForceMultiplier * maxIsometricForce;
	
	
	
	mdi.activation = getStateVariable(s, STATE_ACTIVATION_NAME);

	double tendonForce = maxIsometricForce*mdi.normTendonForce;
	mdi.activeFiberForce =  tendonForce/mli.cosPennationAngle - mdi.passiveFiberForce;

}

/* Calculate muscle's activation rate (derivative) that will be integrated */
double Thelen2003Muscle::calcActivationRate(const SimTK::State& s) const
{
	double excitation = getExcitation(s);
	double activation = getActivation(s);

	if (excitation >= activation) {
		return (excitation - activation) / getActivationTimeConstant();
	} else {
		return (excitation - activation) / getDeactivationTimeConstant();
	}
}

//_____________________________________________________________________________
/**
 * From cmg_dt.c - calc_tendon_force_dt
 *
 * CALC_TENDON_FORCE_DT: this routine calculates the force in tendon by finding
 * tendon strain and using it in an exponential function (JBME 2003 - Thelen)
 * FmaxTendonStrain - Function is parameterized by the tendon strain due to maximum isometric muscle force
 *     This should be specified as a dynamic parameter in the muscle file
 *
 * @param aNormTendonLength Normalized length of the tendon.
 * @return The force in the tendon.
 */
double Thelen2003Muscle::calcTendonForce(const SimTK::State& s, double aNormTendonLength) const
{
	const double &optimalFiberLength = getPropertyValue<double>("optimal_fiber_length");
	const double &tendonSlackLength = getPropertyValue<double>("tendon_slack_length");
	const double &fmaxTendonStrain = getPropertyValue<double>("FmaxTendonStrain");

	double norm_resting_length = tendonSlackLength / optimalFiberLength;
	double tendon_strain =  (aNormTendonLength - norm_resting_length) / norm_resting_length;

	double KToe = 3;
	double ToeStrain = 0.609*fmaxTendonStrain;
	double ToeForce = 0.333333;
	double klin = 1.712/fmaxTendonStrain;

	double tendon_force;
	if (tendon_strain>ToeStrain)
		tendon_force = klin*(tendon_strain-ToeStrain)+ToeForce;
	else if (tendon_strain>0) 
		tendon_force = ToeForce*(exp(KToe*tendon_strain/ToeStrain)-1.0)/(exp(KToe)-1);
	else
		tendon_force=0.;

	// Add on a small stiffness so that tendon never truly goes slack for non-zero tendon lengths
	tendon_force+=0.001*(1.+tendon_strain);

	return tendon_force;
}

//_____________________________________________________________________________
/**
 * From gmc.dt.c - calc_passive_fiber_force_dt
 *
 * CALC_PASSIVE_FIBER_FORCE_DT: written by Darryl Thelen
 * this routine calculates the passive force in the muscle fibers using
 * an exponential-linear function instead of cubic splines.
 * It always returns a non-zero force for all muscle lengths
 * This equation is parameterized using the following dynamic parameters
 * which must be specified in the muscle file
 * Dynamic Parameters:
 *   FmaxMuscleStrain - passive muscle strain due to the application of 
 *                      maximum isometric muscle force
 *	 KshapePassive - exponential shape factor
 *
 *  The normalized force due to passive stretch is given by
 *  For L < (1+maxStrain)*Lo
 *		f/f0 = exp(ks*(L-1)/maxStrain) / exp(ks)
 *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The passive force in the muscle fibers.
 */
double Thelen2003Muscle::calcPassiveForce(const SimTK::State& s, double aNormFiberLength) const
{
	double passive_force;

	const double &fmaxMuscleStrain = getPropertyValue<double>("FmaxMuscleStrain");
	const double &kShapePassive = getPropertyValue<double>("KshapePassive");

	if (aNormFiberLength>(1+fmaxMuscleStrain)) { // Switch to a linear model at large forces
		double slope=(kShapePassive/fmaxMuscleStrain)*(exp(kShapePassive*(1.0+fmaxMuscleStrain-1.0)/fmaxMuscleStrain)) / (exp(kShapePassive));
		passive_force=1.0+slope*(aNormFiberLength-(1.0+fmaxMuscleStrain));
	}
	else
		passive_force = (exp(kShapePassive*(aNormFiberLength-1.0)/fmaxMuscleStrain)) / (exp(kShapePassive));

	return passive_force;
}

//_____________________________________________________________________________
/**
 * From gmc.dt.c - calc_active_force_dt
 *
 * CALC_ACTIVE_FORCE_DT: this routine calculates the active component of force
 * in the muscle fibers. It uses the current fiber length to interpolate the
 * active force-length curve - described by Gaussian curve as in Thelen, JBME 2003
 * *
 * @param aNormFiberLength Normalized length of the muscle fiber.
 * @return The active force in the muscle fibers.
 */
double Thelen2003Muscle::calcActiveForce(const SimTK::State& s, double aNormFiberLength) const
{
	double x=-(aNormFiberLength-1.)*(aNormFiberLength-1.)/getPropertyValue<double>("KshapeActive");
	return exp(x);
}

//_____________________________________________________________________________
/**
 * From gmc_dt.c - calc_norm_fiber_velocity_dt
 *
 * CALC_NORM_FIBER_VELOCITY_DT: written by Darryl Thelen
 * this routine calculates the normalized fiber velocity (scaled to Vmax) by inverting the
 * muscle force-velocity-activation relationship (Thelen, JBME 2003)
 * This equation is parameterized using the following dynamic parameters
 * which must be specified in the muscle file
 * Dynamic Parameters:
 *   damping - normalized passive damping in parallel with contractile element
 *   Af - velocity shape factor from Hill's equation
 *   Flen	- Maximum normalized force when muscle is lengthening
 *
 * @param aActivation Activation of the muscle.
 * @param aActiveForce Active force in the muscle fibers.
 * @param aVelocityDependentForce Force value that depends on fiber velocity.
 * @return The velocity of the muscle fibers.
 */
double Thelen2003Muscle::calcFiberVelocity(const SimTK::State& s, double aActivation, double aActiveForce, double aVelocityDependentForce) const
{
	double epsilon=1.e-6;

	// Don't allow zero activation
	if (aActivation<epsilon) 
		aActivation=epsilon;

	double Fa = aActivation*aActiveForce;
	double Fv = aVelocityDependentForce;

	const double &damping = getPropertyValue<double>("damping");
	const double &af = getPropertyValue<double>("Af");
	const double &flen = getPropertyValue<double>("Flen");

	double norm_fiber_velocity;
	if (Fv<Fa) {		// Muscle shortening
		if (Fv<0) {	// Extend the force-velocity curve for negative forces using linear extrapolation
			double F0=0;
			double b=Fa+F0/af;
        	double fv0 = (F0-Fa)/(b+damping);
			double F1=epsilon;
			b=Fa+F1/af;
        	double fv1 = (F1-Fa)/(b+damping);
			b = (F1-F0)/(fv1-fv0);
        	norm_fiber_velocity = fv0 + (Fv-F0)/b;
		}
		else {
			double b=Fa+Fv/af;
			norm_fiber_velocity = (Fv-Fa)/(b+damping);
		}
	}
	else if (Fv<(.95*Fa*flen)) {
		double b=(2+2./af)*(Fa*flen-Fv)/(flen-1.);
		norm_fiber_velocity = (Fv-Fa)/(b+damping);
	}
	else {  // Extend the force-velocity curve for forces that exceed maximum using linear extrapolation
		double F0=.95*Fa*flen;
		double b=(2+2./af)*(Fa*flen-F0)/(flen-1.);
		double fv0 = (F0-Fa)/(b+damping);
		double F1=(.95+epsilon)*Fa*flen;
		b=(2+2./af)*(Fa*flen-F1)/(flen-1.);
		double fv1 = (F1-Fa)/(b+damping);
		b = (fv1-fv0)/(F1-F0);
		norm_fiber_velocity = fv0 + b*(Fv-F0);
	}

	return norm_fiber_velocity;
}

//_____________________________________________________________________________
/**
 * Find the force produced by an actuator (the musculotendon unit), assuming
 * static equilibrium. Using the total muscle-tendon length, it finds the
 * fiber and tendon lengths so that the forces in each match. This routine
 * takes pennation angle into account, so its definition of static equilibrium
 * is when tendon_force = fiber_force * cos(pennation_angle). This funcion
 * will modify the object's values for length, fiberLength, activeForce, 
 * and passiveForce.
 *
 * @param aActivation Activation of the muscle.
 * @return The isometric force in the muscle.
 */
double Thelen2003Muscle::computeIsometricForce(SimTK::State& s, double aActivation) const
{
#define MAX_ITERATIONS 100
#define ERROR_LIMIT 0.01

    int i;
    double length,tendon_length, fiber_force, tmp_fiber_length, min_tendon_stiffness;
    double cos_factor, fiber_stiffness;
    double old_fiber_length, length_change, tendon_stiffness, percent;
    double error_force = 0.0, old_error_force, tendon_force, norm_tendon_length;
    double passiveForce;
    double fiberLength;

    const double &maxIsometricForce = getPropertyValue<double>("max_isometric_force");
    const double &optimalFiberLength = getPropertyValue<double>("optimal_fiber_length");
	const double &tendonSlackLength = getPropertyValue<double>("tendon_slack_length");
	const double &pennationAngleAtOptimal = getPropertyValue<double>("pennation_angle_at_optimal");

    if (optimalFiberLength < ROUNDOFF_ERROR) {
       setStateVariable(s, STATE_FIBER_LENGTH_NAME, 0.0);
       setForce(s, 0.0);
       return 0.0;
    }

	length = getLength(s);

   // Make first guess of fiber and tendon lengths. Make fiber length equal to
   // optimal_fiber_length so that you start in the middle of the active+passive
   // force-length curve. Muscle_width is the width, or thickness, of the
   // muscle-tendon unit. It is the shortest allowable fiber length because if
   // the muscle-tendon length is very short, the pennation angle will be 90
   // degrees and the fibers will be vertical (assuming the tendon is horizontal).
   // When this happens, the fibers are as long as the muscle is wide.
   // If the resting tendon length is zero, then set the fiber length equal to
   // the muscle tendon length / cosine_factor, and find its force directly.

   double muscle_width = _muscleWidth;

	if (tendonSlackLength < ROUNDOFF_ERROR) {
		tendon_length = 0.0;
		cos_factor = cos(atan(muscle_width / length));
		fiberLength = length / cos_factor;

		double activeForce = calcActiveForce(s, fiberLength / optimalFiberLength)  * aActivation;
		if (activeForce < 0.0) activeForce = 0.0;

		passiveForce = calcPassiveForce(s, fiberLength / optimalFiberLength);
		if (passiveForce < 0.0) passiveForce = 0.0;

		setStateVariable(s, STATE_FIBER_LENGTH_NAME, fiberLength);
		tendon_force = (activeForce + passiveForce) * maxIsometricForce * cos_factor;
		setForce(s, tendon_force);
		return tendon_force;
   } else if (length < tendonSlackLength) {
      setStateVariable(s, STATE_FIBER_LENGTH_NAME, muscle_width);
      _model->getMultibodySystem().realize(s, SimTK::Stage::Velocity);
      setForce(s, 0.0);
      return 0.0;
   } else {
      fiberLength = optimalFiberLength;
      tendon_length = length - sqrt(fiberLength*fiberLength-_muscleWidth*_muscleWidth);
	  cos_factor = cos(atan(muscle_width / fiberLength));

      /* Check to make sure tendon is not shorter than its slack length. If it
       * is, set the length to its slack length and re-compute fiber length.
       */

      if (tendon_length < tendonSlackLength) {
         tendon_length = tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor;
         if (fiberLength < muscle_width)
           fiberLength = muscle_width;
      }

   }

   // Muscle-tendon force is found using an iterative method. First, you guess
   // the length of the muscle fibers and the length of the tendon, and
   // calculate their respective forces. If the forces match (are within
   // ERROR_LIMIT of each other), stop; else change the length guesses based
   // on the error and try again.
   for (i = 0; i < MAX_ITERATIONS; i++) {
		double activeForce = calcActiveForce(s, fiberLength/ optimalFiberLength) * aActivation;
      if( activeForce <  0.0) activeForce = 0.0;

      passiveForce =  calcPassiveForce(s, fiberLength / optimalFiberLength);
      if (passiveForce < 0.0) passiveForce = 0.0;

      fiber_force = (activeForce + passiveForce) * maxIsometricForce * cos_factor;

      norm_tendon_length = tendon_length / optimalFiberLength;
      tendon_force = calcTendonForce(s, norm_tendon_length) * maxIsometricForce;
		setForce(s, tendon_force);

      old_error_force = error_force;
 
      error_force = tendon_force - fiber_force;

      if (DABS(error_force) <= ERROR_LIMIT) // muscle-tendon force found!
         break;

      if (i == 0)
         old_error_force = error_force;

      if (DSIGN(error_force) != DSIGN(old_error_force)) {
         percent = DABS(error_force) / (DABS(error_force) + DABS(old_error_force));
         tmp_fiber_length = old_fiber_length;
         old_fiber_length = fiberLength;
         fiberLength = fiberLength + percent * (tmp_fiber_length - fiberLength);
      } else {
         // Estimate the stiffnesses of the tendon and the fibers. If tendon
         // stiffness is too low, then the next length guess will overshoot
         // the equilibrium point. So we artificially raise it using the
         // normalized muscle force. (_activeForce+_passiveForce) is the
         // normalized force for the current fiber length, and we assume that
         // the equilibrium length is close to this current length. So we want
         // to get force = (_activeForce+_passiveForce) from the tendon as well.
         // We hope this will happen by setting the tendon stiffness to
         // (_activeForce+_passiveForce) times its maximum stiffness.
			double tendon_elastic_modulus = 1200.0;
			double tendon_max_stress = 32.0;

         tendon_stiffness = calcTendonForce(s, norm_tendon_length) *
				maxIsometricForce / tendonSlackLength;

         min_tendon_stiffness = (activeForce + passiveForce) *
	         tendon_elastic_modulus * maxIsometricForce /
	         (tendon_max_stress * tendonSlackLength);

         if (tendon_stiffness < min_tendon_stiffness)
            tendon_stiffness = min_tendon_stiffness;

         fiber_stiffness = maxIsometricForce / optimalFiberLength *
            (calcActiveForce(s, fiberLength / optimalFiberLength)  +
            calcPassiveForce(s, fiberLength / optimalFiberLength));

         // determine how much the fiber and tendon lengths have to
         // change to make the error_force zero. But don't let the
	      // length change exceed half the optimal fiber length because
	      // that's too big a change to make all at once.
         length_change = fabs(error_force/(fiber_stiffness / cos_factor + tendon_stiffness));

         if (fabs(length_change / optimalFiberLength) > 0.5)
            length_change = 0.5 * optimalFiberLength;

         // now change the fiber length depending on the sign of the error
         // and the sign of the fiber stiffness (which equals the sign of
         // the slope of the muscle's force-length curve).
         old_fiber_length = fiberLength;

         if (error_force > 0.0)
             fiberLength += length_change;
         else
             fiberLength -= length_change;


      }
      tendon_length = length - sqrt(fiberLength*fiberLength-_muscleWidth*_muscleWidth);

      // Check to make sure tendon is not shorter than its slack length. If it is,
      // set the length to its slack length and re-compute fiber length.
      if (tendon_length < tendonSlackLength) {
         tendon_length = tendonSlackLength;
         cos_factor = cos(atan(muscle_width / (length - tendon_length)));
         fiberLength = (length - tendon_length) / cos_factor ;
      }
	}

	_model->getMultibodySystem().realize(s, SimTK::Stage::Position);
	setStateVariable(s, STATE_FIBER_LENGTH_NAME,  fiberLength);

//cout << "ThelenMuscle computeIsometricForce " << getName() << "  t=" << s.getTime() << " force = " << tendon_force << endl;

   return tendon_force;
}