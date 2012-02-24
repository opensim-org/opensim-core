#ifndef __Muscle_h__
#define __Muscle_h__

// Muscle.h
// Author: Peter Loan, Frank C. Anderson, Ajay Seth
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


// INCLUDE
#include "PathActuator.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A base class for modeling a muscle-tendon actuator. It defines muscle parameters
 * and methods to PathActuator, but does not implement all of the necessary methods,
 * and remains an abstract class. The path information for a muscle is contained
 * in PathActuator, and the force-generating behavior should be defined in
 * the derived classes.
 *
 * This class defines a subset of muscle models that include an active fiber
 * (contractile element) in series with a tendon. This class defines common 
 * data members and handles the geometry of a unipennate fiber in connection
 * with a tendon. No states are assumed, but concrete classes are free to
 * add whatever states are necessary to describe the specific behavior of a
 * muscle.
 *
 * @version 2.0
 * @author Ajay Seth
 *
 * @version 1.0
 * @author Peter Loan
 * @author Frank C. Anderson
 * 
 */
class OSIMSIMULATION_API Muscle : public PathActuator  
{

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// MUSCLE PARAMETER ACCESSORS
	//--------------------------------------------------------------------------
	/** @name Muscle Parameters Access Methods
	 */ 
	//@{
	/** get/set the maximum isometric force (in N) that the fibers can generate */
	double getMaxIsometricForce() const; 
	void setMaxIsometricForce(double aMaxIsometricForce);

	/** get/set the optimal length (in m) of the muscle fibers (lumped as a single fiber) */
	double getOptimalFiberLength() const;
	void setOptimalFiberLength(double aOptimalFiberLength);

	/** get/set the resting (slack) length (in m) of the tendon that is in series with the muscle fiber */
	double getTendonSlackLength() const;
	void setTendonSlackLength(double aTendonSlackLength);

	/** get/set the angle (in radians) between fibers at their optimal fiber length and the tendon */
	double getPennationAngleAtOptimalFiberLength() const;
	void setPennationAngleAtOptimalFiberLength(double aPennationAngle);
	
	/** get/set the maximum contraction velocity of the fibers, in optimal fiber-lengths per second */
	double getMaxContractionVelocity() const;
	void setMaxContractionVelocity(double aMaxContractionVelocity);
	// End of Muscle Parameter Accessors.
    //@} 

	//--------------------------------------------------------------------------
	// MUSCLE STATE DEPENDENT ACCESSORS
	//--------------------------------------------------------------------------
	/** @name Muscle State Dependent Access Methods
	 *  Get quantities of interest common to all muscles
	 */ 
	//@{
	/** get the activation level of the muscle, which modulates the active force of the muscle 
	    and has a normalized (0 to 1) value */
	double getActivation(const SimTK::State& s) const;

	/** get the current working fiber length (m) for the muscle */
	double getFiberLength(const SimTK::State& s) const;
	/** get the current pennation angle (radians) between the fiber and tendon at the current fiber length  */
	double getPennationAngle(const SimTK::State& s) const;
	/** get the current tendon length (m)  given the current joint angles and fiber length */
	double getTendonLength(const SimTK::State& s) const;
	/** get the current normalized fiber length (fiber_length/optimal_fiber_length) */
	double getNormalizedFiberLength(const SimTK::State& s) const;
	/** get the current fiber length (m) projected (*cos(pennationAngle)) onto the tendon direction */
	double getFiberLengthAlongTendon(const SimTK::State& s) const;
	/** get the current tendon strain (delta_l/tendon_slack_length is dimensionless)  */
	double getTendonStrain(const SimTK::State& s) const;

	/** get current fiber velocity (m/s) positive is lengthening */
	double getFiberVelocity(const SimTK::State& s) const;
	/** get normalize fiber velocity (fiber_length/s / max_contraction_velocity) */
	double getNormalizedFiberVelocity(const SimTK::State& s) const;
	/** get the current afiber velocity (m/s) projected onto the tendon direction */
	double getFiberVelocityAlongTendon(const SimTK::State& s) const;
	/** get pennation angular velocity (radians/s) */
	double getPennationAngularVelocity(const SimTK::State& s) const;

	/** get the current fiber force (N) applied to the tendon */
	double getFiberForce(const SimTK::State& s) const;
	/** get the current active fiber force (N) due to activation*force_length*force_velocity relationships */
	double getActiveFiberForce(const SimTK::State& s) const;
	/** get the current passive fiber force (N) passive_force_length relationship */
	double getPassiveFiberForce(const SimTK::State& s) const;
	/** get the current active fiber force (N) projected onto the tendon direction */
	double getActiveFiberForceAlongTendon(const SimTK::State& s) const;
	/** get the current passive fiber force (N) projected onto the tendon direction */
	double getPassiveFiberForceAlongTendon(const SimTK::State& s) const;
	/** get the current tendon force (N) applied to bones */
	double getTendonForce(const SimTK::State& s) const;
	/** get the stress in the muscle (part of the Actuator interface as well) */
	double getStress(const SimTK::State& s) const;
	
	/** set the excitation (control) for this muscle. NOTE if controllers are connected to the
	    muscle and are adding in their controls, and setExcitation is called after the model's
		computeControls(), then setExcitation will override the controller values. If called 
		before computeControls, then controller value(s) are added to the excitation set here. */
	void setExcitation(SimTK::State& s, double excitation) const;
	double getExcitation(const SimTK::State& s) const;


	/** TO BE DEPRECATED only for backward compatibility for now */
	virtual void setActivation(SimTK::State& s, double activation) const = 0;

	// End of Muscle's State Dependent Accessors.
    //@} 

	/** Actuator interface for a muscle computes the tension in the muscle
	    and applied by the tendon to bones (i.e. not the fiber force) */
	double computeActuation(const SimTK::State& s) const = 0;


	/** @name Muscle initialization 
	 */ 
	//@{
	/** Find and set the equilibrium state of the muscle (if any) */
	void equilibrate(SimTK::State& s) const {return computeInitialFiberEquilibrium(s); }
	// End of Muscle's State Dependent Accessors.
    //@} 


protected:
	struct MuscleLengthInfo;
	struct FiberVelocityInfo;
	struct MuscleDynamicsInfo;

	/** Developer Access to intermediate values computed by the model */
	const MuscleLengthInfo& getMuscleLengthInfo(const SimTK::State& s) const;
	MuscleLengthInfo& updMuscleLengthInfo(const SimTK::State& s) const;

	const FiberVelocityInfo& getFiberVelocityInfo(const SimTK::State& s) const;
	FiberVelocityInfo& updFiberVelocityInfo(const SimTK::State& s) const;

	const MuscleDynamicsInfo& getMuscleDynamicsInfo(const SimTK::State& s) const;
	MuscleDynamicsInfo& updMuscleDynamicsInfo(const SimTK::State& s) const;


	//--------------------------------------------------------------------------
	// CALCULATIONS
	//--------------------------------------------------------------------------
	/** @name Muscle State Dependent Calculations
	 *  Developers must override these method to implement the desired behavior
	 *  of their muscle models. Unless you are augmenting the behavior
	 *  of an existing muscle class or writing a new derived class, you do not
	 *  have access to these methods. 
	 */ 
	//@{
	/** calculate muscle's position related values such fiber and tendon lengths,
		normalized lengths, pennation angle, etc... */
	virtual void calcMuscleLengthInfo(const SimTK::State& s, MuscleLengthInfo& mli) const;

	/** calculate muscle's fiber velocity and pennation angular velocity, etc... */
	virtual void calcFiberVelocityInfo(const SimTK::State& s, FiberVelocityInfo& fvi) const;

	/** calculate muscle's active and passive force-length, force-velocity, 
	    tendon force, relationships and their related values */
	virtual void  calcMuscleDynamicsInfo(const SimTK::State& s, MuscleDynamicsInfo& mdi) const;

	/** compute initial fiber length (velocity) such that muscle fiber and tendon are 
	    in static equilibrium and update the state */
	virtual void computeInitialFiberEquilibrium(SimTK::State& s) const = 0;

	// End of Muscle's State Related Calculations.
    //@} 

	//--------------------------------------------------------------------------
	// PARENT INTERFACES
	//--------------------------------------------------------------------------
	/** @name Interfaces imposed by parent classes
	 */ 
	//@{

	/** Force interface applies tension to bodies, and Muscle also checks that 
	    applied muscle tension is not negative. */
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForce) const;
	

	/** Model Component creation interface */
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void setup(Model &aModel);
	
	/** Override behavior for adding state variables to automatically define
	    state variable derivatives as cache variables. */
	void addStateVariables(const Array<std::string> &stateVariableNames) const;

	// Update the geometry attached to the muscle (location of muscle points and connecting segments
	//  all in global/interial frame)
	virtual void updateGeometry(const SimTK::State& s);
	// End of Interfaces imposed by parent classes.
    //@} 

public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	/** @name Connstructors and Destructor
	 */ 
	//@{
	/** Default constructor */
	Muscle();
	/** Copy constructor */
	Muscle(const Muscle &aMuscle);
	/** Free memory used to generate display geometry */
	virtual ~Muscle();
	virtual Object* copy() const = 0;	// Needed by operator= and to put Actuators in Arrays
#ifndef SWIG
	Muscle& operator=(const Muscle &aMuscle);
#endif


	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);

	/** @name Old Junk to be deprecated
	 */ 
	//@{
	virtual double calcPennationAngle(const SimTK::State &s) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const = 0;
	// End of Old Junk.
    //@} 

	OPENSIM_DECLARE_DERIVED(Muscle, PathActuator);

private:
	void setNull();
	void setupProperties();
	void copyData(const Muscle &aMuscle);


//=============================================================================
// DATA
//=============================================================================
protected:

	/** The assumed fixed muscle-width from which the fiber pennation angle is calculated */
	double _muscleWidth;

	// Structure to hold muscle length dependent muscle quantities
	struct MuscleLengthInfo {
		double fiberLength;
		double normFiberLength;
		double tendonLength;
		double normTendonLength;
		double tendonStrain;
		double pennationAngle;
		double cosPennationAngle;
		MuscleLengthInfo(): fiberLength(SimTK::NaN), normFiberLength(SimTK::NaN),
			tendonLength(SimTK::NaN), normTendonLength(SimTK::NaN), tendonStrain(SimTK::NaN),
			pennationAngle(SimTK::NaN), cosPennationAngle(SimTK::NaN) {}
		friend std::ostream& operator<<(std::ostream& o, const MuscleLengthInfo& mli) {
			o << "Muscle::MuscleLengthInfo should not be serialized!" << std::endl;
			return o;
		}
    };

	// Structure to fiber velocity and related quantities
	struct FiberVelocityInfo {
		double fiberVelocity;
		double normFiberVelocity;
		double pennationAngularVelocity;
		FiberVelocityInfo(): fiberVelocity(SimTK::NaN), normFiberVelocity(SimTK::NaN),
			pennationAngularVelocity(SimTK::NaN) {};
		friend std::ostream& operator<<(std::ostream& o, const FiberVelocityInfo& fvi) {
			o << "Muscle::FiberVelocityInfo should not be serialized!" << std::endl;
			return o;
		}
    };

	// Structure to hold force dependent muscle quantities
	struct MuscleDynamicsInfo {
		double activation;
		double forceLengthMultiplier;
		double forceVelocityMultiplier;
		double passiveForceMultiplier;
		double activeFiberForce;
		double passiveFiberForce;
		double normTendonForce;
		double tendonStrainRate;
		double normTendonStrainRate;
		MuscleDynamicsInfo(): activation(SimTK::NaN), forceLengthMultiplier(SimTK::NaN), forceVelocityMultiplier(SimTK::NaN),
			passiveForceMultiplier(SimTK::NaN), activeFiberForce(SimTK::NaN),
			passiveFiberForce(SimTK::NaN), normTendonForce(SimTK::NaN), 
			tendonStrainRate(SimTK::NaN), normTendonStrainRate(SimTK::NaN) {};
		friend std::ostream& operator<<(std::ostream& o, const MuscleDynamicsInfo& mdi) {
			o << "Muscle::MuscleDynamicsInfo should not be serialized!" << std::endl;
			return o;
		}
	};

	/** to support deprecated muscles */
	double _maxIsometricForce;
	double _optimalFiberLength;
	double _pennationAngleAtOptimal;
	double _tendonSlackLength;

//=============================================================================
};	// END of class Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Muscle_h__