#ifndef __Muscle_h__
#define __Muscle_h__

// Muscle.h
// Author: Peter Loan, Frank C. Anderson, Ajay Seth
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


// INCLUDE
#include "PathActuator.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Coordinate;

//=============================================================================
//=============================================================================
/**
 * A base class representing a muscle-tendon actuator. It adds data and methods
 * to PathActuator, but does not implement all of the necessary methods,
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
// DATA
//=============================================================================
protected:

	/** Optimal length of the muscle fibers */
	PropertyDbl _optimalFiberLengthProp;
	double &_optimalFiberLength;

	/** Maximum isometric force that the fibers can generate */
	PropertyDbl _maxIsometricForceProp;
	double &_maxIsometricForce;

	/** Resting length of the tendon */
	PropertyDbl _tendonSlackLengthProp;
	double &_tendonSlackLength;

	/** Angle between tendon and fibers at optimal fiber length */
	PropertyDbl _pennationAngleAtOptimalProp;
	double &_pennationAngleAtOptimal;

	/** Maximum contraction velocity of the fibers, in optimal fiberlengths per second */
	PropertyDbl _maxContractionVelocityProp;
	double &_maxContractionVelocity;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Muscle();
	Muscle(const Muscle &aMuscle);
	virtual ~Muscle();
	virtual Object* copy() const = 0;

	void setName(const std::string &aName);
#ifndef SWIG
	Muscle& operator=(const Muscle &aMuscle);
#endif
	void copyData(const Muscle &aMuscle);

	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode();


	//--------------------------------------------------------------------------
	// MUSCLE PARAMETERS
	//--------------------------------------------------------------------------
	virtual double getMaxIsometricForce() const { return _maxIsometricForce; }
	virtual double getOptimalFiberLength() const { return _optimalFiberLength; }
	virtual double getTendonSlackLength() const { return _tendonSlackLength; }
	virtual double getPennationAngleAtOptimalFiberLength() const { return _pennationAngleAtOptimal; }
	virtual double getMaxContractionVelocity() const { return _maxContractionVelocity; }

	virtual void setMaxIsometricForce(double aMaxIsometricForce) { _maxIsometricForce = aMaxIsometricForce;}
	virtual void setOptimalFiberLength(double aOptimalFiberLength) { _optimalFiberLength = aOptimalFiberLength;}
	virtual void setTendonSlackLength(double aTendonSlackLength) { _tendonSlackLength = aTendonSlackLength;}
	virtual void setPennationAngleAtOptimalFiberLength(double aPennationAngle) {_pennationAngleAtOptimal = aPennationAngle;}
	virtual void setMaxContractionVelocity(double aMaxContractionVelocity) {_maxContractionVelocity = aMaxContractionVelocity;}

	//--------------------------------------------------------------------------
	// PROBE MUSCLE FOR INFO
	//--------------------------------------------------------------------------
	virtual double getPennationAngle(const SimTK::State& s) const;
	virtual double getTendonLength(const SimTK::State& s) const;
	virtual double getFiberLength(const SimTK::State& s) const = 0;
	virtual double getNormalizedFiberLength(const SimTK::State& s) const = 0;
	virtual double getFiberLengthAlongTendon(const SimTK::State& s) const;
	virtual double getFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForce(const SimTK::State& s) const;
	virtual double getPassiveFiberForce(const SimTK::State& s) const = 0;
	virtual double getActiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getPassiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getTendonForce(const SimTK::State& s) const = 0;
	virtual double getActivation(const SimTK::State& s) const = 0;

	virtual void setActivation(SimTK::State& s, double activation) const = 0;


	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s ) const = 0;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const = 0;
	virtual double evaluateForceLengthVelocityCurve(double aActivation, double aNormalizedLength, double aNormalizedVelocity) const;
	virtual double calcPennation( double aFiberLength, double aOptimalFiberLength, double aInitialPennationAngle) const;
    
	virtual void equilibrate(SimTK::State& state) const =0;

protected:

	//--------------------------------------------------------------------------
	// FORCE APPLICATION
	//--------------------------------------------------------------------------
	virtual void computeForce(const SimTK::State& state, 
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces, 
							  SimTK::Vector& generalizedForce) const;
public:
	virtual OpenSim::Array<std::string> getRecordLabels() const {
		OpenSim::Array<std::string> labels("");
		labels.append(getName());
		return labels;
	}
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
		OpenSim::Array<double> values(1);
		values.append(getForce(state));
		return values;
	};

	OPENSIM_DECLARE_DERIVED(Muscle, PathActuator);

private:
	void setNull();
	void setupProperties();

protected:
	// Update the geometry attached to the muscle (location of muscle points and connecting segments
	//  all in global/interial frame)
	virtual void updateGeometry(const SimTK::State& s) const;

	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void initState(SimTK::State& s) const;
    virtual void setDefaultsFromState(const SimTK::State& state);

//=============================================================================
};	// END of class Muscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Muscle_h__


