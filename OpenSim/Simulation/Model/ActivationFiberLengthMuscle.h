#ifndef __ActivationFiberLengthMuscle_h__
#define __ActivationFiberLengthMuscle_h__

// ActivationFiberLengthMuscle.h
// Author: Ajay Seth
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
#include "Muscle.h"

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
 * A base class representing a muscle-tendon actuator. It adds states to the 
 * Muscle class, but does not implement all of the necessary methods,
 * so it is abstract too. The path information for a muscle is contained
 * in this class, and the force-generating behavior should be defined in
 * the derived classes.
 *
 * @author Peter Loan
 * @author Frank C. Anderson
 * @author Ajay Seth
 * @version 2.0
 */
class OSIMSIMULATION_API ActivationFiberLengthMuscle : public Muscle  
{
//=============================================================================
// DATA
//=============================================================================
protected:

	// Defaults for state variables.
    double _defaultActivation;
    double _defaultFiberLength;

	static const int STATE_ACTIVATION;
	static const int STATE_FIBER_LENGTH;

	static const std::string STATE_ACTIVATION_NAME;
	static const std::string STATE_FIBER_LENGTH_NAME;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ActivationFiberLengthMuscle();
	ActivationFiberLengthMuscle(const ActivationFiberLengthMuscle &aMuscle);
	virtual ~ActivationFiberLengthMuscle();
	virtual Object* copy() const = 0;

#ifndef SWIG
	ActivationFiberLengthMuscle& operator=(const ActivationFiberLengthMuscle &aMuscle);
#endif
	virtual void equilibrate(SimTK::State& state) const;

    //--------------------------------------------------------------------------
    // GET
    //--------------------------------------------------------------------------
    // Defaults
    virtual double getDefaultActivation() const;
    virtual void setDefaultActivation(double activation);
    virtual double getDefaultFiberLength() const;
    virtual void setDefaultFiberLength(double length);

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double getFiberLength(const SimTK::State& s) const;
    virtual void setFiberLength(SimTK::State& s, double fiberLength) const;
	virtual double getFiberLengthDeriv(const SimTK::State& s) const;
	virtual void setFiberLengthDeriv(const SimTK::State& s, double fiberLengthDeriv) const;
	virtual double getNormalizedFiberLength(const SimTK::State& s) const;
	virtual double getFiberLengthAlongTendon(const SimTK::State& s) const;
	virtual double getTendonLength(const SimTK::State& s) const;
	virtual double getFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForce(const SimTK::State& s) const;
	virtual double getPassiveFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getPassiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getPassiveForce( const SimTK::State& s) const;
	virtual void setPassiveForce(const SimTK::State& s, double aForce) const;
	virtual double getTendonForce(const SimTK::State& s) const;
	virtual void setTendonForce(const SimTK::State& s, double aForce) const;
	virtual double getActivation(const SimTK::State& s) const;
    virtual void setActivation(SimTK::State& s, double activation) const;
	virtual double getActivationDeriv(const SimTK::State& s) const;
	virtual void setActivationDeriv(const SimTK::State& s, double activationDeriv) const;
    virtual double getExcitation( const SimTK::State& s) const;
	virtual double getStress(const SimTK::State& s) const;


	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void computeEquilibrium(SimTK::State& s ) const;
	virtual double computeActuation( const SimTK::State& s ) const = 0;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const = 0;
	virtual double computeIsokineticForceAssumingInfinitelyStiffTendon(SimTK::State& s, double aActivation) const;
   
	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
protected:
	virtual void postScale(const SimTK::State& s, const ScaleSet& aScaleSet);

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

	virtual Array<std::string> getStateVariableNames() const;
	virtual SimTK::SystemYIndex getStateVariableSystemIndex(const std::string &stateVariableName) const;

	OPENSIM_DECLARE_DERIVED(ActivationFiberLengthMuscle, Muscle);

private:
	void setNull();
	void setupProperties();

protected:
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void initState(SimTK::State& s) const;
    virtual void setDefaultsFromState(const SimTK::State& state);

	virtual void setStateVariableDeriv(const SimTK::State& s, const std::string &aStateName, double aValue) const;
	virtual double getStateVariableDeriv(const SimTK::State& s, const std::string &aStateName) const;

	virtual SimTK::Vector computeStateVariableDerivatives(const SimTK::State& s) const;

//=============================================================================
};	// END of class ActivationFiberLengthMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Muscle_h__


