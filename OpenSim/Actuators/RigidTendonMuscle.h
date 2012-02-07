#ifndef __RigidTendonMuscle_h__
#define __RigidTendonMuscle_h__

// RigidTendonMuscle.h
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
#include "osimActuatorsDLL.h"
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/PropertyObjPtr.h>

#ifdef SWIG
	#ifdef OSIMACTUATORS_API
		#undef OSIMACTUATORS_API
		#define OSIMACTUATORS_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a RigidTendonMuscle actuator with no states.
 * The path information for a RigidTendonMuscle is contained
 * in the base class, and the force-generating behavior should is defined in
 * this class. The force (muscle tension) assumes rigid tendon so that 
 * fiber-length and velocity are kinematics dependent and the force-length
 * force-velocity relationships are evaluated directly.
 * The control of this mode is its activation. Force production is instantaneous  
 * with no excitation-to-activation dynamics. 
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMACTUATORS_API RigidTendonMuscle : public Muscle  
{
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	RigidTendonMuscle();
	/** Convenicencec Contructor */
	RigidTendonMuscle(const std::string &aName,double aMaxIsometricForce,double aOptimalFiberLength,double aTendonSlackLength,double aPennationAngle);
	RigidTendonMuscle(const RigidTendonMuscle &aRigidTendonMuscle);
	virtual ~RigidTendonMuscle();
	virtual Object* copy() const;

	void setName(const std::string &aName);
#ifndef SWIG
	RigidTendonMuscle& operator=(const RigidTendonMuscle &aRigidTendonMuscle);
#endif
   void copyData(const RigidTendonMuscle &aRigidTendonMuscle);
	/** Override of the default implementation to account for versioning. */
	virtual void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1);


	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double getTendonLength(const SimTK::State& s) const;
	virtual double getFiberLength(const SimTK::State& s) const;
	virtual double getNormalizedFiberLength(const SimTK::State& s) const;
	virtual double getFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForce(const SimTK::State& s) const;
	virtual double getPassiveFiberForce(const SimTK::State& s) const;
	virtual double getActiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getPassiveFiberForceAlongTendon(const SimTK::State& s) const;
	virtual double getTendonForce(const SimTK::State& s) const {return getFiberForce(s)*cos(getPennationAngle(s)); }
	
	
	virtual double getActivation(const SimTK::State& s) const;
	virtual void setActivation(SimTK::State& s, double activation) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual double computeActuation( const SimTK::State& s ) const;
	virtual double computeIsometricForce(SimTK::State& s, double activation) const;
	virtual void equilibrate(SimTK::State& s) const {}
    
	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
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

	OPENSIM_DECLARE_DERIVED(RigidTendonMuscle, PathActuator);

private:
	void setNull();
	void setupProperties();

protected:
	virtual void setup(Model& aModel);
	virtual void createSystem(SimTK::MultibodySystem& system) const;
	virtual void initState(SimTK::State& s) const;
    virtual void setDefaultsFromState(const SimTK::State& state);

//=============================================================================
};	// END of class RigidTendonMuscle
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __RigidTendonMuscle_h__


