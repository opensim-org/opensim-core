#ifndef __Force_h__
#define __Force_h__
// Force.h
// Author: Peter Eastman, Ajay Seth
/*
 * Copyright (c)  2009 Stanford University. All rights reserved.
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
#include "OpenSim/Simulation/osimSimulationDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include <SimTKsimbody.h>

namespace OpenSim {

class Model;
class Body;
class Coordinate;
class ForceAdapter;

/**
 * This abstract class represents a force applied to bodies or generalized coordinates during a simulation.
 * Each subclass represents a different type of force.  The actual force computation is done by a SimTK::Force,
 * which is created by createSystem().
 *
 * @author Peter Eastman
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Force : public ModelComponent
{
	protected:
	/** Flag indicating whether the force is disabled or not.  Disabled
	means that the force is not active in subsequent dynamics realizations. */
	PropertyBool _isDisabledProp;

	/** ID for the force in Simbody. */
	SimTK::ForceIndex _index;

//=============================================================================
// METHODS
//=============================================================================
public:
	Force();
	Force(const Force &aForce);
	virtual ~Force();

	/**
	 * deserialization from XML, necessary so that derived classes can (de)serialize
	 */
	Force(DOMElement* aNode): ModelComponent(aNode) {setNull(); setupProperties(); };

#ifndef SWIG
	Force& operator=(const Force &aForce);
#endif
	void copyData(const Force &aForce);
	virtual Object* copy() const = 0;
	/**
	 * Methods to query a Force for the value actually applied during simulation
	 * The names of the quantities (column labels) is returned by this first function
	 * getRecordLabels()
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const {
		return OpenSim::Array<std::string>();
	}
	/**
	 * Given SimTK::State object extract all the values necessary to report forces, application location
	 * frame, etc. used in conjunction with getRecordLabels and should return same size Array
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
		return OpenSim::Array<double>();
	};

	/** return if the Force is disabled or not */
	virtual bool isDisabled(const SimTK::State &s) const;
	/** Set the Force as disabled (true) or not (false)*/
	virtual void setDisabled(SimTK::State &s, bool disabled);
    /**
     * Get the number of state variables allocated by this force.  The default implementation
     * returns 0.  Subclasses that allocate state variables must override it.
     */
	virtual int getNumStateVariables() const { return 0;};

	/** return a flag indicating whether the Force is applied along a Path
	 *  if you override this method to return true for a specific subclass, it must
	 * also implement the getGeometryPath() mathod
	 */
	virtual bool hasGeometryPath() const { return false;};
	OPENSIM_DECLARE_DERIVED(Force, Object);

protected:
	/**
	 * Subclasses should override these methods appropriately.
	 */
    virtual void setup(Model& model);
	virtual void initState(SimTK::State& state) const;

	/**
	 * Default is to create a ForceAdapter which is a SimTK::Force::Custom
	 * as the udnerlying computational component. Subclasses override to
	 * employ other SimTK::Forces.
	 */
	virtual void createSystem(SimTK::MultibodySystem& system) const;
    virtual void setDefaultsFromState(const SimTK::State& state);

	/**
	 * Subclasses must implement this method to compute the forces that should be applied to bodies
	 * and generalized speeds.
	 * This is invoked by ForceAdapter to perform the force computation.
	 */
	virtual void computeForce(const SimTK::State& state,
							  SimTK::Vector_<SimTK::SpatialVec>& bodyForces,
							  SimTK::Vector& generalizedForces) const {};
	/**
	 * Subclasses may optionally override this method to compute a contribution to the potential
	 * energy of the system.  The default implementation returns 0, which is appropriate for forces
	 * that do not contribute to potential energy.
	 */
	virtual double computePotentialEnergy(const SimTK::State& state) const;
	/**
	 * Apply a force to a particular body.  This method applies only a force, not a torque, which is
	 * equivalent to applying the force at the body's center of mass.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aBody    the body to apply the force to
	 * @param aForce   the force to apply, specified in the inertial frame
	 * @param bodyForces  the current set of system bodyForces this force is added to
	 */
	void applyForce(const SimTK::State &s, const OpenSim::Body &aBody,
					const SimTK::Vec3& aForce, SimTK::Vector_<SimTK::SpatialVec> &bodyForces) const;
	/**
	 * Apply a force to a particular body.  Based on what point the force is applied at, this method
	 * automatically determines the torque produced by the force and applies that as well.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aBody    the body to apply the force to
	 * @param aPoint   the point at which to apply the force, specifieid in the inertial frame
	 * @param aForce   the force to apply, specified in the body's frame
	 * @param bodyForces  the current set of system bodyForces this force is added to
	 */
	void applyForceToPoint(const SimTK::State &s, const OpenSim::Body &aBody, const SimTK::Vec3& aPoint,
						   const SimTK::Vec3& aForce, SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const;
	/**
	 * Apply a torque to a particular body.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aBody    the body to apply the force to
	 * @param aTorque  the torque to apply, specified in the inertial frame
	 * @param bodyForces  the current set of system bodyForces this force is added to
	 */
	void applyTorque(const SimTK::State &s, const OpenSim::Body &aBody,
					 const SimTK::Vec3& aTorque, SimTK::Vector_<SimTK::SpatialVec> &bodyForces) const;
	/**
	 * Apply a generalized force.
	 *
	 * This method may only be called from inside computeForce().  Invoking it at any other time
	 * will produce an exception.
	 *
	 * @param aCoord  the generalized coordinate to apply the force to
	 * @param aForce  the force to apply
	 * @param generalizedForces  the current set of system generalizedForces this force is added to
	 */
	void applyGeneralizedForce(const SimTK::State &s, const Coordinate &aCoord,
							   double aForce, SimTK::Vector &generalizedForces) const;


private:

	void setNull();
	void setupProperties();

	friend class ForceAdapter;

//=============================================================================
};	// END of class Force
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __Force_h__


