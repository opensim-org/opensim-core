#ifndef OPENSIM_FORCE_H_
#define OPENSIM_FORCE_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Force.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Eastman, Ajay Seth                                        *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */
// INCLUDE
#include "OpenSim/Simulation/osimSimulationDLL.h"
#include <OpenSim/Common/PropertyBool.h>
#include "OpenSim/Simulation/Model/ModelComponent.h"
#include <Simbody.h>

namespace OpenSim {

class Model;
class Body;
class Coordinate;
class ForceAdapter;


/**
 * This abstract class represents a force applied to bodies or generalized 
 * coordinates during a simulation. Each subclass represents a different type 
 * of force. The actual force computation is done by a SimTK::Force, which is 
 * created by addToSystem().
 *
 * @author Peter Eastman
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Force : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Force, ModelComponent);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with this class. **/
    /**@{**/
    /** A Force element is active (enabled) by default. **/
	OpenSim_DECLARE_PROPERTY(isDisabled, bool,
		"Flag indicating whether the force is disabled or not. Disabled means"
		" that the force is not active in subsequent dynamics realizations.");
    /**@}**/

//=============================================================================
// PUBLIC METHODS
//=============================================================================

    // Constructors are protected; has default destructor.

    /** Implements a copy constructor just so it can invalidate the 
    SimTK::Force index after copying. **/
	Force(const Force &aForce);

#ifndef SWIG
    /** Implements a copy assignment operator just so it can invalidate the 
    SimTK::Force index after the assignment. **/
	Force& operator=(const Force &aForce);
#endif

	/** Return if the Force is disabled or not. */
	bool isDisabled(const SimTK::State& s) const;
	/** Set the Force as disabled (true) or not (false). */
	void setDisabled(SimTK::State& s, bool disabled) const;

	/**
	 * Methods to query a Force for the value actually applied during 
     * simulation. The names of the quantities (column labels) is returned by 
     * this first function getRecordLabels().
	 */
	virtual OpenSim::Array<std::string> getRecordLabels() const {
		return OpenSim::Array<std::string>();
	}
	/**
	 * Given SimTK::State object extract all the values necessary to report 
     * forces, application location frame, etc. used in conjunction with 
     * getRecordLabels and should return same size Array.
	 */
	virtual OpenSim::Array<double> getRecordValues(const SimTK::State& state) const {
		return OpenSim::Array<double>();
	};


	/** Return a flag indicating whether the Force is applied along a Path. If
    you override this method to return true for a specific subclass, it must
	also implement the getGeometryPath() method. **/
	virtual bool hasGeometryPath() const { return getPropertyIndex("GeometryPath").isValid();};

protected:
	/** Default constructor sets up Force-level properties; can only be
    called from a derived class constructor. **/
    Force();

	/** Deserialization from XML, necessary so that derived classes can 
    (de)serialize. **/
	Force(SimTK::Xml::Element& node) : Super(node) 
    {   setNull(); constructProperties(); }

    //--------------------------------------------------------------------------
    // ModelComponent interface.
    //--------------------------------------------------------------------------

	/** Subclass should override; be sure to invoke Force::initStateFromProperties() at the
    beginning of the overriding method. **/
	virtual void initStateFromProperties(SimTK::State& state) const OVERRIDE_11;

	/** Default is to create a ForceAdapter which is a SimTK::Force::Custom
	as the underlying computational component. Subclasses override to employ 
    other SimTK::Forces; be sure to invoke Force::addToSystem() at the
    beginning of the overriding method. **/
	virtual void addToSystem(SimTK::MultibodySystem& system) const OVERRIDE_11;
	/** Subclass should override; be sure to invoke 
    Force::setPropertiesFromState() at the beginning of the overriding method. **/
    virtual void setPropertiesFromState(const SimTK::State& state) OVERRIDE_11;
    
    //--------------------------------------------------------------------------
    // Force interface.
    //--------------------------------------------------------------------------

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
	 * Apply a force at a particular point (a "station") on a given body. Note
     * that the point Vec3(0) is the body origin, not necessarily the center
     * of mass whose location is maintained relative to the body origin.
     * Although this applies a pure force to the given point, that will also
     * result in a torque acting on the body when looking at the resultant at
     * some other point.
	 *
	 * This method may only be called from inside computeForce(). Invoking it 
     * at any other time will produce an exception.
	 *
     * @param state      state used only to determine which element of 
     *                      \a bodyForces to modify
	 * @param body       the body to apply the force to
	 * @param point      the point at which to apply the force, specifieid in 
     *                      the body's frame
	 * @param force      the force to apply, specified in the inertial 
     *                      (ground) reference frame
	 * @param bodyForces the set of system bodyForces to which this force 
     *                      is added
	 */
	void applyForceToPoint(const SimTK::State&                state, 
                           const OpenSim::Body&               body, 
                           const SimTK::Vec3&                 point,
						   const SimTK::Vec3&                 force, 
                           SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const;
	/**
	 * Apply a torque to a particular body.
	 *
	 * This method may only be called from inside computeForce(). Invoking it 
     * at any other time will produce an exception.
	 *
     * @param state      state used only to determine which element of 
     *                      \a bodyForces to modify
	 * @param body       the body to apply the force to
	 * @param torque     the torque to apply, specified in the inertial frame
	 * @param bodyForces the set of system bodyForces to which this force 
     *                      is added
	 */
	void applyTorque(const SimTK::State&                state, 
                     const OpenSim::Body&               body,
					 const SimTK::Vec3&                 torque, 
                     SimTK::Vector_<SimTK::SpatialVec>& bodyForces) const;
	/**
	 * Apply a generalized force.
	 *
	 * This method may only be called from inside computeForce(). Invoking it 
     * at any other time will produce an exception.
	 *
     * @param state              state used only to determine which element of 
     *                              \a generalizedForces to modify
	 * @param coord              the generalized coordinate to to which the 
     *                              force should be applied
	 * @param force              the (scalar) force to apply
	 * @param generalizedForces  the set of system generalizedForces to which
     *                              the force is to be added
	 */
	void applyGeneralizedForce(const SimTK::State&  state, 
                               const Coordinate&    coord,
							   double               force, 
                               SimTK::Vector&       generalizedForces) const;

protected:
	/** ID for the force in Simbody. */
	SimTK::ForceIndex   _index;

private:
	void setNull();
	void constructProperties();
	void copyData(const Force &aForce);

	friend class ForceAdapter;

//=============================================================================
};	// END of class Force
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FORCE_H_


