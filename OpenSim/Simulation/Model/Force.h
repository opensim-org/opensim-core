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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "OpenSim/Simulation/Model/ModelComponent.h"

namespace OpenSim {

class PhysicalFrame;
class Coordinate;


/**
 * This abstract class represents a force applied to bodies or generalized 
 * coordinates during a simulation. Each subclass represents a different type 
 * of force. The actual force computation is done by a SimTK::Force, which is 
 * created by extendAddToSystem().
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
    /* Note: appliesForce replaced isDisabled as of OpenSim 4.0  */
    OpenSim_DECLARE_PROPERTY(appliesForce, bool,
        "Flag indicating whether the force is applied or not. If true the force"
        "is applied to the MultibodySystem otherwise the force is not applied."
        "NOTE: Prior to OpenSim 4.0, this behavior was controlled by the "
        "'isDisabled' property, where 'true' meant that force was not being "
        "applied. Thus, if 'isDisabled' is true, then 'appliesForce` is false.");
    //=========================================================================
    // OUTPUTS
    //=========================================================================
    OpenSim_DECLARE_OUTPUT(potential_energy, double,
                           computePotentialEnergy, SimTK::Stage::Velocity);

//=============================================================================
// PUBLIC METHODS
//=============================================================================

    /**
    * Tell SimBody to parallelize this force. Should be 
    * set to true for any forces that will take time to 
    * complete their calcForce method. Note that all forces
    * that set this flag to false will be put in series on a
    * thread that is running in parallel with other forces
    * that marked this flag as true.
    */
    virtual bool shouldBeParallelized() const
    {
        return false;
    }

    /** Return if the Force is applied (or enabled) or not.                   */
    bool appliesForce(const SimTK::State& s) const;
    /** %Set whether or not the Force is applied.                             */
    void setAppliesForce(SimTK::State& s, bool applyForce) const;

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
    virtual OpenSim::Array<double>
    getRecordValues(const SimTK::State& state) const {
        return OpenSim::Array<double>();
    };


    /** Return a flag indicating whether the Force is applied along a Path. If
    you override this method to return true for a specific subclass, it must
    also implement the getGeometryPath() method. **/
    virtual bool hasGeometryPath() const {
        return getPropertyIndex("GeometryPath").isValid();
    };

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

    /** Subclass should override; be sure to invoke 
    Super::extendInitStateFromProperties() at the
    beginning of the overriding method. **/
    void extendInitStateFromProperties(SimTK::State& state) const override;

    /** Default is to create a ForceAdapter which is a SimTK::Force::Custom
    as the underlying computational component. Subclasses override to employ 
    other SimTK::Forces; be sure to invoke Force::extendAddToSystem() at the
    beginning of the overriding method. **/
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /** Subclass should override; be sure to invoke 
    Force::extendSetPropertiesFromState() at the beginning of the overriding 
    method. **/
    void extendSetPropertiesFromState(const SimTK::State& state) override;
    
    //--------------------------------------------------------------------------
    // Force interface.
    //--------------------------------------------------------------------------

    /**
     * Subclasses must implement this method to compute the forces that should 
     * be applied to bodies and generalized speeds.
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
     * @param point      the point at which to apply the force, specified in 
     *                      the body's frame
     * @param force      the force to apply, specified in the inertial 
     *                      (ground) reference frame
     * @param bodyForces the set of system bodyForces to which this force 
     *                      is added
     */
    void applyForceToPoint(const SimTK::State&                state, 
                           const PhysicalFrame&               body, 
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
                     const PhysicalFrame&               body,
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
     * @param coord              the generalized coordinate to which the 
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
    void updateFromXMLNode(SimTK::Xml::Element& node,
                           int versionNumber) override;

    /** ID for the force in Simbody. */
    SimTK::ResetOnCopy<SimTK::ForceIndex> _index;

private:
    void setNull();
    void constructProperties();

    friend class ForceAdapter;

//=============================================================================
};  // END of class Force
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FORCE_H_


