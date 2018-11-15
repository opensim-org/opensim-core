#ifndef OPENSIM_ROLLING_ON_SURFACE_CONSTRAINT_H_
#define OPENSIM_ROLLING_ON_SURFACE_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  RollingOnSurfaceConstraint.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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
#include "UnilateralConstraint.h"
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class implementing a collection of rolling-without-slipping and 
 * non-penetration constraints on a surface.  
 * The underlying Constraints in Simbody are:
 *      PointInPlane to oppose penetration into the ground (unilaterally)
 *      ConstantAngle about the normal to the enforce no twisting
 *      NoSlip1D along one axis of the plane
 *      NoSlip1D along the other axis
 *
 * mu = Coulomb Friction Coefficient
 *
 * Each of these constraints have conditions dependent on the reaction forces
 * that they generate individually and collectively:
 *   PointInPlane normal force (Fn) must be positive (in the direction of the 
 *   normal) ConstantAngle the reaction torque cannot exceed contactRadius*mu*Fn
 *   Both NoSlip conditions are treated together, the magnitude of the combined
 *   reaction forces (in the plane) cannot exceed mu*Fn

 * @author Ajay Seth
 */
class OSIMSIMULATION_API RollingOnSurfaceConstraint 
:   public UnilateralConstraint {
OpenSim_DECLARE_CONCRETE_OBJECT(RollingOnSurfaceConstraint, 
                                UnilateralConstraint);
public:
    OpenSim_DECLARE_PROPERTY(surface_normal, SimTK::Vec3,
        "Surface normal direction in the surface body.");
    OpenSim_DECLARE_PROPERTY(surface_height, double,
        "Surface height in the direction of the normal in the surface body.");
    OpenSim_DECLARE_PROPERTY(friction_coefficient, double,
        "Coulomb friction coefficient for rolling on the surface.");
    OpenSim_DECLARE_PROPERTY(contact_radius, double,
        "A guess at the area of contact approximated by a circle of radius.");

    OpenSim_DECLARE_SOCKET(rolling_body, PhysicalFrame,
        "A frame fixed to the rolling body.");
    OpenSim_DECLARE_SOCKET(surface_body, PhysicalFrame,
        "A frame fixed to the surface body.");

//=============================================================================
// DATA
//=============================================================================
private:

    /** Get the indices of underlying constraints to access from Simbody */
    SimTK::ResetOnCopy<std::vector<SimTK::ConstraintIndex>> _indices;

    /**  This cache acts a temporary hold for the constraint conditions when 
    time has not changed */
    std::vector<bool> _defaultUnilateralConditions;

//=============================================================================
// METHODS
//=============================================================================
public:
    // CONSTRUCTION
    RollingOnSurfaceConstraint();
    virtual ~RollingOnSurfaceConstraint();

    /** %Set rolling body by its name */
    void setRollingBodyByName(const std::string& aBodyName);
    /** %Set surface body by its name */
    void setSurfaceBodyByName(const std::string& aBodyName);

    /**
    * Get whether or not the RollingOnSurfaceConstraint is enforced.
    * Simbody multibody system instance is realized every time the isEnforced
    * changes, BUT multiple sets to the same value have no cost.
    *
    * @param state  the state of the system that includes the constraint status
    */
    bool isEnforced(const SimTK::State& state) const override;

    /**
    * %Set whether or not the RollingOnSurfaceConstraint is enforced.
    * Since the constraint is composed of multiple constraints, this method can
    * disable all the constraints, but enabling is not guaranteed. For example,
    * if the unilateral conditions are violated the constraint will be disabled.
    *
    * @param state      the state to set whether the constraint is enforced or 
                        not.
    * @param isEnforced if true the constraint is enforced.
    */
    bool setIsEnforced(SimTK::State& state, bool isEnforced) override;

    // This method allows finer granularity over the subconstraints according
    // to imposed behavior
    bool setIsEnforced(SimTK::State& state,
                       bool isEnforced,
                       std::vector<bool> shouldBeOn);

    /**
    * Ask the RollingOnSurfaceConstraint for the forces it is imposing on the 
    * system. Simbody multibody system must be realized to at least 
    * Stage::Dynamics. Returns: the bodyForces on those bodies being 
    * constrained (constrainedBodies) a SpatialVec (6 components) describing 
    * resulting torque and force mobilityForces acting along constrained 
    * mobilities
    *
    * @param state  State of model
    * @param bodyForcesInAncestor   Vector of SpatialVecs contain constraint 
    *                               forces
    * @param mobilityForces     Vector of forces that act along the constrained
    *                           mobilities associated with this constraint
    */
    void calcConstraintForces(const SimTK::State& state,
                       SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor,
                       SimTK::Vector& mobilityForces) const override;

    void setContactPointForInducedAccelerations(const SimTK::State &s,
        SimTK::Vec3 point) override;

    /** Test whether unilateral conditions are being satisfied.
        Note: system must be realized to at least Stage::Dynamics */
    std::vector<bool>
    unilateralConditionsSatisfied(const SimTK::State& state) override;


    /** %Set whether constraint is enforced but use cached values 
    for unilateral conditions instead of automatic reevaluation */
    bool setIsEnforcedWithCachedUnilateralConditions(bool isEnforced,
                                                     SimTK::State& state);

protected:
    /** Extend ModelComponent interface */
    void extendConnectToModel(Model& aModel) override;

    /** Create the SimTK::Constraints: which implements this
     * RollingOnSurfaceConstraint. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    /** Populate the SimTK::State: with defaults for the
     * RollingOnSurfaceConstraint. */
    void extendInitStateFromProperties(SimTK::State& state) const override;
    /** Given an existing SimTK::State set defaults for the
     * RollingOnSurfaceConstraint. */
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    /** Updating XML formating to latest revision */
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
                           int versionNumber) override;

private:
    void setNull();
    /** Construct RollingOnSurfaceConstraint's properties */
    void constructProperties();

    // References to the PhysicalFrames on the 
    SimTK::ReferencePtr<const PhysicalFrame> _rollingFrame;
    SimTK::ReferencePtr<const PhysicalFrame> _surfaceFrame;


//=============================================================================
};  // END of class RollingOnSurfaceConstraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_ROLLING_ON_SURFACE_CONSTRAINT_H_


