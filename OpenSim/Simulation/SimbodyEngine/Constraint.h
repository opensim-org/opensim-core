#ifndef OPENSIM_CONSTRAINT_H_
#define OPENSIM_CONSTRAINT_H_
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Constraint.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A parent class for implementing a Simbody Constraint.
 * Specific constraints should be derived from this class. 
 *
 * @author Frank C. Anderson
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Constraint : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Constraint, ModelComponent);

//=============================================================================
// PROPERTY
//=============================================================================
public:
    /* Note: 'isEnforced' replaced 'isDisabled' as of OpenSim 4.0 */
    OpenSim_DECLARE_PROPERTY(isEnforced, bool,
        "Flag indicating whether the constraint is enforced or not."
        "Enforced means that the constraint is active in subsequent "
        "dynamics realizations. NOTE: Prior to OpenSim 4.0, this behavior "
        "was controlled by the 'isDisabled' property, where 'true' meant "
        "the constraint was not being enforced. Thus, if 'isDisabled' is"
        "'true', then 'isEnforced' is false." );

//=============================================================================
// METHODS
//=============================================================================
//--------------------------------------------------------------------------
// CONSTRUCTION
//-------------------------------------------------------------------------
    Constraint();
    virtual ~Constraint();

    virtual void updateFromConstraint(SimTK::State& s,
                                      const Constraint &aConstraint);

    /**
    * Determine whether or not this Constraint is being enforced. */
    virtual bool isEnforced(const SimTK::State& s) const;

   /**
    * Set whether or not this Constraint is enforced.
    * The realization Stage is dropped to Instance whenever the 'isEnforced'
    * flag is changed, but setting the same value has no effect. */
    virtual bool setIsEnforced(SimTK::State& s, bool isEnforced);

    virtual void
    calcConstraintForces(const SimTK::State& s,
                       SimTK::Vector_<SimTK::SpatialVec>& bodyForcesInAncestor, 
                       SimTK::Vector& mobilityForces) const;

    /** 
     * Methods to query the Constraint forces (defaults to the Lagrange 
     * multipliers) applied to the MultibodySystem. The names of the quantities
     * (column labels) are returned by this first method, getRecordLabels() */
    virtual Array<std::string> getRecordLabels() const;
    /**
     * Given a SimTK::State, extract all the values necessary to report 
     * constraint forces (e.g. multipliers). Subclasses can override to report
     * the location, frame, etc.. of force application. This method is used in
     * conjunction with getRecordLabels() and must return an Array of equal
     * size. */
    virtual Array<double> getRecordValues(const SimTK::State& state) const;

    /**
    * This method specifies the interface that a constraint must implement
    * in order to be used by the Induced Accelerations Analysis
    */
    virtual void setContactPointForInducedAccelerations(const SimTK::State &s,
                                                        SimTK::Vec3 point){
        throw Exception("This constraint does not implement "
                        "setContactPointForInducedAccelerations");
    }

protected:
    // ModelComponent interface.
    void extendConnectToModel(Model& aModel) override;
    void extendInitStateFromProperties(SimTK::State& state) const override;
    void extendSetPropertiesFromState(const SimTK::State& state) override;

    /** Helper method to assign the underlying SimTK::Constraint index */
    void assignConstraintIndex(SimTK::ConstraintIndex ix) const {
        const_cast<Self*>(this)->_index = ix;
     }

    void updateFromXMLNode(SimTK::Xml::Element& node,
                           int versionNumber) override;

private:
    void setNull();
    void constructProperties();

    /** ID for the constraint in Simbody. */
    SimTK::ResetOnCopy<SimTK::ConstraintIndex> _index;

    friend class SimbodyEngine;

//=============================================================================
};  // END of class Constraint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONSTRAINT_H_


