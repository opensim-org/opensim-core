#ifndef MUSCOLLO_MUCOCONSTRAINT_H
#define MUSCOLLO_MUCOCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoConstraint.h                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "MucoBounds.h"
#include "MuscolloUtilities.h"

#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/osimSimulation.h>

#include <simbody/internal/Constraint.h>


namespace OpenSim {

using SimTK::ConstraintIndex;

// ============================================================================
// MucoConstraint
// ============================================================================

class OSIMMUSCOLLO_API MucoConstraint : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoConstraint, Object);
public:
    // Default constructor.
    MucoConstraint();
    // Convenience constructor.
    MucoConstraint(const std::string& name, 
        const std::vector<MucoBounds>& bounds, 
        const std::vector<std::string>& suffixes);
    // TODO Generic constructor.

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    MucoBounds getBoundsAtIndex(const int& idx) const
    {   return MucoBounds(get_lower_bounds(idx), get_upper_bounds(idx)); }
    void setBoundsAtIndex(const int& idx, const MucoBounds& bounds) {
        upd_lower_bounds(idx) = bounds.getLower();
        upd_upper_bounds(idx) = bounds.getUpper();
    }

    // TODO add check that property has correct number of elements
    int getNumberEquations() const
    {   return m_num_equations;  }

    // Function to calculate position errors in constraint equations.
    virtual void calcConstraintErrors(const SimTK::State& state, 
        SimTK::Vector errors) const;

protected:
    OpenSim_DECLARE_LIST_PROPERTY(lower_bounds, double, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(upper_bounds, double, "TODO");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(suffixes, std::vector<std::string>, 
        "TODO"); 

   mutable int m_num_equations;

   void constructProperties();
};

// ============================================================================
// MucoSimbodyConstraint
// ============================================================================

class OSIMMUSCOLLO_API MucoSimbodyConstraint : public MucoConstraint {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoSimbodyConstraint, MucoConstraint);
public:
    // Default constructor.
    MucoSimbodyConstraint();
    // Generic constructor.

    void initialize(Model& model) const;

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    ConstraintIndex getConstraintIndex() const
    {   return ConstraintIndex(get_constraint_index()); }
    void setConstraintIndex(const ConstraintIndex& cid) 
    {   set_constraint_index(cid); }

    // Function to calculate position errors in constraint equations.
    void calcConstraintErrors(const SimTK::State& state,
        SimTK::Vector errors) const override;

    // TODO
    //virtual void calcVelocityErrors(const SimTK::State& state,
    //    SimTK::Vector_<double> out) const;
    //virtual void calcAccelerationErrors(const SimTK::State& state,
    //    SimTK::Vector_<double> out) const;

 
private:
    OpenSim_DECLARE_PROPERTY(constraint_index, int, "TODO");
    OpenSim_DECLARE_PROPERTY(holonomic_only_mode, bool, "TODO");

    
    
    mutable int m_num_position_eqs;
    mutable int m_num_velocity_eqs;
    mutable int m_num_acceleration_eqs;
    mutable SimTK::ReferencePtr<SimTK::Constraint> m_constraint_ref;

    // This member variable avoids unnecessary extra allocation of memory for
    // spatial accelerations, which are incidental to the computation of
    // generalized accelerations when specifying the dynamics with model 
    // constraints present.
    mutable SimTK::Vector_<SimTK::SpatialVec> A_GB;

    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONSTRAINT_H