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

#include <OpenSim/Common/Object.h>
#include <simbody/internal/Constraint.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "osimMuscolloDLL.h"

using SimTK::ConstraintIndex;

namespace OpenSim {

// ============================================================================
// MucoConstraint
// ============================================================================

/// A path constraint to be enforced in the optimal control problem.
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoConstraint : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoConstraint, Object);
friend class MucoSimbodyConstraint;
public:
    // Default constructor.
    MucoConstraint();

    // Get and set methods.
    std::vector<MucoBounds> getBounds() const 
    {   return m_bounds; }
    std::vector<std::string> getSuffixes() const 
    {   return m_suffixes; }
    void setBounds(const std::vector<MucoBounds>& bounds) {
        for (int i = 0; i < bounds.size(); ++i) {
            set_bounds(i, bounds[i]);
        }
        m_bounds = bounds;
    }  
    void setSuffixes(const std::vector<std::string>& suffixes) {
        // TODO can we set this list property without a loop?
        for (int i = 0; i < suffixes.size(); ++i) {
            set_suffixes(i, suffixes[i]);
        }
        m_suffixes = suffixes;
    }
    int getNumEquations() const
    {   return m_num_equations; }
    int getIndex() const
    {   return m_index; }
    
    /// Get a list of constraint labels based on the constraint name and, if
    /// specified, the list of suffixes. If no suffixes have been specified, 
    /// zero-indexed, numeric suffixes will be applied as a default.
    std::vector<std::string> getConstraintLabels();

    /// Calculate errors in the constraint equations. The *errors* argument 
    /// represents the concatenated error vector for all constraints in the 
    /// MucoProblem. The *m_index* and *m_num_equations* internal variables are 
    /// used to create a view into *errors* to access the elements this 
    /// MucoConstraint is responsible for.
    void calcConstraintErrors(const SimTK::State& state, 
            SimTK::Vector& errors) const {
    
        // This vector shares writable, borrowed space from the *errors* vector 
        // (provided by the MucoProblem) to the elements for which this 
        // MucoConstraint provides constraint error information.
        SimTK::Vector theseErrors(m_num_equations, 
            errors.getContiguousScalarData() + m_index, true);
        calcConstraintErrorsImpl(state, theseErrors);
    }

    /// For use by solvers. This also performs error checks on the Problem.
    void initialize(const Model& model, const int& index) const;

    /// Print the name, type, constraint index, number of scalar equations, and
    /// bounds for this constraint.
    void printDescription(std::ostream& stream = std::cout) const;
    
protected:
    OpenSim_DECLARE_LIST_PROPERTY(bounds, MucoBounds, "The bounds on the set "
        "of scalar constraint equations.");
    OpenSim_DECLARE_LIST_PROPERTY(suffixes, std::string, "(Optional) A list of "
        "strings to create unique labels for the scalar constraint equations. "
        "These are appended to the name of the MucoConstraint object when "
        "calling getConstraintLabels()."); 

    /// Perform any caching. Make sure to first clear any caches, as this is
    /// invoked every time the problem is solved.
    /// Upon entry, getModel() is available.
    /// Use this opportunity to check for errors in user input, in addition to
    /// the checks provided in initialize().
    virtual void initializeImpl() const {}
    /// Precondition: state is realized to SimTK::Stage::Position.
    /// If you need access to the controls, you must realize to Velocity:
    /// @code
    /// getModel().realizeVelocity(state);
    /// @endcode
    virtual void calcConstraintErrorsImpl(const SimTK::State& state,
        SimTK::Vector& errors) const;
    /// For use within virtual function implementations.
    const Model& getModel() const {
        OPENSIM_THROW_IF(!m_model, Exception,
            "Model is not available until the start of initializing.");
        return m_model.getRef();
    }  

private: 
   void constructProperties();

   mutable SimTK::ReferencePtr<const Model> m_model;
   mutable std::vector<MucoBounds> m_bounds;
   mutable std::vector<std::string> m_suffixes;
   mutable int m_index;
   mutable int m_num_equations;
};

inline void MucoConstraint::calcConstraintErrorsImpl(const SimTK::State&,
        SimTK::Vector&) const {}

// ============================================================================
// MucoSimbodyConstraint
// ============================================================================

/// A class to conveniently add a Simbody constraint in the model to the
/// problem as a MucoConstraint.
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoSimbodyConstraint : public MucoConstraint {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoSimbodyConstraint, MucoConstraint);
public:
    // Default constructor.
    MucoSimbodyConstraint();

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    ConstraintIndex getModelConstraintIndex() const
    {   return ConstraintIndex(get_model_constraint_index()); }
    void setModelConstraintIndex(const ConstraintIndex& cid) 
    {   set_model_constraint_index(cid); }
    void enforcePositionLevelOnly(const bool& tf) 
    {   set_enforce_position_level_only(tf); }
    bool isEnforcingPositionLevelOnly() 
    {   return get_enforce_position_level_only(); }

protected:
    void initializeImpl() const override;
    void calcConstraintErrorsImpl(const SimTK::State& state,
        SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_PROPERTY(model_constraint_index, int, "The index of the "
        "constraint in the model's constraint set, which maps to a "
        "SimTK::ConstraintIndex. Note that this index is separate in use from "
        "the *m_index* member variable of MucoConstraint.");
    OpenSim_DECLARE_PROPERTY(enforce_position_level_only, bool, "Whether or "
        "not to only enforce the position-level constraint equations of a "
        "holonomic constraint, and not the derivatives. An error is thrown if "
        "a MucoSimbodyConstraint for a nonholonomic or acceleration constraint "
        "has this property enabled.");

    mutable int m_num_position_eqs;
    mutable int m_num_velocity_eqs;
    mutable int m_num_acceleration_eqs;
    mutable SimTK::ReferencePtr<const SimTK::Constraint> m_constraint_ref;

    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONSTRAINT_H