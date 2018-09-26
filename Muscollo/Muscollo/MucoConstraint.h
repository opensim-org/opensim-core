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

class Model;

using SimTK::ConstraintIndex;

// ============================================================================
// MucoConstraint
// ============================================================================

/// A path constraint to be enforced in the optimal control problem.
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoConstraint : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoConstraint, Object);
public:
    // Default constructor.
    MucoConstraint();

    // Get and set methods.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<MucoBounds> getBounds() const {
        checkConsistency(getProperty_lower_bounds().size());

        std::vector<MucoBounds> boundsVec;
        for (int i = 0; i < getProperty_lower_bounds().size(); ++i) {
            boundsVec.push_back({get_lower_bounds(i), get_upper_bounds(i)});
        } 
        return boundsVec;
    }
    void setBounds(const std::vector<MucoBounds>& boundsVec) {
        checkConsistency(boundsVec.size());

        for (int i = 0; i < boundsVec.size(); ++i) {
            set_lower_bounds(i, boundsVec[i].getLower());
            set_upper_bounds(i, boundsVec[i].getUpper());
        }
    }
    MucoBounds getBoundsAtIndex(const int& idx) const {
        checkConsistency(getProperty_lower_bounds().size());
        if ((idx < 0) || (idx >= m_num_equations)) {
            OPENSIM_THROW(Exception, "Invalid index.");
        }

        return MucoBounds(get_lower_bounds(idx), get_upper_bounds(idx)); 
    }

    std::vector<std::string> getSuffixes() const {
        checkConsistency(get_suffixes().size());
        return get_suffixes(); 
    }
    void setSuffixes(const std::vector<std::string>& suffixes) {   
        checkConsistency(suffixes.size());
        set_suffixes(suffixes); 
    }

    // Function to calculate position errors in constraint equations.
    SimTK::Vector calcConstraintErrors(const SimTK::State& state) const {
        SimTK::Vector errors(m_num_equations, 0.0);
        calcConstraintErrorsImpl(state, errors);
        return errors;
    }
    /// For use by solvers. This also performs error checks on the Problem.
    void initialize(const Model& model) const {
        m_model.reset(&model);
        initializeImpl();
    }
    
protected:
    OpenSim_DECLARE_LIST_PROPERTY(lower_bounds, double, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(upper_bounds, double, "TODO");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(suffixes, std::vector<std::string>, 
        "TODO"); 

    /// Perform any caching. Make sure to first clear any caches, as this is
    /// invoked every time the problem is solved.
    /// Upon entry, getModel() is available.
    /// Use this opportunity to check for errors in user input.
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

    mutable int m_num_equations;
    
private: 
   void constructProperties();

   mutable SimTK::ReferencePtr<const Model> m_model;

   void checkConsistency(const int& length) const {
       if (length) {
           if (!m_num_equations) { 
              m_num_equations = length;
           } else {
               OPENSIM_THROW_IF(length != m_num_equations, Exception,
                   "Length of input argument is not consistent with current "
                   "number of scalar constraint equations.");
           }
       } else {
           OPENSIM_THROW(Exception, "Property not specified.");
       }
   }
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
    ConstraintIndex getConstraintIndex() const
    {   return ConstraintIndex(get_constraint_index()); }
    void setConstraintIndex(const ConstraintIndex& cid) 
    {   set_constraint_index(cid); }
    void enforcePositionLevelOnly(const bool& tf) 
    {   set_enforce_position_level_only(tf); }
    bool isEnforcingPositionLevelOnly() 
    {   return get_enforce_position_level_only(); }

protected:
    void initializeImpl() const override;
    void calcConstraintErrorsImpl(const SimTK::State& state,
        SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_PROPERTY(constraint_index, int, "TODO");
    OpenSim_DECLARE_PROPERTY(enforce_position_level_only, bool, "TODO");

    mutable int m_num_position_eqs;
    mutable int m_num_velocity_eqs;
    mutable int m_num_acceleration_eqs;
    mutable SimTK::ReferencePtr<const SimTK::Constraint> m_constraint_ref;

    mutable SimTK::Vector perr;
    mutable SimTK::Vector pverr;
    mutable SimTK::Vector pvaerr;

    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONSTRAINT_H