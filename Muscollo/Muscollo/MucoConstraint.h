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
// MucoConstraintInfo
// ============================================================================

/// TODO
class OSIMMUSCOLLO_API MucoConstraintInfo : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoConstraintInfo, Object);
public:
    MucoConstraintInfo();

    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    // Get and set methods.
    std::vector<MucoBounds> getBounds() const {
        std::vector<MucoBounds> bounds;
        for (int i = 0; i < getProperty_bounds().size(); ++i) {
            bounds.push_back(get_bounds(i));
        }
        return bounds;
    }
    /// @copydoc getBounds()
    std::vector<std::string> getSuffixes() const {
        std::vector<std::string> suffixes;
        for (int i = 0; i < getProperty_suffixes().size(); ++i) {
            suffixes.push_back(get_suffixes(i));
        }
        return suffixes;
    }
    void setBounds(const std::vector<MucoBounds>& bounds) {
        checkEquationConsistency(bounds.size());
        for (int i = 0; i < bounds.size(); ++i) {
            set_bounds(i, bounds[i]);
        }
    }
    void setSuffixes(const std::vector<std::string>& suffixes) {
        checkEquationConsistency(suffixes.size());
        for (int i = 0; i < suffixes.size(); ++i) {
            set_suffixes(i, suffixes[i]);
        }
    }
    int getNumEquations() const {   
        OPENSIM_THROW_IF(!m_num_equations, Exception, "Insufficient "
            "information available to determine the number of scalar "
            "constraint equations. Please set the bounds property. Optionally, "
            "you may also set the suffixes property.");
        return m_num_equations;  
    }

    /// Get a list of constraint labels based on the constraint name and, if
    /// specified, the list of suffixes. If no suffixes have been specified, 
    /// zero-indexed, numeric suffixes will be applied as a default.
    std::vector<std::string> getConstraintLabels();

    /// Print the name, type, number of scalar equations, and bounds for this 
    /// constraint.
    void printDescription(std::ostream& stream = std::cout) const;

private:
    OpenSim_DECLARE_LIST_PROPERTY(bounds, MucoBounds, "The bounds on the set "
        "of scalar constraint equations.");
    OpenSim_DECLARE_LIST_PROPERTY(suffixes, std::string, "(Optional) A list of "
        "strings to create unique labels for the scalar constraint equations. "
        "These are appended to the name of the MucoConstraint object when "
        "calling getConstraintLabels().");

    void constructProperties();

    int m_num_equations;
    void checkEquationConsistency(int propSize) {
        if (!m_num_equations) {
            m_num_equations = propSize;
        } else {
            OPENSIM_THROW_IF(m_num_equations != propSize, Exception, "Size of "
                "property assignment not consistent with current number of "
                "constraint equations.");
        }
    }
};

// ============================================================================
// MucoMultibodyConstraintInfo
// ============================================================================

class OSIMMUSCOLLO_API MucoMultibodyConstraintInfo : public MucoConstraintInfo{
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoMultibodyConstraintInfo, 
        MucoConstraintInfo);
public:
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal variable. Avoid repeated calls to this function.
    ConstraintIndex getSimbodyConstraintIndex() const
    {   return ConstraintIndex(m_simbody_constraint_index); }

    /// Get the number of scalar constraint equations at each kinematic level.
    /// Note that the total number of scalar constraint equations enforced is
    /// *NOT* equal to the sum of each of these values -- you must include the
    /// first and second derivatives of the position equations and the first
    /// derivatives of the velocity equations into that count as well (this 
    /// value can be obtained by calling getNumEquations()). 
    int getNumPositionEquations() const
    {   return m_num_position_eqs; }
    int getNumVelocityEquations() const
    {   return m_num_velocity_eqs; }
    int getNumAccelerationEquations() const
    {   return m_num_acceleration_eqs; }

private:
    int m_num_position_eqs;
    int m_num_velocity_eqs;
    int m_num_acceleration_eqs;
    int m_simbody_constraint_index;

    /// The constructor for this class is private since we don't want the user
    /// constructing these infos directly. Rather, they should be constructed
    /// directly from the model within the initialization for each MucoPhase,
    /// which is a friend of this class. This implementation ensures that the
    /// correct information is passed to set the default property values, which 
    /// should be sufficient for most users. 
    MucoMultibodyConstraintInfo(int cid, int mp, int mv, int ma);
    friend class MucoPhase;
};

// ============================================================================
// MucoPathConstraint
// ============================================================================

/// A path constraint to be enforced in the optimal control problem.
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoPathConstraint : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoPathConstraint, Object);
public:
    // Default constructor.
    MucoPathConstraint();
        
    MucoConstraintInfo getConstraintInfo() const 
    {   return get_constraint_info(); }
    void setConstraintInfo(const MucoConstraintInfo& cInfo) 
    {   return set_constraint_info(cInfo); }
    int getPathConstraintIndex() const
    {   return m_path_constraint_index; }

    /// Calculate errors in the path constraint equations. The *errors* argument 
    /// represents the concatenated error vector for all path constraints in the 
    /// MucoProblem. The *m_path_constraint_index* variable the number of scalar
    /// constraint equations (obtained from the MucoConstraintInfo) are used to 
    /// create a view into *errors* to access the elements this 
    /// MucoPathConstraint is responsible for.
    void calcPathConstraintErrors(const SimTK::State& state,
            SimTK::Vector& errors) const {
    
        // This vector shares writable, borrowed space from the *errors* vector 
        // (provided by the MucoProblem) to the elements for which this 
        // MucoPathConstraint provides constraint error information.
        SimTK::Vector theseErrors(getConstraintInfo().getNumEquations(), 
            errors.getContiguousScalarData() + getPathConstraintIndex(), true);
        calcPathConstraintErrorsImpl(state, theseErrors);
    }

    /// For use by solvers. This also performs error checks on the Problem.
    void initialize(const Model& model, const int& pathConstraintIndex) const;
    
protected:
    OpenSim_DECLARE_PROPERTY(constraint_info, MucoConstraintInfo, "TODO.");

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
    virtual void calcPathConstraintErrorsImpl(const SimTK::State& state,
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
   mutable int m_path_constraint_index;
};

inline void MucoPathConstraint::calcPathConstraintErrorsImpl(
        const SimTK::State&, SimTK::Vector&) const {}

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONSTRAINT_H