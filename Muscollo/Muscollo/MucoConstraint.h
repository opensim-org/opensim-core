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

namespace OpenSim {

// ============================================================================
// MucoConstraintInfo
// ============================================================================

/// Information for a given constraint in the optimal control problem. The name
/// should correspond to a MucoPathConstraint in the problem.
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoConstraintInfo : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(MucoConstraintInfo, Object);
public:
    MucoConstraintInfo();

    int getNumEquations() const 
    {   return m_num_equations; }
    /// Get the bounds on the scalar constraint equations. If the number of 
    /// equations have been set, but not the bounds, zero-bounds are returned
    /// as a default. If nothing has been set, this returns an empty vector.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<MucoBounds> getBounds() const {
        std::vector<MucoBounds> bounds;
        for (int i = 0; i < getNumEquations(); ++i) {
            if (getProperty_bounds().empty()) {
                bounds.push_back({0.0, 0.0});
            } else { 
                bounds.push_back(get_bounds(i));
            }
        }
        return bounds;
    }
    /// Get the suffixes for the scalar constraint equations. If the suffixes
    /// have not been set, this returns an empty vector.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<std::string> getSuffixes() const {
        std::vector<std::string> suffixes;
        for (int i = 0; i < getProperty_suffixes().size(); ++i) {
            suffixes.push_back(get_suffixes(i));
        }
        return suffixes;
    }
    /// @details Note: if the number of equations has not been set, this updates 
    /// the internal equation count variable. If the number of equations has 
    /// been set and the vector passed is the incorrect size, an error is 
    /// thrown.
    void setBounds(const std::vector<MucoBounds>& bounds) {
        updProperty_bounds().clear();
        for (int i = 0; i < (int)bounds.size(); ++i) {
            updProperty_bounds().appendValue(bounds[i]);
        }
        updateNumEquationsFromProperty(getProperty_bounds());
    }
    /// @copydoc setBounds()
    void setSuffixes(const std::vector<std::string>& suffixes) {
        updProperty_suffixes().clear();
        for (int i = 0; i < (int)suffixes.size(); ++i) {
            updProperty_suffixes().appendValue(suffixes[i]);
        }
        updateNumEquationsFromProperty(getProperty_suffixes());
    }
    /// Get a list of constraint labels based on the constraint name and, if
    /// specified, the list of suffixes. If no suffixes have been specified, 
    /// zero-indexed, numeric suffixes will be applied as a default. The length
    /// of the returned vector is equal to the value returned by 
    /// getNumEquations().
    std::vector<std::string> getConstraintLabels() const;

    /// Print the name, type, number of scalar equations, and bounds for this 
    /// constraint.
    void printDescription(std::ostream& stream = std::cout) const;

private:
    OpenSim_DECLARE_LIST_PROPERTY(bounds, MucoBounds, "(Optional) The bounds " 
        "on the set of scalar constraint equations.");
    OpenSim_DECLARE_LIST_PROPERTY(suffixes, std::string, "(Optional) A list of "
        "strings to create unique labels for the scalar constraint equations. "
        "These are appended to the name of the MucoConstraint object when "
        "calling getConstraintLabels().");

    void constructProperties();

    int m_num_equations = 0;
    void setNumEquations(int numEqs) {
        m_num_equations = numEqs;
        checkPropertySize(getProperty_bounds());
        checkPropertySize(getProperty_suffixes());
    }
    friend class MucoPathConstraint;
    friend class MucoMultibodyConstraint;
    
    void updateNumEquationsFromProperty(const AbstractProperty& prop) {
        if (!m_num_equations) {
            m_num_equations = prop.size();
        } else {
            checkPropertySize(prop);
        }
    }
    void checkPropertySize(const AbstractProperty& prop) {
        if (!prop.empty()) {
            OPENSIM_THROW_IF(m_num_equations != prop.size(), Exception, "Size "
                "of property " + prop.getName() + " is not consistent with "
                "current number of constraint equations.");
        }
    }
    
};

// ============================================================================
// MucoMultibodyConstraint
// ============================================================================

/// The kinematic level for a scalar multibody constraint within a 
/// MucoMultibodyConstraint. Each scalar constraint is automatically assigned
/// a KinematicLevel enum value when a MucoMultibodyConstraint is 
/// instantiated.
enum class KinematicLevel {
    Position,
    DtPosition,
    Velocity,
    DtDtPosition,
    DtVelocity,
    Acceleration
};

/// A model multibody constraint to be enforced in the optimal control problem. 
/// Objects of this class can only be instantiated by a MucoPhase, since 
/// information from each constraint in the model is required to ensure that 
/// the correct values are assigned to internal variables during construction. 
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoMultibodyConstraint {
public:
    const MucoConstraintInfo& getConstraintInfo() const 
    {   return m_constraint_info; }
    void setConstraintInfo(const MucoConstraintInfo& cInfo) {
        OPENSIM_THROW_IF(cInfo.getNumEquations() != 
            m_constraint_info.getNumEquations(), Exception, "Size of "
            "properties in constraint info passed are not consistent with the "
            "number of scalar constraint equations in this multibody "
            "constraint.");
        m_constraint_info = cInfo; 
    }

    /// Get the SimTK::ConstraintIndex associated with this 
    /// MucoMultibodyConstraint. Note that a SimTK::ConstraintIndex is different 
    /// from what is returned from MucoPathConstraint::getPathConstraintIndex(): 
    /// the former is an index to a model constraint, whereas the latter is an 
    /// index to the path constraint errors vector in a MucoProblem.
    SimTK::ConstraintIndex getSimbodyConstraintIndex() const
    {   return m_simbody_constraint_index; }
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
    /// Get a vector of enums specifying the kinematic level of each scalar
    /// constraint equation in the associated model constraint, as each equation 
    /// may need to be treated differently in a solver (e.g. don't add Lagrange 
    /// multipliers for derivatives of position or velocity constraint equations 
    /// when looping through all scalar 
    /// constraint equations).
    std::vector<KinematicLevel> getKinematicLevels() const
    {   return m_kinematic_levels; }

    /// Convenience method for calculating constraint errors given a 
    /// SimTK::State object. This may not be the most efficient solution for 
    /// solvers, but could be useful for a quick implementation or for 
    /// debugging model constraints causing issues in an optimal control 
    /// problem. 
    /// @precondition The errors vector passed must be consistent with the 
    /// number of scalar constraint equations associated with this 
    /// MucoMultibodyConstraint.
    void calcMultibodyConstraintErrors(const Model& model, 
        const SimTK::State& state, SimTK::Vector& errors) const;

private:
    MucoConstraintInfo m_constraint_info;
    int m_num_position_eqs;
    int m_num_velocity_eqs;
    int m_num_acceleration_eqs;
    SimTK::ConstraintIndex m_simbody_constraint_index;
    std::vector<KinematicLevel> m_kinematic_levels;

    /// The constructor for this class is private since we don't want the user
    /// constructing these infos directly. Rather, they should be constructed
    /// directly from the model within the initialization for each MucoPhase,
    /// which is a friend of this class. This implementation ensures that the
    /// correct information is passed to set the default property values, which 
    /// should be sufficient for most users. 
    MucoMultibodyConstraint(SimTK::ConstraintIndex cid, int mp, int mv, int ma);
    friend class MucoProblemRep;
};

// ============================================================================
// MucoPathConstraint
// ============================================================================

/// A path constraint to be enforced in the optimal control problem.
/// @par For developers
/// Every time the problem is solved, a copy of this constraint is used. An individual
/// instance of a constraint is only ever used in a single problem. Therefore, there
/// is no need to clear cache variables that you create in initializeImpl().
/// Also, information stored in this constraint does not persist across multiple
/// solves.
/// @ingroup mucoconstraint
class OSIMMUSCOLLO_API MucoPathConstraint : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MucoPathConstraint, Object);
public:
    MucoPathConstraint();
        
    const MucoConstraintInfo& getConstraintInfo() const 
    {   return get_MucoConstraintInfo(); }
    MucoConstraintInfo& updConstraintInfo() 
    {   return upd_MucoConstraintInfo(); }
    void setConstraintInfo(const MucoConstraintInfo& cInfo) 
    {   set_MucoConstraintInfo(cInfo); }
    /// For use by solvers. This index is the location of this 
    /// MucoPathConstraint's first error in the MucoProblem's full path 
    /// constraint errors vector. Since it is set by the MucoProblem, it is only
    /// available after initialization.
    int getPathConstraintIndex() const {   
        OPENSIM_THROW_IF(m_path_constraint_index == -1, Exception,
            "Path constraint index is not available until after "
            "initialization.");
        return m_path_constraint_index; 
    }

    /// Calculate errors in the path constraint equations. The *errors* argument 
    /// represents the concatenated error vector for all path constraints in the 
    /// MucoProblem. This method creates a view into *errors* to access the 
    /// elements for this MucoPathConstraint and passes this view to
    /// calcPathConstraintErrorsImpl().
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
    void initializeOnModel(const Model& model,
            const int& pathConstraintIndex) const;
    
protected:
    OpenSim_DECLARE_UNNAMED_PROPERTY(MucoConstraintInfo, "The bounds and "
        "labels for this MucoPathConstraint.");

    /// Perform any caching.
    /// The number of scalar constraint equations this MucoPathConstraint 
    /// implements must be defined here (see setNumEquations() below).
    /// Upon entry, getModel() is available.
    /// The passed-in model is equivalent to getModel().
    /// Use this opportunity to check for errors in user input, in addition to
    /// the checks provided in initialize().
    virtual void initializeOnModelImpl(const Model&) const = 0;
    /// Set the number of scalar equations for this MucoPathConstraint. This 
    /// must be set within initializeImpl(), otherwise an exception is thrown
    /// during initialization.
    void setNumEquations(int numEqs) const {
        // TODO avoid const_cast
        const_cast<MucoPathConstraint*>(this)
            ->updConstraintInfo().setNumEquations(numEqs);
    }
    /// @precondition The state is realized to SimTK::Stage::Position.
    /// If you need access to the controls, you must realize to Velocity:
    /// @code
    /// getModel().realizeVelocity(state);
    /// @endcode
    virtual void calcPathConstraintErrorsImpl(const SimTK::State& state,
        SimTK::Vector& errors) const = 0;
    /// For use within virtual function implementations.
    const Model& getModel() const {
        OPENSIM_THROW_IF(!m_model, Exception,
            "Model is not available until the start of initializing.");
        return m_model.getRef();
    }  

private: 
   void constructProperties();

   mutable SimTK::ReferencePtr<const Model> m_model;
   mutable int m_path_constraint_index = -1;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOCONSTRAINT_H
