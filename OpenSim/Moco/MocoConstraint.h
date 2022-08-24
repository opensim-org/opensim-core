#ifndef OPENSIM_MOCOCONSTRAINT_H
#define OPENSIM_MOCOCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoConstraint.h                                                  *
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

#include "MocoBounds.h"
#include "MocoUtilities.h"
#include "MocoConstraintInfo.h"
#include "osimMocoDLL.h"

#include <simbody/internal/Constraint.h>

#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MocoProblemInfo;


// ============================================================================
// MocoKinematicConstraint
// ============================================================================

#ifndef SWIG
// TODO: Temporarily avoiding MSVC error:
// python_moco_wrap.cxx(13610): error C3431: 'KinematicLevel': a scoped
// enumeration cannot be redeclared as an unscoped enumeration
/// The kinematic level for a scalar kinematic constraint within a
/// MocoKinematicConstraint. Each scalar constraint is automatically assigned
/// a KinematicLevel enum value when a MocoKinematicConstraint is
/// instantiated.
enum class KinematicLevel {
    Position,
    DtPosition,
    Velocity,
    DtDtPosition,
    DtVelocity,
    Acceleration
};
#endif

/// A model kinematic constraint to be enforced in the optimal control problem.
/// Objects of this class can only be instantiated by a MocoPhase, since
/// information from each constraint in the model is required to ensure that
/// the correct values are assigned to internal variables during construction.
class OSIMMOCO_API MocoKinematicConstraint {
public:
    const MocoConstraintInfo& getConstraintInfo() const {
        return m_constraint_info;
    }
    void setConstraintInfo(const MocoConstraintInfo& cInfo) {
        OPENSIM_THROW_IF(
                cInfo.getNumEquations() != m_constraint_info.getNumEquations(),
                Exception,
                "Size of "
                "properties in constraint info passed are not consistent with "
                "the "
                "number of scalar constraint equations in this multibody "
                "constraint.");
        m_constraint_info = cInfo;
    }

    /// Get the SimTK::ConstraintIndex associated with this
    /// MocoKinematicConstraint. Note that a SimTK::ConstraintIndex is different
    /// from what is returned from MocoPathConstraint::getPathConstraintIndex():
    /// the former is an index to a model constraint, whereas the latter is an
    /// index to the path constraint errors vector in a MocoProblem.
    SimTK::ConstraintIndex getSimbodyConstraintIndex() const {
        return m_simbody_constraint_index;
    }
    /// Get the number of scalar constraint equations at each kinematic level.
    /// Note that the total number of scalar constraint equations enforced is
    /// *NOT* equal to the sum of each of these values -- you must include the
    /// first and second derivatives of the position equations and the first
    /// derivatives of the velocity equations into that count as well (this
    /// value can be obtained by calling getNumEquations()).
    int getNumPositionEquations() const { return m_num_position_eqs; }
    int getNumVelocityEquations() const { return m_num_velocity_eqs; }
    int getNumAccelerationEquations() const { return m_num_acceleration_eqs; }
    /// Get a vector of enums specifying the kinematic level of each scalar
    /// constraint equation in the associated model constraint, as each equation
    /// may need to be treated differently in a solver (e.g. don't add Lagrange
    /// multipliers for derivatives of position or velocity constraint equations
    /// when looping through all scalar
    /// constraint equations).
    std::vector<KinematicLevel> getKinematicLevels() const {
        return m_kinematic_levels;
    }

    /// Convenience method for calculating constraint errors given a
    /// SimTK::State object. This may not be the most efficient solution for
    /// solvers, but could be useful for a quick implementation or for
    /// debugging model constraints causing issues in an optimal control
    /// problem.
    /// @precondition The errors vector passed must be consistent with the
    /// number of scalar constraint equations associated with this
    /// MocoKinematicConstraint.
    void calcKinematicConstraintErrors(const Model& model,
            const SimTK::State& state, SimTK::Vector& errors) const;

private:
    MocoConstraintInfo m_constraint_info;
    int m_num_position_eqs;
    int m_num_velocity_eqs;
    int m_num_acceleration_eqs;
    SimTK::ConstraintIndex m_simbody_constraint_index;
    std::vector<KinematicLevel> m_kinematic_levels;

    /// The constructor for this class is private since we don't want the user
    /// constructing these infos directly. Rather, they should be constructed
    /// directly from the model within the initialization for each MocoPhase,
    /// which is a friend of this class. This implementation ensures that the
    /// correct information is passed to set the default property values, which
    /// should be sufficient for most users.
    MocoKinematicConstraint(SimTK::ConstraintIndex cid, int mp, int mv, int ma);
    friend class MocoProblemRep;
};

// ============================================================================
// MocoPathConstraint
// ============================================================================

/** A path constraint to be enforced in the optimal control problem.
The use of 'path' here is unrelated to muscle paths, GeometryPath,
or file system paths (e.g., Path).
@par For developers
Every time the problem is solved, a copy of this constraint is used. An
individual instance of a constraint is only ever used in a single problem.
Therefore, there is no need to clear cache variables that you create in
initializeImpl(). Also, information stored in this constraint does not
persist across multiple solves.
@ingroup mocopathcon */
class OSIMMOCO_API MocoPathConstraint : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoPathConstraint, Object);

public:
    MocoPathConstraint();

    const MocoConstraintInfo& getConstraintInfo() const {
        return get_MocoConstraintInfo();
    }
    MocoConstraintInfo& updConstraintInfo() { return upd_MocoConstraintInfo(); }
    void setConstraintInfo(const MocoConstraintInfo& cInfo) {
        set_MocoConstraintInfo(cInfo);
    }
    /** For use by solvers. This index is the location of this
    MocoPathConstraint's first error in the MocoProblem's full path
    constraint errors vector. Since it is set by the MocoProblem, it is only
    available after initialization. */
    int getPathConstraintIndex() const {
        OPENSIM_THROW_IF(m_path_constraint_index == -1, Exception,
                "Path constraint index is not available until after "
                "initialization.");
        return m_path_constraint_index;
    }

    /** Calculate errors in the path constraint equations. The *errors* argument
    represents the error vector for this MocoPathConstraint. The errors vector
    is passed to calcPathConstraintErrorsImpl(), which is defined by derived
    classes.
    @precondition initializeOnModel() has been invoked. */
    void calcPathConstraintErrors(
            const SimTK::State& state, SimTK::Vector& errors) const {
        calcPathConstraintErrorsImpl(state, errors);
    }

    /** Calculate errors in the path constraint equations. The *errors* argument
    represents the concatenated error vector for all path constraints in the
    MocoProblem. This method creates a view into *errors* to access the
    elements for this MocoPathConstraint and passes this view to
    calcPathConstraintErrorsImpl().
    @precondition initializeOnModel() has been invoked. */
    void calcPathConstraintErrorsView(
            const SimTK::State& state, SimTK::Vector& errors) const {

        // This vector shares writable, borrowed space from the *errors* vector
        // (provided by the MocoProblem) to the elements for which this
        // MocoPathConstraint provides constraint error information.
        SimTK::Vector theseErrors(getConstraintInfo().getNumEquations(),
                errors.updContiguousScalarData() + getPathConstraintIndex(),
                true);
        calcPathConstraintErrorsImpl(state, theseErrors);
    }

    /** Perform error checks on user input for this constraint, and cache
    quantities needed when computing the constraint errors.
    to efficiently evaluate the constraint.
    This function must be invoked before invoking
    calcPathConstraintErrors(). */
    void initializeOnModel(const Model& model, const MocoProblemInfo&,
            const int& pathConstraintIndex) const;

    /// Print the description for this path constraint.
    void printDescription() const;

protected:
    OpenSim_DECLARE_UNNAMED_PROPERTY(MocoConstraintInfo,
            "The bounds and labels for this MocoPathConstraint.");

    /// Perform any caching.
    /// The number of scalar constraint equations this MocoPathConstraint
    /// implements must be defined here (see setNumEquations() below).
    /// @precondition The model is initialized (initSystem()) and getModel()
    /// is available.
    /// The passed-in model is equivalent to getModel().
    /// Use this opportunity to check for errors in user input, in addition to
    /// the checks provided in initialize().
    virtual void initializeOnModelImpl(
            const Model&, const MocoProblemInfo&) const = 0;
    /// Set the number of scalar equations for this MocoPathConstraint. This
    /// must be set within initializeImpl(), otherwise an exception is thrown
    /// during initialization.
    void setNumEquations(int numEqs) const {
        // TODO avoid const_cast
        const_cast<MocoPathConstraint*>(this)
                ->updConstraintInfo()
                .setNumEquations(numEqs);
    }
    /// @precondition The state is realized to SimTK::Stage::Position.
    /// If you need access to the controls, you must realize to Velocity:
    /// @code
    /// getModel().realizeVelocity(state);
    /// @endcode
    virtual void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const = 0;
    /// Print a more detailed description unique to each path constraint.
    virtual void printDescriptionImpl() const {};
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

#endif // OPENSIM_MOCOCONSTRAINT_H
