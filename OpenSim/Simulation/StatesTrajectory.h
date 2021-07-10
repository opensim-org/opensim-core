#ifndef OPENSIM_STATES_TRAJECTORY_H_
#define OPENSIM_STATES_TRAJECTORY_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StatesTrajectory.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia                                                    *
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

#include <vector>

#include <OpenSim/Common/Exception.h>
#include <OpenSim/Common/TimeSeriesTable.h>
#include <SimTKcommon/internal/IteratorRange.h>

#include "osimSimulationDLL.h"

namespace SimTK {
class State;
}

namespace OpenSim {

class Storage;
class Model;

// Design note: This class is part of OpenSim instead of Simbody since Simbody
// users are likely to be interested in a more general State container that
// doesn't assume the states are sequential in time.

// TODO See the bottom of this file for a class description to use once the
// OSTATES file format is implemented.
//
/** 
 * \section StatesTrajectory
 * This class holds a sequence of SimTK::State%s. You can obtain a
 * StatesTrajectory during a simulation via the StatesTrajectoryReporter. You
 * can also create a StatesTrajectory from a states storage (.sto) file (see
 * createFromStatesStorage()). Users can modify a trajectory by appending
 * states to it, but users cannot modify the individual states that are already
 * in a trajectory.
 *
 * This class was introduced in OpenSim version 4.0, and enables scripting
 * (Python/MATLAB) and C++ users to postprocess their results with greater ease
 * and flexibility than with an Analysis.
 *
 * @note In a future release, we plan to support an OSTATES file format that
 * allows one to write the trajectory to a file with full numerical precision.
 *
 * \subsection st_guarantees Guarantees
 * This class is designed to ensure the following:
 * - The states are ordered nondecreasing in time (adjacent states *can* have
 *   the same time).
 * - All states in the trajectory are consistent with each other (see
 *   isConsistent()).
 *
 * @note These guarantees apply when using this class through C++, Java,
 * or the %OpenSim GUI, but **not** through Python or MATLAB. This is because
 * Python and MATLAB do not enforce constness and thus allow modifying the
 * trajectory.
 *
 * \subsection st_using_model Using with an OpenSim:: Model 
 * A StatesTrajectory is not very useful on its own, since neither the
 * trajectory nor the contained states know how the Component%s name the state
 * variables they create. You probably want to use the trajectory with an
 * OpenSim::Model, through which the state variables have a meaning (e.g.,
 * `model.getStateVariableValue(states[0], "soleus_r/activation")`).
 *
 * SimTK::State%s have a tight association with a specific OpenSim::Model
 * (actually, with the SimTK::System within an OpenSim::Model). However,
 * the StatesTrajectory does not know anything about the model to which
 * it corresponds. So, for example, you could use a single StatesTrajectory
 * with a generic gait2392 model as well as with a scaled (subject-specific)
 * gait2392 model. This flexibility may be beneficial in some scenarios, but
 * also allows one to accidentally use the wrong model with a given states
 * trajectory, potentially leading to silent errors that could compromise a
 * scientific study.
 *
 * To increase your confidence that a StatesTrajectory matches a given Model,
 * you can perform some weak checks with isCompatibleWith().
 *
 * \subsection st_usage Usage
 * Here are a few basic things you can do with a StatesTrajectory, assuming you
 * already have one:
 * @code{.cpp}
 * StatesTrajectory states = getStatesTrajectorySomehow();
 * const double numStates = states.getSize();
 * const double initialTime = states[0].getTime();
 * const double finalTime = states.back().getTime();
 * @endcode
 *
 * Without a model, you can access the state variable values, but you won't
 * know the identity of such state variables.
 * @code{.cpp}
 * int numGeneralizedCoordinates = states[0].getNQ();
 * const SimTK::Vector& generalizedCoordinateValues = states[0].getQ();
 * @endcode
 *
 * To do most things with the StatesTrajectory, you'll need a model as well as
 * its underlying SimTK::System. **It is therefore required that you call
 * `Model::initSystem()` before you try to use any states with the model**:
 * @code{.cpp}
 * Model model("subject01.osim");
 * model.initSystem();
 * double knee_angle = model.getStateVariableValue(states[0], "knee/flexion/value");
 * Vec3 com = model.calcMassCenterPosition(states[0]);
 * @endcode
 *
 * Depending on the quantity you want to obtain, you may also need to realize
 * the state to a certain stage:
 * @code{.cpp}
 * model.getMultibodySystem().realize(states[0], SimTK::Stage::Velocity);
 * model.getMuscles().get("soleus_r").getActivation(states[0]);
 * @endcode
 *
 * You can iterate through a trajectory to access the value of a state variable
 * at each time in the trajectory.
 * @code{.cpp}
 * for (const auto& state : states) {
 *     std::cout << state.getTime() << " "
 *               << model.getStateVariableValue(state, "knee/flexion/value")
 *               << std::endl;
 * }
 * @endcode
 */
class OSIMSIMULATION_API StatesTrajectory {
public:
    /** Create an empty trajectory of states. */
    StatesTrajectory() {}

    /** The number of SimTK::State%s in the trajectory. */
    size_t getSize() const;

    /// @name Accessing individual SimTK::State%s
    /// @{
    /** Get a const reference to the state at a given index in the trajectory.
     * Here's an example of getting a state variable value from the first state
     * in the trajectory: 
     * @code{.cpp}
     * Model model("subject01.osim");
     * const StatesTrajectory states = getStatesTrajectorySomehow();
     * const auto& state = states[0];
     * model.getStateVariableValue(state, "knee/flexion/value");
     * @endcode
     * This function does not check if the index is larger than the size of
     * the trajectory; see get() if you want this check. */
    const SimTK::State& operator[](size_t index) const {
        return m_states[index];
    }
    /** Get a const reference to the state at a given index in the trajectory.

     * @throws IndexOutOfRange If the index is greater than the size of the
     *                         trajectory.
     */
    const SimTK::State& get(size_t index) const {
        try {
            return m_states.at(index);
        } catch (const std::out_of_range&) {
            OPENSIM_THROW(IndexOutOfRange, index, 0, 
                          static_cast<unsigned>(m_states.size() - 1));
        }
    }
    /** Get a const reference to the first state in the trajectory. */
    const SimTK::State& front() const { 
        return m_states.front();
    }
    /** Get a const reference to the last state in the trajectory. */
    const SimTK::State& back() const { 
        return m_states.back();
    }
    /// @}
    
    /** Iterator type that does not allow modifying the trajectory.
     * Most users do not need to understand what this is. */
    typedef std::vector<SimTK::State>::const_iterator const_iterator;

    /** A helper type to allow using range for loops over a subset of the
     * trajectory. */
    typedef SimTK::IteratorRange<const_iterator> IteratorRange;

    /// @name Iterating through the trajectory
    /// @{

    /** Iterator pointing to first SimTK::State; does not allow modifying the
     * states. Allows using this class in a range for loop. */
    const_iterator begin() const { return m_states.cbegin(); }
    /** Iterator pointing past the end of the trajectory. Allows using this
     * class in a range for loop. */
    const_iterator end() const { return m_states.cend(); }
    /// @}

    /// @name Modify the contents of the trajectory
    /// @{
    /** Clear all the states in the trajectory. */
    void clear();
    /** Append a SimTK::State to this trajectory.
     * This function ensures that the time in the new SimTK::State is greater
     * than or equal to the time in the last SimTK::State in the trajectory.
     *
     * The state that ends up in the trajectory is a deep copy of the one
     * passed in.
     */
    void append(const SimTK::State& state);
    /// @}

    /// @name Checks for integrity
    /// @{

    /** Checks isNondecreasingInTime() and isConsistent().
     * The design of this class is such that this method should always return
     * true. This check may be more useful in Python or MATLAB, in which it's
     * possible to edit the trajectory such that this method could return
     * false.  */
    // TODO better name?
    bool hasIntegrity() const;

    /** Returns true if times are non-decreasing; false otherwise. */
    bool isNondecreasingInTime() const;

    /** Checks if the states have the same number of state variables,
     * constraints, etc. Returns true if the following quantities are the same
     * for all states in the trajectory:
     * - number of generalized coordinates (Q's)
     * - number of generalized speeds (U's)
     * - number of auxiliary state variables (Z's)
     * - number of position constraints (QErr's)
     * - number of velocity constraints (UErr's)
     * - number of acceleration constraints (UDotErr's)
     * - number of constraint Lagrange multipliers
     * - number of event triggers
     *
     * Returns false otherwise.
     */
    // TODO an option to throw an exception with detailed information about the
    // mismatch?
    bool isConsistent() const;

    /** Weak check for if the trajectory can be used with the given model.
     * Returns true if the trajectory isConsistent() and if the number of speeds
     * in the model matches the number of U's in state.
     *
     * Returns false otherwise. This method **cannot** guarantee that the
     * trajectory will work with the given model, and makes no attempt to
     * determine if the trajectory was generated with the given model.
     */
    bool isCompatibleWith(const Model& model) const;
    /// @}

    /** Export the continuous state variables to a data table, perhaps to write
     * to a file and postprocess in MATLAB/Python/Excel. The names of the
     * columns in the table will be the absolute names of the continuous state
     * variables (e.g., `knee/flexion/angle`).
     *
     * You must provide a model that is compatible with this trajectory,
     * since only the model knows the names of the state variables.
     *
     * By default, all continuous state variables are written to the table
     * (one per column). If you only want some of them to be written to the
     * table, use the `stateVars` argument to specify their absolute names
     * (e.g., `knee/flexion/angle`).
     *
     * @code
     * auto allStateVars = states.exportToTable(model);
     * auto kneeStates = states.exportToTable(model, {"knee/flexion/value",
     *                                                "knee/flexion/speed"});
     * @endcode
     *
     * @throws IncompatibleModel Thrown if the Model fails the check
     *      isCompatibleWith().
     *
     * See DataAdapter for details on writing to files.
     */
    TimeSeriesTable exportToTable(const Model& model,
            const std::vector<std::string>& stateVars = {}) const;

private:

    std::vector<SimTK::State> m_states;

public:

    /** Thrown when trying to append a state that is not consistent with the
     * rest of the trajectory. */
    class InconsistentState : public OpenSim::Exception {
    public:
        InconsistentState(const std::string& file, size_t line,
                const std::string& func, const double& stateTime) :
                OpenSim::Exception(file, line, func) {
            std::ostringstream msg;
            msg << "Cannot append the provided state (at time = " <<
                stateTime << " seconds) to the trajectory because it is " <<
                "inconsistent with the trajectory.";
            addMessage(msg.str());
        }
    };

    /** Thrown when trying to use a StatesTrajectory with an incompatible model.
     * See isCompatibleWith(). */
#ifndef SWIG
    class IncompatibleModel : public OpenSim::Exception {
    public:
        IncompatibleModel(const std::string& file, size_t line,
                          const std::string& func, const Model& model);
    };
#endif

    /** Thrown when trying to create a StatesTrajectory from states data,
     * and the data does not contain a column for every continuous state
     * variable. */
    class MissingColumns : public OpenSim::Exception {
    public:
        MissingColumns(const std::string& file, size_t line,
                const std::string& func,
                const std::string& modelName,
                std::vector<std::string> missingStates) :
                OpenSim::Exception(file, line, func) {
            std::string msg = "The following ";
            msg += std::to_string(missingStates.size()) + " states from Model '";
            msg += modelName + "' are missing from the data:\n";
            for (unsigned i = 0; i < (missingStates.size() - 1); ++i) {
                msg += "    " + missingStates[i] + "\n";
            }
            msg += "    " + missingStates.back();
    
            addMessage(msg);
        }
    };
    
    /** Thrown when trying to create a StatesTrajectory from states data, and
     * the data contains columns that do not correspond to continuous state
     * variables. */
    class ExtraColumns : public OpenSim::Exception {
    public:
        ExtraColumns(
                const std::string& file, size_t line,
                const std::string& func,
                const std::string& modelName,
                std::vector<std::string> extraStates) :
                OpenSim::Exception(file, line, func) {
            std::string msg = "The following ";
            msg += std::to_string(extraStates.size()) + " columns from the ";
            msg += "states Storage are not states in Model '" + modelName + "':\n";
            for (unsigned i = 0; i < (extraStates.size() - 1); ++i) {
                msg += "    " + extraStates[i] + "\n";
            }
            msg += "    " + extraStates.back();
    
            addMessage(msg);
        }
    };

    /** Thrown by createFromStatesStorage(). */
    class DataIsInDegrees : public OpenSim::Exception {
    public:
        DataIsInDegrees(const std::string& file, size_t line,
                const std::string& func) : OpenSim::Exception(file, line, func) {
            addMessage("States Storage is in degrees, but this is inappropriate "
                    "for creating a StatesTrajectory. Edit the Storage so that "
                    "angles are in radians, and set 'inDegrees' to "
                    "no in the header.");
        }
    };

    /// @name Create partial trajectory from a states table
    /// @{

    /**
     * This function is identical to createFromStatesTable() except that this
     * function accepts a Storage instead of a TimeSeriesTable.
     */
    // TODO When OSTATES support is complete, add the following blurb to
    // the doxygen block above.
    /* #### History
     * Before OpenSim 4.0, the only way to save states to a file was as
     * a Storage file, typically called a states storage file and named
     * `*_states.sto`. You can use this function to create a StatesTrajectory
     * from such a Storage file. OpenSim 4.0 introduced the ability to save and
     * read a complete StatesTrajectory to/from an OSTATES file, and so this
     * function should only be used when you are stuck with pre-4.0 files. */
    static StatesTrajectory createFromStatesStorage(const Model& model,
            const Storage& sto,
            bool allowMissingColumns = false,
            bool allowExtraColumns = false,
            bool assemble = false);

    /** Create a partial trajectory of States from a states table.
     * The resulting StatesTrajectory will restore continuous state
     * variable values, but not discrete state variable values, modeling
     * option values, etc. Also, keep in mind that states files usually
     * do not contain the state values to full precision, and thus cannot
     * exactly reproduce results from the initial state trajectory. Lastly,
     * this function optionally modifies each state to obey any constraints in
     * the model (by calling Model::assemble()).
     *
     * The states in the resulting trajectory will be realized to
     * SimTK::Stage::Instance. You should not use the resulting trajectory with
     * an instance of the model other than the one you passed to this function
     * (the state contains Instance-stage cache variables that are pointers to
     * objects in the model; e.g., force elements).
     *
     * @note The naming convention for state variables changed in OpenSim v4.0;
     * `ankle_r/ankle_angle_r/speed` used to be `ankle_angle_r_u`,
     * `soleus_r/activation` used to be `soleus_r.activation`, etc. This
     * function can handle column labels that use the pre-v4.0 naming
     * convention.
     *
     * @param model The Model to which the states belong. A Model is necessary
     *      because the data file lists the state variables by name.
     * @param table The table containing state values.
     * @param allowMissingColumns If false, throws exception if there are
     *      continuous state variables in the Model for which there is no
     *      column in the table. If true, no exception is thrown but such
     *      state variables are set to NaN.
     * @param allowExtraColumns If false, throws exception if there are
     *      columns in the table that do not correspond to continuous state
     *      variables in the Model. If true, such columns of the table are
     *      ignored.
     * @param assemble Modify state variable values to satisfy
     *      kinematic constraints (by calling Model::assemble()).
     *      Use this option if the provided states are incomplete (for example,
     *      the values for dependent coordinates are unspecified).
     *      Caution: enforcing constraints can drastically alter the
     *      provided states if they do not already obey the constraints.
     *      Do not use this option with results from a forward simulation: the
     *      states trajectory from a forward simulation may not meet the
     *      model's assembly accuracy, and therefore assembling could
     *      alter the trajectory and cause inconsistency between coordinate
     *      values and speeds.
     *
     * Here is how you might use this function in python:
     * @code{.py}
     * import opensim
     * model = opensim.Model("subject01.osim")
     * table = opensim.TimeSeriesTable("subject01_states.sto")
     * states = opensim.StatesTrajectory.createFromStatesTable(model, table)
     * print(states[0].getTime())
     * print(model.getStateVariableValue(states[0], "knee/flexion/value"))
     * @endcode
     *
     * @throws MissingColumns See the description of the
     *      `allowMissingColumns` argument.
     *
     * @throws ExtraColumns See the description of the
     *      `allowExtraColumns` argument.
     *
     * @throws NonUniqueLabels Thrown if multiple columns in
     *      the table have the same name.
     *
     * @throws DataIsInDegrees Thrown if the table is in degrees
     *      (inDegrees=yes); angular quantities must use radians to properly
     *      create the trajectory.
     */
    static StatesTrajectory createFromStatesTable(const Model& model,
            const TimeSeriesTable& table,
            bool allowMissingColumns = false,
            bool allowExtraColumns = false,
            bool assemble = false);

    /** Convenience form of createFromStatesStorage() that takes the path to a
     * Storage file instead of a Storage object. This convenience form uses the
     * default values for `allowMissingColumns` and `allowExtraColumns`. */
    static StatesTrajectory createFromStatesStorage(const Model& model,
            const std::string& filepath);
};

} // namespace

// TODO The following class description should be revisited when support for
// OSTATES files is completed.
/* This class holds a sequence of SimTK::State%s that can be saved to a
 * plain-text OSTATES file.  The trajectory can also be populated from such a
 * file. You may obtain a StatesTrajectory from a simulation, or other
 * numerical methods whose task is to produce a trajectory of states. Users can
 * modify the trajectory by appending states to it, but users cannot modify the
 * individual states that are already in the trajectory.
 *
 * This class was introduced in OpenSim version 4.0, and enables scripting
 * (Python/MATLAB) and C++ users to postprocess their results with greater ease
 * and more versatility than with an Analysis.
 *
 * @note This class is a work in progress. The ability to read and write a
 * StatesTrajectory to an OSTATES is not available yet, even though the
 * documentation is written as if this ability exists.
 *
 * ### Guarantees
 * This class is designed to ensure the following:
 * - The states are ordered nondecreasing in time (adjacent states *can* have
 *   the same time).
 * - All states in the trajectory are consistent with each other (see
 *   isConsistent()).
 *
 * @note These guarantees apply when using this class through C++, Java,
 * or the %OpenSim GUI, but **not** through Python or MATLAB. This is because
 * Python and MATLAB do not enforce constness and thus allow modifying the
 * trajectory.
 *
 * ### File format
 * StatesTrajectory files use the file extension `.ostates`, with the XML
 * format. Therefore, you could theoretically edit a StatesTrajectory file in
 * a typical text editing program, or in Python/MATLAB using XML libraries
 * (such a process is is likely to be painful; consider using the
 * createFromStatesStorage() utility instead).
 *
 * A SimTK::State object contains many different types of data, but only some
 * are saved into the OSTATES file:
 * 
 * type of data                 | saved in OSTATES?
 * ---------------------------- | -----------------
 * (continuous) state variables | yes
 * discrete state variables     | yes
 * modeling options             | yes
 * cache variables              | no
 *
 * The cache variables (e.g., total system mass, control signals) are not saved
 * to the OSTATES file because they can be computed from the state variables
 * and are not necessary for describing the state of the system (see
 * Component::addStateVariable).
 *
 * %OpenSim Object%s (Model OSIM files, Tool setup files) also use an
 * XML format, but that format is *completely unrelated* to the XML format used
 * for OSTATES files.
 */

#endif // OPENSIM_STATES_TRAJECTORY_H_
