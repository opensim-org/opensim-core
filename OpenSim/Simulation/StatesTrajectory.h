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
 * Copyright (c) 2016 Stanford University and the Authors                     *
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
#include <Simbody.h>

#include <OpenSim/Common/Exception.h>

#include "osimSimulationDLL.h"

namespace OpenSim {

class Storage;
class Model;

// Design note: This class is part of OpenSim instead of Simbody since Simbody
// users are likely to be interested in a more general State container that
// doesn't assume the states are sequential in time.

/** A sequence of SimTK::State%s that can be saved to a plain-text OSTATES
 * file.  The trajectory can also be populated from such a file. You may obtain
 * a StatesTrajectory from a simulation, or other numerical methods whose task
 * is to produce a trajectory of states. Users can modify the trajectory by
 * appending states to it, but users cannot modify the individual states that
 * are already in the trajectory.
 *
 * This class was introduced in OpenSim version 4.0, and enables scripting
 * (Python/MATLAB) and C++ users to postprocess their results with greater ease
 * and more versatility than with an Analysis.
 *
 * ### Guarantees
 * This class is designed to ensure the following:
 * - The states are ordered nondecreasing in time (adjacent states *can* have
 *   the same time).
 * - All states in the trajectory are consistent with each other (see
 *   isConsistent()).
 *
 * @note These gaurantees apply when using this class through C++, Java,
 * or the %OpenSim GUI, but **not** through Python or MATLAB. This is because
 * Python and MATLAB do not enforce constness and thus allow modifying the
 * trajectory.
 *
 * ### Using with a %Model 
 * A StatesTrajectory is not very useful on its own, since neither the
 * trajectory nor the contained states know of the names that the Component%s
 * give to the state variables. You probably want to use the trajectory with an
 * OpenSim::Model, through which the state variables have a meaning (e.g.,
 * `model.getStateVariableValue(states[0], "soleus_r/activation")`).
 *
 * SimTK::State%s have a tight association with a specific OpenSim::Model
 * (actually, with the SimTK::System within an OpenSim::Model). However,
 * neither the StatesTrajectory nor the OSTATES file knows the model to which
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
 *
 * ### Usage
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
 *
 * Here's an example of iterating from a state at `timeA` to the last state
 * before `timeB`:
 * @code{.cpp}
 * for (const auto& state : SimTK::makeIteratorRange(
 *                              states.getIteratorAt(timeA),
 *                              states.getIteratorBefore(timeB)) {
 *     ...
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
            OPENSIM_THROW(IndexOutOfRange, index, 0, m_states.size() - 1);
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


    /// @name Accessing states by time
    /// @{
    /** Get the index of the state just before (or at) the given time.
     *
     * The tolerance exists to handle imprecise
     * times. For example, if the trajectory contains states with times 1.3
     * and 1.5 and you ask for the index of the state before time 1.4, you'll
     * normally get the state with time 1.3. However, if you set a tolerance of
     * 0.15, then you'll get the state with time 1.5 (because it is the state
     * just before 1.55 = 1.4 + 0.15).
     *
     * It is unlikely you'll need to use this tolerance, but it is useful
     * in cases where you observe the state times in less than full precision
     * (perhaps in console output or storage files). Consider this scenario:
     * the in-memory time of a state is actually 1.500001 but it is printed to
     * the console as 1.500; you might be tempted to get this state by asking
     * for the one that is before/at time 1.5, but this would end up giving you
     * the state at time 1.3. You could remedy this issue by setting the
     * tolerance to 1e-3 (the precision of the console output).
     *
     * For a trajectory with times 0.2, 0.5, 0.7, this code will get the state
     * at time 0.5:
     * @code
     * const auto& state = states[states.getIndexBefore(0.6)];
     * @endcode
     *
     * @throws TimeOutOfRange If there are no states before the given time.
     * @throws Exception If the trajectory is empty.
     */
    size_t getIndexBefore(const double& time,
                          const double& tolerance = 0) const;
    /** Get the index of the state just after (or at) the given time.
     *
     * See getIndexBefore() for an explanation of the tolerance.
     *
     * For a trajectory with times 0.2, 0.5, 0.7, this code will get the state
     * at time 0.5:
     * @code
     * const auto& state = states[states.getIndexAfter(0.4)];
     * @endcode
     *
     * @throws TimeOutOfRange If there are no states after the given time.
     * @throws Exception If the trajectory is empty.
     * */
    size_t getIndexAfter(const double& time,
                         const double& tolerance = 0) const;

    /** Get the index of the state closest to the given time, within a given
     * tolerance. If there are multiple states exactly at the given time, the
     * index is that of the first of these states.
     *
     * The tolerance should be set to the precision of the time where you
     * observed it (perhaps in console output or storage files).
     *
     * For a trajectory with times 0.2, 0.501, 0.7, this code gives the index for
     * the state at time 0.5:
     * @code
     * states.getIndexAt(0.5, 0.01);
     * @endcode
     *
     * @throws NoStateAtTime If there is no state at the given time.
     * @throws Exception If the trajectory is empty.
     */
    size_t getIndexAt(const double& time, const double& tolerance = 0) const;
    /** Iterate over the states in a given time interval (inclusive).
     *
     * For a trajectory with times 0.2, 0.5, 0.7, 1.5, 1.8, this code will
     * iterate through the states at times 0.5, 0.7, 1.5:
     * @code
     * for (const auto& state : states.getBetween(0.5, 1.5)) {
     *     state.getTime();
     * }
     * @endcode
     *
     * The startTime must be less than or equal to the endTime, otherwise an
     * exception is thrown.
     *
     * See getIndexBefore() for an explanation of the tolerance.
     */
    IteratorRange getBetween(const double& startTime, const double& endTime,
            const double& tolerance = 0) const;
    /** Iterator pointing to the state just before (or at) the given time.
     *
     * For a trajectory with times 0.2, 0.5, 0.7, this code finds the state
     * at time 0.5:
     * @code
     * states.getIteratorBefore(0.6)->getTime();
     * @endcode
     *
     * If there are no states before the given time, then the returned iterator
     * matches end():
     * @code
     * auto it = states.getIteratorBefore(0.1);
     * if (it == states.end()) std::cout << "No states before." << std::endl;
     * @endcode
     *
     * See getIndexBefore() for an explanation of the tolerance.
     * */
    const_iterator getIteratorBefore(const double& time,
                                     const double& tolerance = 0) const;
    /** Iterator pointing to the state just after (or at) the given time.
     *
     * For a trajectory with times 0.2, 0.5, 0.7, this code finds the state
     * at time 0.5:
     * @code
     * states.getIteratorAfter(0.4)->getTime();
     * @endcode
     *
     * If there are no states after the given time, then the returned iterator
     * matches end():
     * @code
     * auto it = states.getIteratorAfter(0.9);
     * if (it == states.end()) std::cout << "No states after." << std::endl;
     * @endcode
     *
     * See getIndexBefore() for an explanation of the tolerance.
     * */
    const_iterator getIteratorAfter(const double& time,
                                    const double& tolerance = 0) const;
    /** Iterator pointing to the state at the given time, within a given
     * tolerance. If there are multiple states exactly at the given time,
     * the iterator points to the first of these states.
     *
     * The tolerance should be set to the precision of the time where you
     * observed it (perhaps in console output or storage files).
     *
     * For a trajectory with times 0.2, 0.501, 0.7, this code finds the state
     * at time 0.5:
     * @code
     * states.getIteratorAt(0.5, 0.01)->getTime();
     * @endcode
     *
     * If there is no state at the given time (within the tolerance), then the
     * returned iterator matches end():
     * @code
     * auto it = states.getIteratorAt(2.5);
     * if (it == states.end()) std::cout << "No state at time." << std::endl;
     * @endcode
     */
    const_iterator getIteratorAt(const double& time,
                                 const double& tolerance = 0) const;
    /// @}

    /// @name Populating the trajectory with states
    /// @{
    /** Append a SimTK::State to this trajectory.
     * This function ensures that the time in the new SimTK::State is
     * greater than the time in the last SimTK::State in the trajectory.
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
     * Returns true if the trajectory isConsistent() and if the following
     * quantities are the same:
     * - number of model state variables and number of Y's in the state 
     * - number of coordinates in the model and number of Q's in state
     * - number of speeds in the model and number of U's in state
     *
     * Returns false otherwise. This method **cannot** gaurantee that the
     * trajectory will work with the given model, and makes no attempt to
     * determine if the trajectory was generated with the given model.
     */
    bool isCompatibleWith(const Model& model) const;
    /// @}

private:

    /** Checks if two states have the same number of state variables,
     * constraints, etc. Returns true if the following quantities are the same
     * for both states:
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
     *
     * This method does NOT check for any relationship between the times in the
     * two states.
     */
    static bool isConsistent(const SimTK::State& stateA,
                             const SimTK::State& stateB);

    // Helper for accessing states by time.
    const_iterator lowerBound(const double& time) const;

    std::vector<SimTK::State> m_states;

public:

    /** Thrown when asking for a state at a time that is out of range. */
    class TimeOutOfRange : public OpenSim::Exception {
    public:
        TimeOutOfRange(const std::string& file, size_t line,
                const std::string& func,
                const double& requestedTime, const std::string& sense,
                const double& firstTime, const double& lastTime) :
                OpenSim::Exception(file, line, func) {
            std::ostringstream msg;
            msg << "There are no states with a time " << sense << " " <<
                requestedTime << "; the range of times is [" <<
                firstTime << ", " << lastTime << "].";
            addMessage(msg.str());
        }
    };

    /** Thrown when asking for a state at a time at which there is no state. */
    class NoStateAtTime : public OpenSim::Exception {
    public:
        NoStateAtTime(const std::string& file, size_t line,
                const std::string& func,
                const double& requestedTime, const double& tolerance) {
            std::ostringstream msg;
            auto fullPrecision = std::numeric_limits<double>::max_digits10;
            msg << std::setprecision(fullPrecision) <<
                "There are no states at the requested time of " <<
                requestedTime << " using the provided tolerance of " <<
                tolerance << ".";
            addMessage(msg.str());
        }
    };

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

    /** Thrown when trying to create a StatesTrajectory from a states Storage, and
     * the Storage does not contain a column for every continuous state variable.
     * */
    class MissingColumnsInStatesStorage : public OpenSim::Exception {
    public:
        MissingColumnsInStatesStorage(const std::string& file, size_t line,
                const std::string& func,
                const std::string& modelName,
                std::vector<std::string> missingStates) :
                OpenSim::Exception(file, line, func) {
            std::string msg = "The following ";
            msg += std::to_string(missingStates.size()) + " states from Model '";
            msg += modelName + "' are missing from the states Storage:\n";
            for (int i = 0; i < (missingStates.size() - 1); ++i) {
                msg += "    " + missingStates[i] + "\n";
            }
            msg += "    " + missingStates.back();
    
            addMessage(msg);
        }
    };
    
    /** Thrown when trying to create a StatesTrajectory from a states Storage, and
     * the Storage contains columns that do not correspond to continuous state
     * variables.
     * */
    class ExtraColumnsInStatesStorage : public OpenSim::Exception {
    public:
        ExtraColumnsInStatesStorage(
                const std::string& file, size_t line,
                const std::string& func,
                const std::string& modelName,
                std::vector<std::string> extraStates) :
                OpenSim::Exception(file, line, func) {
            std::string msg = "The following ";
            msg += std::to_string(extraStates.size()) + " columns from the ";
            msg += "states Storage are not states in Model '" + modelName + "':\n";
            for (int i = 0; i < (extraStates.size() - 1); ++i) {
                msg += "    " + extraStates[i] + "\n";
            }
            msg += "    " + extraStates.back();
    
            addMessage(msg);
        }
    };
    
    /** Thrown by createFromStatesStorage(). */
    class NonUniqueColumnsInStatesStorage : public OpenSim::Exception {
    public:
        NonUniqueColumnsInStatesStorage(const std::string& file, size_t line,
                const std::string& func) : OpenSim::Exception(file, line, func) {
            addMessage("States Storage column labels are not unique.");
        }
    };
    
    /** Thrown by createFromStatesStorage(). */
    class StatesStorageIsInDegrees : public OpenSim::Exception {
    public:
        StatesStorageIsInDegrees(const std::string& file, size_t line,
                const std::string& func) : OpenSim::Exception(file, line, func) {
            addMessage("States Storage is in degrees, but this is inappropriate "
                    "for creating a StatesTrajectory. Edit the Storage so that "
                    "angles are in radians, and set 'inDegrees' to "
                    "no in the header.");
        }
    };
    
    /** Thrown by createFromStatesStorage(). */
    class VaryingNumberOfStatesPerRow : public OpenSim::Exception {
    public:
        VaryingNumberOfStatesPerRow(const std::string& file, size_t line,
                const std::string& func,
                int numDepColumns, int smallestNumStates) :
                    OpenSim::Exception(file, line, func) {
            std::string msg = "States Storage has varying number of entries ";
            msg += "per row (from " + std::to_string(smallestNumStates) + " to ";
            msg += std::to_string(numDepColumns) + "). You must provide a ";
            msg += "States Storage that has the same number ";
            msg += "of entires in every row.";
            addMessage(msg);
        }
    };

    /// @name Create partial trajectory from pre-4.0 files
    /// @{
    /** Create a partial trajectory of States from a (pre-4.0) states Storage
     * object. The resulting StatesTrajectory will restore continuous state
     * variable values, but not discrete state variable values, modeling
     * option values, etc. Also, keep in mind that states Storage files usually
     * do not contain the state values to full precision, and thus cannot
     * exactly reproduce results from the initial state trajectory; constraints
     * may not be satisfied, etc.
     * 
     * #### History
     * Before OpenSim 4.0, the only way to save states to a file was as
     * a Storage file, typically called a states storage file and named
     * `*_states.sto`. You can use this function to create a StatesTrajectory
     * from such a Storage file. OpenSim 4.0 introduced the ability to save and
     * read a complete StatesTrajectory to/from an OSTATES file, and so this
     * function should only be used when you are stuck with pre-4.0 files.
     *
     * @note The naming convention for state variables changed in OpenSim v4.0;
     * `ankle_r/ankle_angle_r/speed` used to be `ankle_angle_r_u`,
     * `soleus_r/activation` used to be `soleus_r.activation`, etc. This
     * function can handle %Storage column labels that use the pre-v4.0 naming
     * convention.
     * 
     * @param model The Model to which the states belong. A Model is necessary
     *      since the storage file lists the state variables by name.
     * @param sto The Storage object containing state values.
     * @param allowMissingColumns If false, throws exception if there are
     *      continuous state variables in the Model for which there is no
     *      column in the Storage. If true, no exception is thrown but such
     *      state variables are set to NaN.
     * @param allowExtraColumns If false, throws exception if there are
     *      columns in the Storage that do not correspond to continuous state
     *      variables in the Model. If true, such columns of the Storage are
     *      ignored.
     *
     * #### Usage
     * Here is how you might use this function in python:
     * @code{.py}
     * import opensim
     * model = opensim.Model("subject01.osim")
     * model.initSystem()
     * sto = opensim.Storage("subject01_states.sto")
     * states = opensim.StatesTrajectory.createFromStatesStorage(model, sto)
     * print(states[0].getTime())
     * print(model.getStateVariableValue(states[0], "knee/flexion/value"))
     * @endcode
     * 
     * @throws MissingColumnsInStatesStorage See the description of the
     *      `allowMissingColumns` argument.
     * 
     * @throws ExtraColumnsInStatesStorage See the description of the
     *      `allowExtraColumns` argument.
     *
     * @throws NonUniqueColumnsInStatesStorage Thrown if multiple columns in
     *      the Storage have the same name.
     * 
     * @throws StatesStorageIsInDegrees Thrown if the Storage is in degrees
     *      (inDegrees=yes); angular quantities must use radians to properly
     *      create the trajectory.
     *
     * @throws VaryingNumberOfStatesPerRow Thrown if the rows of the storage
     *      don't all have the same number of entries.
     */
    static StatesTrajectory createFromStatesStorage(const Model& model,
            const Storage& sto,
            bool allowMissingColumns = false,
            bool allowExtraColumns = false);

    /** Convenience form of createFromStatesStorage() that takes the path to a
     * Storage file instead of a Storage object. This convenience form uses the
     * default values for `allowMissingColumns` and `allowExtraColumns`. */
    static StatesTrajectory createFromStatesStorage(const Model& model,
            const std::string& filepath);
    /// @}
};

} // namespace

#endif // OPENSIM_STATES_TRAJECTORY_H_
