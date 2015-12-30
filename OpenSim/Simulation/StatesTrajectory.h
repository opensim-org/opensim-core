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
 * Copyright (c) 2015 Stanford University and the Authors                     *
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
        std::string msg = "The following are states in Model '" + modelName;
        msg += "' but are missing from the ";
        msg += "states Storage: ";
        for (int i = 0; i < (missingStates.size() - 1); ++i) {
            msg += missingStates[i] + ", ";
        }
        msg += missingStates[missingStates.size() - 1] + ".";

        // TODO message should account for pre-v4.0 column names.
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
        std::string msg = "The following columns from the states Storage ";
        msg += "are not states in Model '" + modelName + "': ";
        for (int i = 0; i < (extraStates.size() - 1); ++i) {
            msg += extraStates[i] + ", ";
        }
        msg += extraStates[extraStates.size() - 1] + ".";

        // TODO message should account for pre-v4.0 column names.
        addMessage(msg);
    }
};

class NonUniqueColumnsInStatesStorage : public OpenSim::Exception {
    using Exception::Exception;
};

// This class is part of OpenSim instead of Simbody since Simbody users are
// likely to be interested in a more general State container that doesn't
// assume the states are sequential in time.

/** A sequence of SimTK::State%s that can be saved to a plain-text file. The
 * trajectory can also be populated from such a file. The states within the
 * trajectory should be sequential in time, though this is only weakly
 * enforced. You may obtain a StatesTrajectory from a simulation, or other
 * numerical methods whose task is to produce a trajectory of states.
 *
 * This class was introduced in OpenSim version 4.0, and largely replaces the
 * need for @ref Analysis "Analyses".
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
 * neither the StatesTrajectory nor the OST file know the model to which it
 * corresponds. So, for example, you could use a single StatesTrajectory with a
 * generic gait2392 model as well as with a scaled (subject-specific) gait2392
 * model. This flexibility may be beneficial in some scenarios, but also allows
 * one to accidentally use the wrong model with a given states trajectory,
 * potentially leading to silent errors that could compromise a scientific
 * study.
 *
 * To increase your confidence that a StatesTrajectory matches a given Model,
 * you can perform some (weak) checks with compatibleWith().
 *
 * TODO acceleration-level calculations?
 * TODO before making use of the states, model must be initSystem(), also none
 * of the cache exists, so you might need to realize the model to certain
 * levels.
 *
 * ### File format
 * StatesTrajectory files use the file extension `.OST`, and use the XML
 * format. Therefore, you can edit a StatesTrajectory file in a typical text
 * editing program, or in Python/MATLAB using XML libraries. However, the
 * easiest way to modify an OST file is to load it as a StatesTrajectory object
 * (in C++, Python, MATLAB), modify the StatesTrajectory object, and write it
 * to an OST again. This only allows limited types of modification (appending
 * SimTK::State%s, editing state variable values), and does not allow more
 * drastic modifications like removing or adding state variables in each
 * SimTK::State.
 *
 * A SimTK::State object contains many different types of data, but only some
 * are saved into the OST file:
 * 
 * type of data                 | saved in OST?
 * ---------------------------- | -------------
 * (continuous) state variables | yes
 * discrete state variables     | yes
 * modeling options             | yes
 * cache variables              | no
 *
 * The cache variables (e.g., total system mass, control signals) are not saved
 * to the OST file because they can be regenerated from the other data and the
 * model (TODO might want to save the cache variables, b/c some of them are
 * expensive to re-compute, e.g. with CMC).
 *
 * TODO easily can edit individual states to violate the sequential nature of
 * the states.
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
 * To do most things with the StatesTrajectory, you'll need a model. **It is
 * essential that you've called `Model::initSystem()` before you try to use any
 * states with the model**:
 * @code{.cpp}
 * Model model("subject01.osim");
 * model.initSystem();
 * double knee_angle = model.getStateVariableValue(states[0], "knee/flexion/value");
 * Vec3 com = model.calcMassCenterPosition(states[0]);
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
 * If you have a modifiable (non-const) trajectory, you could also modify
 * the individual states:
 * @code{.cpp}
 * for (auto& state : states) {
 *     model.setStateVariableValue(state, "knee/flexion/value", 1.0);
 * }
 * @endcode
 *
 */
class OSIMSIMULATION_API StatesTrajectory {
public:
    /** Create an empty trajectory of states. */
    StatesTrajectory() {}

    /** The number of SimTK::State%s in the trajectory. */
    size_t getSize() const;

    /// @name Accessing and modifying individual SimTK::State%s
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
    /** Get a modifiable reference to the state at a given index in the
     * trajectory.
     * Here's an example of setting a state variable value in the first state
     * in the trajectory.
     * @code{.cpp}
     * Model model("subject01.osim");
     * StatesTrajectory states = getStatesTrajectorySomehow();
     * auto& state = states[0];
     * // The following line modifies the first state in `states`:
     * model.setStateVariableValue(state, "knee/flexion/value", 0.5);
     * @endcode
     * This function does not check if the index is larger than the
     * size of the trajectory; see upd() if you want this check. */
    SimTK::State& operator[](size_t index) {
        return m_states[index];
    }

    /** Get a const reference to the state at a given index in the trajectory.
     *  
     * Use this instead of upd() if you don't want to accidentally edit the
     * provided state (though, in Python and MATLAB, both get() and upd() allow
     * you to modify the provided state.

     * @throws std::out_of_range if the index is greater than the size of the
     *                           trajectory.
     */
    const SimTK::State& get(size_t index) const {
        return m_states.at(index);
    }
    /** Get a modifiable reference to the state at a given index in the
     * trajectory.
     *
     * Use this instead of get() if you want to edit the provided state.
     * However, you can only use this method if your StatesTrajectory is
     * modifiable (non-const).
     *
     * @throws std::out_of_range if the index is greater than the size of the
     *                           trajectory.
     */
    SimTK::State& upd(size_t index) {
        return m_states.at(index);
    }
    /// @}
    
    /** Iterator type allowing modification of the trajectory. Most users do
     * not need to understand what this is. */
    typedef std::vector<SimTK::State>::iterator iterator;
    /** Iterator type that does not allow modification of the trajectory.
     * Most users do not need to understand what this is. */
    typedef std::vector<SimTK::State>::const_iterator const_iterator;

    /** @name Iterating through the trajectory
     * @{ */
    /** Iterator pointing to first SimTK::State, only available if the
     * trajectory is modifiable (non-const). */
    iterator begin() { return m_states.begin(); }
    /** Iterator pointing to the end of the trajectory, only available if the
     * trajectory is modifiable (non-const). */
    iterator end() { return m_states.end(); }

    /** Iterator pointing to first SimTK::State; does not allow modifying the
     * states. */
    const_iterator cbegin() const { return m_states.cbegin(); }
    /** Iterator pointing to end of the trajectory; does not allow modifying the
     * states. */
    const_iterator cend() const { return m_states.cend(); }
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
    /** TODO */
    // TODO should we check for consistency whenever appending?
    bool consistent() const;
    /** TODO */
    bool compatibleWith(const Model& model);
    /// @}

private:
    std::vector<SimTK::State> m_states;

public:
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
     * Before OpenSim 4.0, the only way to save states to a file was as
     * a Storage file, typically called a states storage file and named
     * `*_states.sto`. You can use this function to create a StatesTrajectory
     * from such a Storage file. OpenSim 4.0 introduced the ability to save and
     * read a complete StatesTrajectory to/from an XML file, and so this
     * function should only be used when you are stuck with pre-4.0 files.
     *
     * TODO backwards compatibility with pre-4.0 states storage files.
     * 
     * @param model The Model to which the states belong. A Model is necessary
     *      since the storage file lists the state variables by name.
     * @param sto   The Storage object containing state values.
     * @param allowMissingColumns If false, throws exception if there are
     *      continuous state variables in the Model for which there is no
     *      column in the Storage. If true, no exception is thrown but such
     *      state variables are set to NaN.
     * @param allowExtraColumns If false, throws exception if there are
     *      columns in the Storage that do not correspond to continuous state
     *      variables in the Model. If true, such columns of the Storage are
     *      ignored.
     * 
     * @throws MissingColumnsInStatesStorage See the description of the
     *      `allowMissingColumns` argument.
     * 
     * @throws ExtraColumnsInStatesStorage See the description of the
     *      `allowExtraColumns` argument.
     */
    // TODO assemble, equilibrateMuscles?
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
