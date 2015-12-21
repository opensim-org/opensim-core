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

// TODO doxygen, associate with StatesTrajectory class.
class StatesMissingFromStorage : OpenSim::Exception {
    using Exception::Exception;
};

class StatesMissingFromModel : OpenSim::Exception {
    using Exception::Exception;
};

// This class is part of OpenSim instead of Simbody since Simbody users are
// likely to be interested in a more general State container that doesn't
// assume the States are sequential in time.

/** TODO
 *
 * TODO before making use of the states, model must be initSystem(), also none
 * of the cache exists, so you might need to realize the model to certain
 * levels.
 *
 * TODO acceleration-level calculations?
 */
class OSIMSIMULATION_API StatesTrajectory {
public:
    /** Create an empty trajectory of states.
    */
    StatesTrajectory() {}

    /** The number of states in the trajectory. */
    size_t getSize() const;

    /// @name Accessing the states.
    /// @{
    /** Get a const reference to the state at a given index in the trajectory.
     * This function does not check if the index is larger than the size of
     * the trajectory; see get() if you want this check. */
    const SimTK::State& operator[](size_t index) const {
        return m_states[index];
    }
    /** Get a modifiable reference to the state at a given index in the
     * trajectory. This function does not check if the index is larger than the
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
    
    /// @name Iterating through the trajectory.
    /// @{
    // TODO
    typedef std::vector<SimTK::State>::iterator iterator;
    typedef std::vector<SimTK::State>::const_iterator const_iterator;

    // TODO example usage.
    iterator begin() { return m_states.begin(); }
    iterator end() { return m_states.end(); }

    // TODO example usage.
    const_iterator cbegin() const { return m_states.cbegin(); }
    const_iterator cend() const { return m_states.cend(); }
    /// @}

    /** Append a State to this trajectory.
     *
     * In Debug mode, this method ensures that the time in the new State is
     * greater than the time in the last state in the trajectory.
     */
    void append(const SimTK::State& state);
    // TODO void append(const StatesTrajectory& states);
   
    // TODO remove states?

    // TODO const SimTK::State& getNearest(const Real& time) const;

    /// @name Create trajectory using partial information
    /// @{
    /** Create a partial trajectory of States from a states Storage object. The
     * StatesTrajectory will contain continuous state variable values, but not
     * modeling options, etc. Also, keep in mind that states Storage files
     * usually do not contain the state to full precision, and thus cannot
     * exactly reproduce results from the initial state trajectory.
     * 
     * In OpenSim 3.3 and earlier, the only way to save states to a file was as
     * a Storage file, typically called a states storage file and named
     * `*_states.sto`. You can use this function to create a StatesTrajectory
     * from such a Storage file.
     *
     * TODO we expect the Storage file to specify each state for all time. what
     * if a certain variable is missing at a specific time?
     * 
     * @param model The Model to which the states belong. A Model is necessary
     *      since the storage file lists the state variables by name.
     * @param sto   The Storage object containing state values.
     * @param checkMissingFromStorage If true, throws exception if there are
     *      continuous state variables in the Model for which there is no
     *      column in the Storage. If false, such state variables are not set
     *      in the StatesTrajectory. TODO set to NaN.
     * @param checkMissingFromModel If true, throws exception if there are
     *      columns in the Storage that do not correspond to continuous state
     *      variables in the Model. If false, such columns of the Storage are
     *      ignored.
     * 
     * @throws StatesMissingFromStorage See the description of the
     *      `checkMissingFromStorage` argument.
     * 
     * @throws StatesMissingFromModel See the description of the
     *      `checkMissingFromModel` argument.
     */
    // TODO assemble, equilibrateMuscles?
    static StatesTrajectory createFromStatesStorage(const Model& model,
            const Storage& sto,
            bool checkMissingFromStorage = true,
            bool checkMissingFromModel = true);

    /** Convenience form of createFromStatesStorage that takes the path to a
     * Storage file instead of a Storage object.
     */
    static StatesTrajectory createFromStatesStorage(const Model& model,
            const std::string& filepath,
            bool checkMissingFromStorage = true,
            bool checkMissingFromModel = true);
    /// @}

private:
    std::vector<SimTK::State> m_states;
};

} // namespace

#endif // OPENSIM_STATES_TRAJECTORY_H_
