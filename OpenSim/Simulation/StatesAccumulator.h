#ifndef OPENSIM_STATES_ACCUMULATOR_H_
#define OPENSIM_STATES_ACCUMULATOR_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StatesAccumulator.h                        *
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

#include "StatesTrajectory.h"
#include <OpenSim/Common/Component.h>
#include <OpenSim/Common/Reporter.h>

#include "osimSimulationDLL.h"

namespace OpenSim {

/** Stores the states during a simulation in a StatesTrajectory.
 *
 * This class was introduced in v4.0 and is intended to replace the
 * StatesReporter analysis.
 *
 * @ingroup reporters
 */
class OSIMSIMULATION_API StatesAccumulator : public AbstractReporter {
OpenSim_DECLARE_CONCRETE_OBJECT(StatesAccumulator, AbstractReporter);

public:
    /** Access the recorded states. */
    const StatesTrajectory& getStates() const; 
    /** Clear the recorded states. */ 
    void clear();

protected:
    // /** Clears the internal StatesTrajectory in preparation for a (new)
    //  * simulation */
    // TODO we have to discuss if the trajectory should be cleared.
    //  void extendRealizeInstance(const SimTK::State& state) const override;

    /** Appends the provided state to the trajectory. */
    void implementReport(const SimTK::State& state) const override;

private:
    StatesTrajectory m_states;
};

} // namespace

#endif // OPENSIM_STATES_ACCUMULATOR_H_
