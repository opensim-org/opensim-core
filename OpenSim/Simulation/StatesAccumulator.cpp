/* -------------------------------------------------------------------------- *
 *                       OpenSim:  StatesAccumulator.cpp                      *
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

#include "StatesAccumulator.h"

using namespace OpenSim;


void StatesAccumulator::clear() {
    m_states.clear();
}

const StatesTrajectory& StatesAccumulator::getStates() const {
    return m_states;
}

/*
void StatesAccumulator::extendRealizeInstance(const SimTK::State& state) const {
    Super::extendRealizeInstance(state);
    const_cast<StatesAccumulator*>(this)->clear();
}
*/

void StatesAccumulator::implementReport(const SimTK::State& state) const {
    const_cast<StatesAccumulator*>(this)->m_states.append(state);
}
