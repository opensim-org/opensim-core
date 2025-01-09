#ifndef OPENSIM_SIGNAL_GENERATOR_H_
#define OPENSIM_SIGNAL_GENERATOR_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim: SignalGenerator.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Chris Dembia, Shrinidhi K. Lakshmikanth, Ajay Seth,             *
 *            Thomas Uchida                                                   *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "Function.h"
#include "Component.h"

namespace OpenSim { 

/** SignalGenerator is a type of Component with no inputs and only one output.
This Component evaluates an OpenSim::Function (stored in its "function"
property) as a function of time.

Here is an example of creating a SignalGenerator whose output is a sinusoid:
@code{.cpp}
auto* signalGen = new SignalGenerator();
signalGen->set_function(Sine(1, 1, 0));
@endcode
*/
class OSIMCOMMON_API SignalGenerator : public Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(SignalGenerator, Component);

public:
    OpenSim_DECLARE_PROPERTY(function, Function,
        "Function used to generate the signal (a function of time)");
    OpenSim_DECLARE_OUTPUT(signal, double, getSignal, SimTK::Stage::Time);

    SignalGenerator();

    double getSignal(const SimTK::State& s) const;

private:
    void constructProperties();

};

} // namespace OpenSim

#endif // OPENSIM_SIGNAL_GENERATOR_H_
