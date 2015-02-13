/* -------------------------------------------------------------------------- *
 *                             OpenSim: Delay.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#include "Delay.h"

using namespace OpenSim;

Delay::Delay() {
    constructInfrastructure();
}

void Delay::constructProperties() {
    constructProperty_delay(0.0);
}

void Delay::constructInputs() {
    constructInput<double>("input", SimTK::Stage::Model);
}

void Delay::constructOutputs() {
    constructOutput<double>("output", &Delay::getValue, SimTK::Stage::Model);
}

double Delay::getValue(const SimTK::State& s) const {
    const auto& subsys = getSystem().getDefaultSubsystem();
    const auto& measure = subsys.getMeasure(_delayMeasureIndex);
    return SimTK::Measure::Delay::getAs(measure).getValue(s);
}

void Delay::extendFinalizeFromProperties() {
    SimTK_VALUECHECK_NONNEG_ALWAYS(get_delay(),
            "delay", "Delay::extendFinalizeFromProperties()");
}
void Delay::extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);

    auto& sub = system.updDefaultSubsystem();
    const auto& input = *static_cast<const Input<double>*>(&getInput("input"));
    SimTK::Measure::Delay delayMeasure(sub,
                                       InputMeasure<double>(sub, input),
                                       get_delay());
    const_cast<Delay*>(this)->_delayMeasureIndex =
        delayMeasure.getSubsystemMeasureIndex();
        
}
