/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoVariableInfo.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia, Nicholas Bianco                             *
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

#include "MocoVariableInfo.h"
#include "MocoUtilities.h"

using namespace OpenSim;

MocoVariableInfo::MocoVariableInfo() {
    constructProperties();
}

MocoVariableInfo::MocoVariableInfo(const std::string& name,
        const MocoBounds& bounds, const MocoInitialBounds& initial,
        const MocoFinalBounds& final) : MocoVariableInfo() {
    setName(name);
    set_bounds(bounds.getAsArray());
    set_initial_bounds(initial.getAsArray());
    set_final_bounds(final.getAsArray());
    validate();
}

void MocoVariableInfo::validate() const {
    const auto& n = getName();
    const auto b = getBounds();
    const auto ib = getInitialBounds();
    const auto fb = getFinalBounds();

    OPENSIM_THROW_IF(ib.isSet() && ib.getLower() < b.getLower(), Exception,
            format("For variable %s, expected "
                   "[initial value lower bound] >= [lower bound], but "
                   "intial value lower bound=%g, lower bound=%g.",
                    n, ib.getLower(), b.getLower()));
    OPENSIM_THROW_IF(fb.isSet() && fb.getLower() < b.getLower(), Exception,
            format("For variable %s, expected "
                   "[final value lower bound] >= [lower bound], but "
                   "final value lower bound=%g, lower bound=%g.",
                    n, fb.getLower(), b.getLower()));
    OPENSIM_THROW_IF(ib.isSet() && ib.getUpper() > b.getUpper(), Exception,
            format("For variable %s, expected "
                   "[initial value upper bound] >= [upper bound], but "
                   "initial value upper bound=%g, upper bound=%g.",
                    n, ib.getUpper(), b.getUpper()));
    OPENSIM_THROW_IF(fb.isSet() && fb.getUpper() > b.getUpper(), Exception,
            format("For variable %s, expected "
                   "[final value upper bound] >= [upper bound], but "
                   "final value upper bound=%g, upper bound=%g.",
                    n, fb.getUpper(), b.getUpper()));
}

void MocoVariableInfo::printDescription(std::ostream& stream) const {
    const auto bounds = getBounds();
    stream << getName() << ". bounds: ";
    bounds.printDescription(stream);
    const auto initial = getInitialBounds();
    if (initial.isSet()) {
        stream << " initial: ";
        initial.printDescription(stream);
    }
    const auto final = getFinalBounds();
    if (final.isSet()) {
        stream << " final: ";
        final.printDescription(stream);
    }
    stream << std::endl;
}

void MocoVariableInfo::constructProperties() {
    constructProperty_bounds();
    constructProperty_initial_bounds();
    constructProperty_final_bounds();
}