/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoBounds.cpp                                           *
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

#include "MucoBounds.h"

using namespace OpenSim;

MucoBounds::MucoBounds() {
    constructProperties();
}

MucoBounds::MucoBounds(double value) : MucoBounds() {
    OPENSIM_THROW_IF(SimTK::isNaN(value), Exception, "NaN value detected. "
        "Please provide a non-NaN value for the bounds.");
    append_bounds(value);
}

MucoBounds::MucoBounds(double lower, double upper) : MucoBounds() {
    OPENSIM_THROW_IF(SimTK::isNaN(lower) || SimTK::isNaN(upper), Exception, 
        "NaN value detected. Please provide a non-NaN values for the bounds.");
    OPENSIM_THROW_IF(lower > upper, Exception,
        "Expected lower <= upper, but lower=" + std::to_string(lower)
        + " and upper=" + std::to_string(upper) + ".");
    append_bounds(lower);
    append_bounds(upper);
}

MucoBounds::MucoBounds(const Property<double>& p) : MucoBounds() {
    assert(p.size() <= 2);
    if (p.size() >= 1) {
        OPENSIM_THROW_IF(SimTK::isNaN(p[0]), Exception, "NaN value detected. "
            "Please provide a non-NaN value for the bounds.");
        append_bounds(p[0]);
        if (p.size() == 2) {
            OPENSIM_THROW_IF(SimTK::isNaN(p[1]), Exception, "NaN value "
                "detected. Please provide a non-NaN value for the bounds.");
            append_bounds(p[1]);
        }
    }
}

void MucoBounds::constructProperties() {
    constructProperty_bounds();
}

void MucoBounds::printDescription(std::ostream& stream) const {
    if (isEquality()) {
        stream << getLower();
    }
    else {
        stream << "[" << getLower() << ", " << getUpper() << "]";
    }
    stream.flush();
}