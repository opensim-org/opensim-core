#ifndef MUSCOLLO_MUSCOLLOUTILITIES_H
#define MUSCOLLO_MUSCOLLOUTILITIES_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MuscolloUtilities.h                                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
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

#include <OpenSim/Common/Storage.h>

#include "osimMuscolloDLL.h"

namespace OpenSim {

class StatesTrajectory;
class Model;

/// Create a Storage from a TimeSeriesTable. Metadata from the
/// TimeSeriesTable is *not* copied to the Storage.
/// You should use TimeSeriesTable if possible, as support for Storage may be
/// reduced in future versions of OpenSim. However, Storage supports some
/// operations not supported by TimeSeriesTable (e.g., filtering, resampling).
// TODO move to the Storage class.
OSIMMUSCOLLO_API Storage convertTableToStorage(const TimeSeriesTable&);

/// Play back a motion (from the Storage) in the simbody-visuailzer. The Storage
/// should contain all generalized coordinates. The visualizer window allows the
/// user to control playback speed.
/// This function blocks until the user exits the simbody-visualizer window.
OSIMMUSCOLLO_API void visualize(Model, Storage);

/// Throw an exception if the property's value is not in the provided set.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyInSet(const Object& obj,
        const Property<T>& p, const std::set<T>& set) {
    const auto& value = p.getValue();
    if (set.find(value) == set.end()) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") has invalid value "
                << value << "; expected one of the following:";
        std::string separator("");
        for (const auto& s : set) {
            msg << " " << separator << s;
            if (separator.empty()) separator = ",";
        }
        msg << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

/// Throw an exception if the property's value is not positive.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyIsPositive(const Object& obj, const Property<T>& p) {
    const auto& value = p.getValue();
    if (value <= 0) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") must be positive, but is "
                << value << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

/// Throw an exception if the property's value is neither in the provided
/// range nor in the provided set.
/// We assume that `p` is a single-value property.
template <typename T>
void checkPropertyInRangeOrSet(const Object& obj, const Property<T>& p,
        const T& lower, const T& upper, const std::set<T>& set) {
    const auto& value = p.getValue();
    if ((value < lower || value > upper) && set.find(value) == set.end()) {
        std::stringstream msg;
        msg << "Property '" << p.getName() << "' (in ";
        if (!obj.getName().empty()) {
            msg << "object '" << obj.getName() << "' of type ";
        }
        msg << obj.getConcreteClassName() << ") has invalid value "
                << value << "; expected value to be in range "
                << lower << "," << upper << ", or one of the following:";
        std::string separator("");
        for (const auto& s : set) {
            msg << " " << separator << s;
            if (separator.empty()) separator = ",";
        }
        msg << ".";
        OPENSIM_THROW(Exception, msg.str());
    }
}

} // namespace OpenSim

#endif // MUSCOLLO_MUSCOLLOUTILITIES_H
