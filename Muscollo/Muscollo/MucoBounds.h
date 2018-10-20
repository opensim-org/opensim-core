#ifndef MUSCOLLO_MUCOBOUNDS_H
#define MUSCOLLO_MUCOBOUNDS_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoBounds.h                                             *
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

#include "osimMuscolloDLL.h"

#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Array.h>

namespace OpenSim {

class MucoPhase;
class MucoVariableInfo;
class MucoParameter;

/// Small struct to handle bounds.
class OSIMMUSCOLLO_API MucoBounds : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoBounds, Object);
public:
    /// The bounds are NaN, which means (-inf, inf).
    MucoBounds() = default;
    /// The lower and upper bound are equal (the variable is constrained to this
    /// single value).
    MucoBounds(double value) {
        set_lower(value);
        set_upper(value);
    }
    /// The variable is constrained to be within [lower, upper].
    MucoBounds(double lower, double upper) {
        OPENSIM_THROW_IF(lower > upper, Exception,
            "Expected lower <= upper, but lower=" + std::to_string(lower)
            + " and upper=" + std::to_string(upper) + ".");
        set_lower(lower);
        set_upper(upper);
    }
    /// True if the lower and upper bounds are both not NaN.
    bool isSet() const {
        return !SimTK::isNaN(get_lower()) && !SimTK::isNaN(get_upper());
    }
    /// True if the lower and upper bounds are the same, resulting in an
    /// equality constraint.
    bool isEquality() const {
        return isSet() && get_lower() == get_upper();
    }
    /// Returns true if the provided value is within these bounds.
    bool isWithinBounds(const double& value) const {
        return get_lower() <= value && value <= get_upper();
    }
    /// The returned array has either 0, 1, or 2 elements.
    /// - 0 elements: bounds are not set.
    /// - 1 element: equality constraint
    /// - 2 elements: range (inequality constraint).
    Array<double> getAsArray() const {
        Array<double> vec;
        if (isSet()) {
            vec.append(get_lower());
            if (get_lower() != get_upper()) vec.append(get_upper());
        }
        return vec;
    }
    double getLower() const { return get_lower(); }
    double getUpper() const { return get_upper(); }
    void printDescription(std::ostream& stream) const {
        if (isEquality()) {
            stream << get_lower();
        } else {
            stream << "[" << get_lower() << ", " << get_upper() << "]";
        }
        stream.flush();
    }
protected:
    /// Used internally to create Bounds from a list property.
    /// The list property must have either 0, 1 or 2 elements.
    MucoBounds(const Property<double>& p) {
        assert(p.size() <= 2);
        if (p.size() >= 1) {
            set_lower(p[0]);
            if (p.size() == 2) set_upper(p[1]);
            else               set_upper(p[0]);
        }
    }

    OpenSim_DECLARE_PROPERTY(lower, double, "The lower bound value.");
    OpenSim_DECLARE_PROPERTY(upper, double, "The upper bound value.");

    friend MucoPhase;
    friend MucoVariableInfo;
    friend MucoParameter;
private:

    void constructProperties() {
        constructProperty_lower(SimTK::NTraits<double>::getNaN());
        constructProperty_upper(SimTK::NTraits<double>::getNaN());
    }
};
/// Used for specifying the bounds on a variable at the start of a phase.
class OSIMMUSCOLLO_API MucoInitialBounds : public MucoBounds {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoInitialBounds, MucoBounds);

    using MucoBounds::MucoBounds;
    friend MucoPhase;
    friend MucoVariableInfo;
};
/// Used for specifying the bounds on a variable at the end of a phase.
class OSIMMUSCOLLO_API MucoFinalBounds : public MucoBounds {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoFinalBounds, MucoBounds);

    using MucoBounds::MucoBounds;
    friend MucoPhase;
    friend MucoVariableInfo;
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOBOUNDS_H