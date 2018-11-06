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

#include <OpenSim/Common/Object.h>
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
    MucoBounds();
    /// The lower and upper bound are equal (the variable is constrained to this
    /// single value).
    MucoBounds(double value);
    /// The variable is constrained to be within [lower, upper].
    MucoBounds(double lower, double upper);
    /// True if the lower and upper bounds are both not NaN.
    bool isSet() const {
        return !getProperty_bounds().empty();
    }
    /// True if the lower and upper bounds are the same, resulting in an
    /// equality constraint.
    bool isEquality() const {
        return isSet() && getLower() == getUpper();
    }
    /// Returns true if the provided value is within these bounds.
    bool isWithinBounds(const double& value) const {
        return getLower() <= value && value <= getUpper();
    }
    double getLower() const {
        if (!isSet()) {
            return SimTK::NTraits<double>::getNaN();
        } else {
            return get_bounds(0);
        }
    }
    double getUpper() const {
        if (!isSet()) {
            return SimTK::NTraits<double>::getNaN();
        } else if (getProperty_bounds().size() == 1) {
            return get_bounds(0);
        } else {
            return get_bounds(1);
        }
    }
    /// The returned array has either 0, 1, or 2 elements.
    /// - 0 elements: bounds are not set.
    /// - 1 element: equality constraint
    /// - 2 elements: range (inequality constraint).
    Array<double> getAsArray() const {
        Array<double> vec;
        if (isSet()) {
            vec.append(getLower());
            if (getLower() != getUpper()) vec.append(getUpper());
        }
        return vec;
    }

    void printDescription(std::ostream& stream) const;

protected:
    /// Used internally to create Bounds from a list property.
    /// The list property must have either 0, 1 or 2 elements.
    MucoBounds(const Property<double>& p);

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value. "
        "2 values: lower, upper bounds on value.");

    friend MucoPhase;
    friend MucoVariableInfo;
    friend MucoParameter;

private:
    void constructProperties();

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