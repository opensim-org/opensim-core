#ifndef OPENSIM_MOCOBOUNDS_H
#define OPENSIM_MOCOBOUNDS_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoBounds.h                                                      *
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

#include "osimMocoDLL.h"

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Array.h>

namespace OpenSim {

class MocoPhase;
class MocoVariableInfo;
class MocoParameter;
class MocoScaleFactor;

/** Small struct-like class to handle bounds. */
class OSIMMOCO_API MocoBounds : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoBounds, Object);
public:
    /// The bounds are unset (NaN).
    MocoBounds();
    /// The lower and upper bound are equal (the variable is constrained to this
    /// single value).
    MocoBounds(double value);
    /// The variable is constrained to be within [lower, upper].
    MocoBounds(double lower, double upper);
    /// Create bounds that are (-inf, inf), so the variable is unconstrained.
    static MocoBounds unconstrained() {
        return MocoBounds(-SimTK::Infinity, SimTK::Infinity);
    }
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
    /** The returned array has either 0, 1, or 2 elements.
    - 0 elements: bounds are not set.
    - 1 element: equality constraint
    - 2 elements: range (inequality constraint). */
    Array<double> getAsArray() const {
        Array<double> vec;
        if (isSet()) {
            vec.append(getLower());
            if (getLower() != getUpper()) vec.append(getUpper());
        }
        return vec;
    }

protected:
    /** Used internally to create Bounds from a list property.
    The list property must have either 0, 1 or 2 elements. */
    MocoBounds(const Property<double>& p);

    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
        "1 value: required value. "
        "2 values: lower, upper bounds on value.");

    friend MocoPhase;
    friend MocoVariableInfo;
    friend MocoParameter;
    friend MocoScaleFactor;

private:
    void constructProperties();

};

inline std::ostream& operator<<(
        std::ostream& stream, const MocoBounds& bounds) {
    if (bounds.isEquality()) {
        stream << bounds.getLower();
    } else {
        stream << "[" << bounds.getLower() << ", " << bounds.getUpper() << "]";
    }
    return stream;
}

/// Used for specifying the bounds on a variable at the start of a phase.
class OSIMMOCO_API MocoInitialBounds : public MocoBounds {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoInitialBounds, MocoBounds);
    using MocoBounds::MocoBounds;
    friend MocoPhase;
    friend MocoVariableInfo;
};
/// Used for specifying the bounds on a variable at the end of a phase.
class OSIMMOCO_API MocoFinalBounds : public MocoBounds {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalBounds, MocoBounds);
    using MocoBounds::MocoBounds;
    friend MocoPhase;
    friend MocoVariableInfo;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOBOUNDS_H
