#ifndef OPENSIM_MOCOCONSTRAINTINFO_H
#define OPENSIM_MOCOCONSTRAINTINFO_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoConstraintInfo.h                                              *
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

#include "MocoBounds.h"
#include "MocoUtilities.h"
#include "osimMocoDLL.h"

#include <OpenSim/Common/Object.h>

namespace OpenSim {

/** Information for a given constraint in the optimal control problem. The name
should correspond to a MocoPathConstraint in the problem.
@ingroup mococonstraint */
class OSIMMOCO_API MocoConstraintInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoConstraintInfo, Object);

public:
    MocoConstraintInfo();

    int getNumEquations() const { return m_num_equations; }
    /// Get the bounds on the scalar constraint equations. If the number of
    /// equations have been set, but not the bounds, zero-bounds are returned
    /// as a default. If nothing has been set, this returns an empty vector.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<MocoBounds> getBounds() const {
        std::vector<MocoBounds> bounds;
        for (int i = 0; i < getNumEquations(); ++i) {
            if (getProperty_bounds().empty()) {
                bounds.push_back({0.0, 0.0});
            } else {
                bounds.push_back(get_bounds(i));
            }
        }
        return bounds;
    }
    /// Get the suffixes for the scalar constraint equations. If the suffixes
    /// have not been set, this returns an empty vector.
    /// @details Note: the return value is constructed fresh on every call from
    /// the internal property. Avoid repeated calls to this function.
    std::vector<std::string> getSuffixes() const {
        std::vector<std::string> suffixes;
        for (int i = 0; i < getProperty_suffixes().size(); ++i) {
            suffixes.push_back(get_suffixes(i));
        }
        return suffixes;
    }
    /// @details Note: if the number of equations has not been set, this updates
    /// the internal equation count variable. If the number of equations has
    /// been set and the vector passed is the incorrect size, an error is
    /// thrown.
    void setBounds(const std::vector<MocoBounds>& bounds) {
        updProperty_bounds().clear();
        for (int i = 0; i < (int)bounds.size(); ++i) {
            updProperty_bounds().appendValue(bounds[i]);
        }
        updateNumEquationsFromProperty(getProperty_bounds());
    }
    /// @copydoc setBounds()
    void setSuffixes(const std::vector<std::string>& suffixes) {
        updProperty_suffixes().clear();
        for (int i = 0; i < (int)suffixes.size(); ++i) {
            updProperty_suffixes().appendValue(suffixes[i]);
        }
        updateNumEquationsFromProperty(getProperty_suffixes());
    }
    /** Get a list of constraint labels based on the constraint name and, if
    specified, the list of suffixes. If no suffixes have been specified,
    zero-indexed, numeric suffixes will be applied as a default. The length
    of the returned vector is equal to the value returned by
    getNumEquations(). */
    std::vector<std::string> getConstraintLabels() const;

    /** Print the name, type, number of scalar equations, and bounds for this
    constraint. */
    void printDescription() const;

private:
    OpenSim_DECLARE_LIST_PROPERTY(bounds, MocoBounds,
            "(Optional) The bounds "
            "on the set of scalar constraint equations.");
    OpenSim_DECLARE_LIST_PROPERTY(suffixes, std::string,
            "(Optional) A list of "
            "strings to create unique labels for the scalar constraint "
            "equations. "
            "These are appended to the name of the MocoConstraint object when "
            "calling getConstraintLabels().");

    void constructProperties();

    int m_num_equations = 0;
    void setNumEquations(int numEqs) {
        m_num_equations = numEqs;
        checkPropertySize(getProperty_bounds());
        checkPropertySize(getProperty_suffixes());
    }
    friend class MocoPathConstraint;
    friend class MocoKinematicConstraint;
    friend class MocoGoal;

    void updateNumEquationsFromProperty(const AbstractProperty& prop) {
        if (!m_num_equations) {
            m_num_equations = prop.size();
        } else {
            checkPropertySize(prop);
        }
    }
    void checkPropertySize(const AbstractProperty& prop) {
        if (!prop.empty()) {
            OPENSIM_THROW_IF(m_num_equations != prop.size(), Exception,
                    "Size of property {} is not consistent with "
                    "current number of constraint equations.",
                    prop.getName());
        }
    }
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONSTRAINTINFO_H
