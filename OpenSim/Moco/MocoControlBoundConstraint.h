#ifndef OPENSIM_MOCOCONTROLBOUNDCONSTRAINT_H
#define OPENSIM_MOCOCONTROLBOUNDCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoControlBoundConstraint.h                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

namespace OpenSim {

class MocoProblemInfo;

/** This class constrains any number of control signals from ScalarActautor%s to
be between two time-based functions. It is possible to constrain the control
signal to be exactly to a provided function; see the equality_with_lower
property.

If a function is a GCVSpline, we ensure that the spline covers the entire
possible time range in the problem (using the problem's time bounds). We do
not perform such a check for other types of functions.

@note If you omit the lower and upper bounds, then this class will not
constrain any control signals, even if you have provided control paths.

@note This class can only constrain control signals for ScalarActuator%s.
@ingroup mocopathcon */
class OSIMMOCO_API MocoControlBoundConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoControlBoundConstraint, MocoPathConstraint);

public:
    MocoControlBoundConstraint();

    /// @name Control paths
    /// Set the control paths (absolute paths to actuators in the model)
    /// constrained by this class.
    /// @{
    void addControlPath(std::string controlPath) {
        append_control_paths(std::move(controlPath));
    }
    void setControlPaths(const std::vector<std::string>& controlPaths) {
        updProperty_control_paths().clear();
        for (const auto& path : controlPaths) { append_control_paths(path); }
    }
    void clearControlPaths() { updProperty_control_paths().clear(); }
    std::vector<std::string> getControlPaths() const {
        std::vector<std::string> paths;
        for (int i = 0; i < getProperty_control_paths().size(); ++i) {
            paths.push_back(get_control_paths(i));
        }
        return paths;
    }
    /// @}

    void setLowerBound(const Function& f) { set_lower_bound(f); }
    void clearLowerBound() { updProperty_lower_bound().clear(); }
    bool hasLowerBound() const { return !getProperty_lower_bound().empty(); }
    const Function& getLowerBound() const { return get_lower_bound(); }

    void setUpperBound(const Function& f) { set_upper_bound(f); }
    void clearUpperBound() { updProperty_upper_bound().clear(); }
    bool hasUpperBound() const { return !getProperty_upper_bound().empty(); }
    const Function& getUpperBound() const { return get_upper_bound(); }

    /// Should the control be constrained to be equal to the lower bound (rather
    /// than an inequality constraint)? In this case, the upper bound must be
    /// unspecified.
    void setEqualityWithLower(bool v) { set_equality_with_lower(v); }
    //// @copydoc setEqualityWithLower()
    bool getEqualityWithLower() const { return get_equality_with_lower(); }

protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;

    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(control_paths, std::string,
            "Constrain the control signal of the actuators specified by these "
            "paths.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
            lower_bound, Function, "Lower bound as a function of time.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
            upper_bound, Function, "Upper bound as a function of time.");
    OpenSim_DECLARE_PROPERTY(equality_with_lower, bool,
            "The control must be equal to the lower bound; "
            "upper must be unspecified (default: false).");

    void constructProperties();

    mutable bool m_hasLower;
    mutable bool m_hasUpper;
    mutable std::vector<int> m_controlIndices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOCONTROLBOUNDCONSTRAINT_H
