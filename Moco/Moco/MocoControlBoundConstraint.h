#ifndef MOCO_MOCOCONTROLBOUNDCONSTRAINT_H
#define MOCO_MOCOCONTROLBOUNDCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoControlBoundConstraint.h                                 *
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

/// TODO
/// @note This class can only constrain control signals for ScalarActuator%s.
class OSIMMOCO_API MocoControlBoundConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoControlBoundConstraint, MocoPathConstraint);

public:
    MocoControlBoundConstraint();

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

    void setLowerBound(const Function& f) { set_lower_bound(f); }
    void clearLowerBound() { updProperty_lower_bound().clear(); }
    // TODO hasLowerBound(), getLowerBound().
    void setUpperBound(const Function& f) { set_upper_bound(f); }
    void clearUpperBound() { updProperty_upper_bound().clear(); }

    /// Should the control be constrained to be equal to the lower bound (rather
    /// than an inequality constraint)? In this case, the upper bound must be
    /// unspecified.
    void setEqualityWithLower(bool v) { set_equality_with_lower(v); }
    //// @copydoc setEqualityWithLower()
    bool getEqualityWithLower() const { return get_equality_with_lower(); }


protected:
    void initializeOnModelImpl(const Model& model) const override;

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
            "upper must be unspecified (default: false).")

            void constructProperties();

    mutable bool m_hasLower;
    mutable bool m_hasUpper;
    mutable std::vector<int> m_controlIndices;
};

} // namespace OpenSim

#endif // MOCO_MOCOCONTROLBOUNDCONSTRAINT_H
