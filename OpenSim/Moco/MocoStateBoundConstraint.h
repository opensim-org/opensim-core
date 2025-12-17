#ifndef OPENSIM_MOCOSTATEBOUNDCONSTRAINT_H
#define OPENSIM_MOCOSTATEBOUNDCONSTRAINT_H
/* -------------------------------------------------------------------------- *
 * OpenSim: MocoStateBoundConstraint.h                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2024 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Allison John                                                    *
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

/** This path constraint allows you to bound any number of state variables
between two time-based functions. It is possible to constrain the state variable
to match the value from a provided function; see the equality_with_lower
property.

If a function is a GCVSpline, we ensure that the spline covers the entire
possible time range in the problem (using the problem's time bounds). We do
not perform such a check for other types of functions.

@note If you omit the lower and upper bounds, then this class will not
constrain any state variable, even if you have provided state paths.

@ingroup mocopathcon */
namespace OpenSim {

class OSIMMOCO_API MocoStateBoundConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoStateBoundConstraint, MocoPathConstraint);

public:
    MocoStateBoundConstraint() { constructProperties(); }

    /** Add a state path (absolute path to state varaibles in the model)
    to be constrained by this class  (e.g., "/slider/position/speed"). */
    void addStatePath(std::string statePath) {
        append_state_paths(std::move(statePath));
    }
    void setStatePaths(const std::vector<std::string>& statePaths) {
        updProperty_state_paths().clear();
        for (const auto& path : statePaths) { append_state_paths(path); }
    }
    void clearStatePaths() { updProperty_state_paths().clear(); }
    std::vector<std::string> getStatePaths() const {
        std::vector<std::string> paths;
        for (int i = 0; i < getProperty_state_paths().size(); ++i) {
            paths.push_back(get_state_paths(i));
        }
        return paths;
    }

    void setLowerBound(const Function& f) { set_lower_bound(f); }
    void clearLowerBound() { updProperty_lower_bound().clear(); }
    bool hasLowerBound() const { return !getProperty_lower_bound().empty(); }
    const Function& getLowerBound() const { return get_lower_bound(); }

    void setUpperBound(const Function& f) { set_upper_bound(f); }
    void clearUpperBound() { updProperty_upper_bound().clear(); }
    bool hasUpperBound() const { return !getProperty_upper_bound().empty(); }
    const Function& getUpperBound() const { return get_upper_bound(); }

    /** Set whether the state should be constrained to be equal to the lower
    bound (rather than an inequality constraint). In this case, the upper bound
    must be unspecified. */
    void setEqualityWithLower(bool v) { set_equality_with_lower(v); }
    //// @copydoc setEqualityWithLower()
    bool getEqualityWithLower() const { return get_equality_with_lower(); }


protected:
    void initializeOnModelImpl(
            const Model& model, const MocoProblemInfo&) const override;

    void calcPathConstraintErrorsImpl(
            const SimTK::State& state, SimTK::Vector& errors) const override;

private:
    OpenSim_DECLARE_LIST_PROPERTY(state_paths, std::string,
            "Constrain the state variables specified by these paths.")
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
            lower_bound, Function, "Lower bound as a function of time.");
    OpenSim_DECLARE_OPTIONAL_PROPERTY(
            upper_bound, Function, "Upper bound as a function of time.");
    OpenSim_DECLARE_PROPERTY(equality_with_lower, bool,
            "The state must be equal to the lower bound; "
            "upper must be unspecified (default: false).");

    void constructProperties();

    mutable bool m_hasLower;
    mutable bool m_hasUpper;
    mutable std::vector<int> m_stateIndices;
};

} // namespace OpenSim

#endif // OPENSIM_MOCOSTATEBOUNDCONSTRAINT_H
