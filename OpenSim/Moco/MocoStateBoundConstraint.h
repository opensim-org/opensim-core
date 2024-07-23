#ifndef MOCOSTATEBOUNDCONSTRAINT_H
#define MOCOSTATEBOUNDCONSTRAINT_H
//
// Created by Allison John on 7/23/24.
//

#include "MocoConstraint.h"
#include "osimMocoDLL.h"

/** A bound constraint for states of the model
 * the state can be between two bounds or equal to the lower bound
 *
 * use Simulation Utilities createSystemYIndexMap
 * realize at level acceleration
 */
namespace OpenSim {

class OSIMMOCO_API MocoStateBoundConstraint : public MocoPathConstraint {
    OpenSim_DECLARE_CONCRETE_OBJECT(
            MocoStateBoundConstraint, MocoPathConstraint);

public:
    MocoStateBoundConstraint() { constructProperties(); }

    void addStatePath(std::string statePath) {
        append_state_paths(std::move(statePath));
    }
    void setStatePaths(const std::vector<std::string>& statePaths) {
        updProperty_state_paths().clear();
        for (const auto& path : statePaths) { append_state_paths(path); }
    }
    void clearControlPaths() { updProperty_state_paths().clear(); }
    std::vector<std::string> getControlPaths() const {
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

    /// Should the state be constrained to be equal to the lower bound (rather
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
    OpenSim_DECLARE_LIST_PROPERTY(state_paths, std::string,
            "Constrain the state variables specified by these paths.")
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
    mutable std::vector<int> m_stateIndices;
};

} // OpenSim

#endif //MOCOSTATEBOUNDCONSTRAINT_H
