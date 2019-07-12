#ifndef MOCO_MOCOCOST_H
#define MOCO_MOCOCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCost.h                                                   *
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

#include "../osimMocoDLL.h"

#include <SimTKcommon/internal/State.h>

#include <OpenSim/Common/Object.h>

namespace OpenSim {

class Model;

// TODO give option to specify gradient and Hessian analytically.

/// A term in the cost functional, to be minimized. Costs depend on the phase's
/// initial and final states and controls, and optionally on the integral of a
/// quantity over the phase.
/// @par For developers
/// Every time the problem is solved, a copy of this cost is used. An individual
/// instance of a cost is only ever used in a single problem. Therefore, there
/// is no need to clear cache variables that you create in initializeImpl().
/// Also, information stored in this cost does not persist across multiple
/// solves.
/// @ingroup mococost
class OSIMMOCO_API MocoCost : public Object {
    OpenSim_DECLARE_ABSTRACT_OBJECT(MocoCost, Object);

public:
    OpenSim_DECLARE_PROPERTY(weight, double,
            "The cost value is multiplied by this weight (default: 1).");

    OpenSim_DECLARE_PROPERTY(
            enabled, bool, "This bool indicates whether this cost is enabled.");

    MocoCost();

    MocoCost(std::string name);

    MocoCost(std::string name, double weight);

    /// Get the number of integrals required by this cost.
    /// This returns either 0 (for a strictly-endpoint cost) or 1.
    int getNumIntegrals() const {
        int num = getNumIntegralsImpl();
        OPENSIM_THROW_IF(num < 0, Exception,
                "Number of integrals must be non-negative.");
        return num;
    }
    /// Calculate the integrand that should be integrated and passed to
    /// calcCost(). If getNumIntegrals() is not zero, this must be implemented.
    SimTK::Real calcIntegrand(const SimTK::State& state) const {
        double integrand = 0;
        if (!get_enabled()) { return integrand; }
        calcIntegrandImpl(state, integrand);
        return integrand;
    }
    struct CostInput {
        const SimTK::State& initial_state;
        const SimTK::State& final_state;
        /// This is computed by integrating calcIntegrand().
        const double& integral;
    };
    /// The returned cost includes the weight.
    // We use SimTK::Real instead of double for when we support adoubles.
    SimTK::Real calcCost(const CostInput& input) const {
        double cost = 0;
        if (!get_enabled()) { return cost; }
        calcCostImpl(input, cost);
        return get_weight() * cost;
    }
    /// For use by solvers. This also performs error checks on the Problem.
    void initializeOnModel(const Model& model) const {
        m_model.reset(&model);
        if (!get_enabled()) { return; }
        initializeOnModelImpl(model);
    }

    /// Print the name, type, and weight for this cost.
    void printDescription(std::ostream& stream = std::cout) const;

protected:
    /// Perform any caching before the problem is solved.
    /// @precondition The model is initialized (initSystem()) and getModel()
    /// is available.
    /// The passed-in model is equivalent to getModel().
    /// Use this opportunity to check for errors in user input.
    // TODO: Rename to extendInitializeOnModel().
    virtual void initializeOnModelImpl(const Model&) const {}
    /// Return the number if integral terms required by this cost.
    /// This must be either 0 or 1.
    virtual int getNumIntegralsImpl() const = 0;
    /// @precondition The state is realized to SimTK::Stage::Position.
    /// If you need access to the controls, you must realize to Velocity:
    /// @code
    /// getModel().realizeVelocity(state);
    /// @endcode
    /// The Lagrange multipliers for kinematic constraints are not available.
    virtual void calcIntegrandImpl(
            const SimTK::State& state, double& integrand) const;
    /// The Lagrange multipliers for kinematic constraints are not available.
    virtual void calcCostImpl(
            const CostInput& input, SimTK::Real& cost) const = 0;
    /// For use within virtual function implementations.
    const Model& getModel() const {
        OPENSIM_THROW_IF(!m_model, Exception,
                "Model is not available until the start of initializing.");
        return m_model.getRef();
    }

private:
    void constructProperties();

    mutable SimTK::ReferencePtr<const Model> m_model;
};

inline void MocoCost::calcIntegrandImpl(
        const SimTK::State& state, double& integrand) const {}

/// Endpoint cost for final time.
/// @ingroup mococost
class OSIMMOCO_API MocoFinalTimeCost : public MocoCost {
    OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalTimeCost, MocoCost);

public:
    MocoFinalTimeCost() = default;
    MocoFinalTimeCost(std::string name) : MocoCost(std::move(name)) {}
    MocoFinalTimeCost(std::string name, double weight)
            : MocoCost(std::move(name), weight) {}

protected:
    int getNumIntegralsImpl() const override { return 0; }
    void calcCostImpl(
            const CostInput& input, SimTK::Real& cost) const override {
        cost = input.final_state.getTime();
    }
};

} // namespace OpenSim

#endif // MOCO_MOCOCOST_H
