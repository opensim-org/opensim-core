#ifndef MUSCOLLO_MUCOCOST_H
#define MUSCOLLO_MUCOCOST_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoCost.h                                               *
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

#include <OpenSim/Common/Object.h>

#include <SimTKcommon/internal/State.h>

#include "osimMuscolloDLL.h"

namespace OpenSim {

class Model;

// TODO give option to specify gradient and hessian analytically.

/// A term in the cost functional, to be minimized.
/// @par For developers
/// Every time the problem is solved, a copy of this cost is used. An individual
/// instance of a cost is only ever used in a single problem. Therefore, there
/// is no need to clear cache variables that you create in initializeImpl().
/// Also, information stored in this cost does not persist across multiple
/// solves.
/// @ingroup mucocost
class OSIMMUSCOLLO_API MucoCost : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MucoCost, Object);
public:
    OpenSim_DECLARE_PROPERTY(weight, double,
            "The cost value is multiplied by this weight (default: 1).");

    MucoCost();

    MucoCost(std::string name);

    MucoCost(std::string name, double weight);

    /// This includes the weight.
    SimTK::Real calcIntegralCost(const SimTK::State& state) const {
        double integrand = 0;
        calcIntegralCostImpl(state, integrand);
        return get_weight() * integrand;
    }
    /// This includes the weight.
    // We use SimTK::Real instead of double for when we support adoubles.
    SimTK::Real calcEndpointCost(const SimTK::State& finalState) const {
        double cost = 0;
        calcEndpointCostImpl(finalState, cost);
        return get_weight() * cost;
    }
    /// For use by solvers. This also performs error checks on the Problem.
    void initializeOnModel(const Model& model) const {
        m_model.reset(&model);
        initializeOnModelImpl(model);
    }

    /// Print the name, type, and weight for this cost.
    void printDescription(std::ostream& stream = std::cout) const;

protected:
    /// Perform any caching before the problem is solved.
    /// Upon entry, getModel() is available.
    /// The passed-in model is equivalent to getModel().
    /// Use this opportunity to check for errors in user input.
    // TODO: Rename to extendInitializeOnModel().
    virtual void initializeOnModelImpl(const Model&) const {}
    /// @precondition The state is realized to SimTK::Stage::Position.
    /// If you need access to the controls, you must realize to Velocity:
    /// @code
    /// getModel().realizeVelocity(state);
    /// @endcode
    virtual void calcIntegralCostImpl(const SimTK::State& state,
            double& integrand) const;
    /// The endpoint cost cannot depend on actuator controls.
    virtual void calcEndpointCostImpl(const SimTK::State& finalState,
            SimTK::Real& cost) const;
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

inline void MucoCost::calcIntegralCostImpl(const SimTK::State&,
        double&) const {}

inline void MucoCost::calcEndpointCostImpl(const SimTK::State&,
        double&) const {}

/// Endpoint cost for final time.
/// @ingroup mucocost
class OSIMMUSCOLLO_API MucoFinalTimeCost : public MucoCost {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoFinalTimeCost, MucoCost);
public:
    MucoFinalTimeCost() = default;
    MucoFinalTimeCost(std::string name) : MucoCost(std::move(name)) {}
    MucoFinalTimeCost(std::string name, double weight)
            : MucoCost(std::move(name), weight) {}
protected:
    void calcEndpointCostImpl(const SimTK::State& finalState,
            SimTK::Real& cost) const override {
        cost = finalState.getTime();
    }
};


} // namespace OpenSim

#endif // MUSCOLLO_MUCOCOST_H
