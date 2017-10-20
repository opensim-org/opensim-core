#ifndef MUSCOLLO_MUCOPROBLEM_H
#define MUSCOLLO_MUCOPROBLEM_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: MucoProblem.h                                           *
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

#include "MucoCost.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSim {

class MucoPhase;
class MucoVariableInfo;

struct OSIMMUSCOLLO_API MucoBounds {
    MucoBounds() = default;
    MucoBounds(double value) : lower(value), upper(value) {}
    MucoBounds(double lower, double upper) {
        OPENSIM_THROW_IF(lower > upper, Exception,
                "Expected lower <= upper, but lower=" + std::to_string(lower)
                + " and upper=" + std::to_string(upper) + ".");
        this->lower = lower;
        this->upper = upper;
    }
    bool isSet() const {
        return !SimTK::isNaN(lower) && !SimTK::isNaN(upper);
    }
    Array<double> getAsArray() const {
        Array<double> vec;
        if (isSet()) {
            vec.append(lower);
            if (lower != upper) vec.append(upper);
        }
        return vec;
    }
    double lower = SimTK::NTraits<double>::getNaN();
    double upper = SimTK::NTraits<double>::getNaN();
protected:
    /// Used internally to convert a list property to a
    /// It's expected that the list property has either 0, 1 or 2 elements.
    MucoBounds(const Property<double>& p) {
        if (p.size() >= 1) {
            lower = p[0];
            if (p.size() == 2) upper = p[1];
            else               upper = p[0];
        }
    }

    friend MucoPhase;
    friend MucoVariableInfo;
};
struct OSIMMUSCOLLO_API MucoInitialBounds : public MucoBounds {
    using MucoBounds::MucoBounds;
    friend MucoPhase;
    friend MucoVariableInfo;
};
struct OSIMMUSCOLLO_API MucoFinalBounds : public MucoBounds {
    using MucoBounds::MucoBounds;
    friend MucoPhase;
    friend MucoVariableInfo;
};

class MucoPhase;

/// Bounds on continuous variables (states, controls). The name should
/// correspond to path of a state variable or an actuator in the model.
class OSIMMUSCOLLO_API MucoVariableInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoVariableInfo, Object);
public:
    MucoVariableInfo();
    MucoVariableInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds&, const MucoFinalBounds&);

    MucoBounds getBounds() const
    {   return MucoBounds(getProperty_bounds()); }
    MucoInitialBounds getInitialBounds() const
    {   return MucoInitialBounds(getProperty_initial_bounds()); }
    MucoFinalBounds getFinalBounds() const
    {   return MucoFinalBounds(getProperty_final_bounds()); }

private:

    // TODO error if not defined.
    // TODO change to RANGE (1, 2).
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2,
            "1 value: required value over all time. "
            "2 values: lower, upper bounds on value over all time.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(initial_bounds, double, 2,
            "1 value: required initial value. "
            "2 values: lower, upper bounds on initial value.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(final_bounds, double, 2,
            "1 value: required final value. "
            "2 values: lower, upper bounds on final value.");

    void constructProperties();
};

class OSIMMUSCOLLO_API MucoPhase : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoPhase, Object);
public:
    MucoPhase();

    void setModel(const Model&);
    void setTimeBounds(const MucoInitialBounds&, const MucoFinalBounds&);
    void setStateInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    void setControlInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    void addCost(const MucoCost&);

    const Model& getModel() const { return get_model(); }
    MucoInitialBounds getTimeInitialBounds() const;
    MucoFinalBounds getTimeFinalBounds() const;
    const MucoVariableInfo& getStateInfo(const std::string& name) const;
    const MucoVariableInfo& getControlInfo(const std::string& name) const;
    SimTK::Real calcIntegralCost(const SimTK::State& state) const;
    SimTK::Real calcEndpointCost(const SimTK::State& finalState) const;
private:
    OpenSim_DECLARE_PROPERTY(model, Model,
            "OpenSim Model to provide dynamics.");
    // TODO error if not provided.
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_initial_bounds, double, 2,
            "1 value: required initial time. "
            "2 values: lower, upper bounds on initial time.");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_final_bounds, double, 2,
            "1 value: required final time. "
            "2 values: lower, upper bounds on final time.");
    OpenSim_DECLARE_LIST_PROPERTY(state_infos, MucoVariableInfo,
            "The state variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(control_infos, MucoVariableInfo,
            "The control variables' bounds.");
    OpenSim_DECLARE_LIST_PROPERTY(costs, MucoCost,
            "Quantities to minimize in the cost functional.");

    void constructProperties();
};


/// TODO explain phases.
class OSIMMUSCOLLO_API MucoProblem : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MucoProblem, Object);
public:
    MucoProblem();

    /// @name Convenience methods for phase 0.
    /// These methods allow you to conveniently edit phase 0 of the problem.
    /// See MucoPhase's documentation for more information.
    /// @{

    /// Set the model to use for phase 0.
    void setModel(const Model&);
    /// Set time bounds for phase 0.
    void setTimeBounds(const MucoInitialBounds&, const MucoFinalBounds&);
    /// Set bounds for a state variable for phase 0.
    void setStateInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    /// Set bounds for a control variable for phase 0.
    void setControlInfo(const std::string& name, const MucoBounds&,
            const MucoInitialBounds& = {}, const MucoFinalBounds& = {});
    /// Add a cost term for phase 0.
    void addCost(const MucoCost&);
    /// @}

    const MucoPhase& getPhase(int index) const
    {   return get_phases(index); }

    // TODO
    //void checkWellPosed();
private:
    // TODO OpenSim_DECLARE_LIST_PROPERTY_ATLEAST(phases, MucoPhase, 1,
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(phases, MucoPhase, 1,
            "List of 1 or more (TODO) MucoPhases.");

    void constructProperties();
};

} // namespace OpenSim

#endif // MUSCOLLO_MUCOPROBLEM_H
