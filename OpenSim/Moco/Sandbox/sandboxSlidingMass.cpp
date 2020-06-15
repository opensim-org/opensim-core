/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxSlidingMass.cpp                                       *
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
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Actuators/osimActuators.h>
#include <tropter/tropter.h>

using tropter::VectorX;

namespace OpenSim {

class MocoVariableInfo : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoVariableInfo, Object);
public:
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(bounds, double, 2, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(initial_bounds, double, 2, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(final_bounds, double, 2, "TODO");
    MocoVariableInfo() {
        constructProperties();
    }
    MocoVariableInfo(const std::string& name,
            const tropter::Bounds& bounds,
            const tropter::InitialBounds& initialBounds = {},
            const tropter::FinalBounds& finalBounds = {})
            : MocoVariableInfo() {
        setName(name);
        if (bounds.is_set()) {
            append_bounds(bounds.lower);
            if (bounds.lower != bounds.upper) {
                append_bounds(bounds.upper);
            }
        }
        if (initialBounds.is_set()) {
            append_initial_bounds(initialBounds.lower);
            if (initialBounds.lower != initialBounds.upper) {
                append_initial_bounds(initialBounds.upper);
            }
        }
        if (finalBounds.is_set()) {
            append_final_bounds(finalBounds.lower);
            if (finalBounds.lower != finalBounds.upper) {
                append_final_bounds(finalBounds.upper);
            }
        }
    }
    tropter::Bounds getBounds() const {
        tropter::Bounds bounds;
        const auto& pBounds = getProperty_bounds();
        if (pBounds.size() >= 1) {
            bounds.lower = pBounds[0];
            if (pBounds.size() == 2) bounds.upper = pBounds[1];
            else                     bounds.upper = pBounds[0];
        }
        return bounds;
    }
    tropter::InitialBounds getInitialBounds() const {
        tropter::InitialBounds initial;
        const auto& pInitial = getProperty_initial_bounds();
        if (pInitial.size() >= 1) {
            initial.lower = pInitial[0];
            if (pInitial.size() == 2) initial.upper = pInitial[1];
            else                      initial.upper = pInitial[0];
        }
        return initial;
    }
    tropter::FinalBounds getFinalBounds() const {
        tropter::FinalBounds final;
        const auto& pFinal = getProperty_final_bounds();
        if (pFinal.size() >= 1) {
            final.lower = pFinal[0];
            if (pFinal.size() == 2) final.upper = pFinal[1];
            else                    final.upper = pFinal[0];
        }
        return final;
    }
private:
    void constructProperties() {
        constructProperty_bounds();
        constructProperty_initial_bounds();
        constructProperty_final_bounds();
    }
};

class MocoGoal : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(MocoGoal, Object);
public:
    OpenSim_DECLARE_PROPERTY(weight, double, "TODO");
    MocoGoal() {
        constructProperties();
    }
    // TODO allow alternate interface that does not require creating a SimTK
    // state (if just minimizing the control signal).
    // TODO create separate integral and endpoint cost types?
    double calcIntegrandCost(const SimTK::State& state) const {
        double integrand = 0;
        calcIntegralCostImpl(state, integrand);
        return get_weight() * integrand;
    }
    /// This includes the weight.
    double calcEndpointCost(const SimTK::State& finalState) const {
        double cost = 0;
        calcEndpointCostImpl(finalState, cost);
        return get_weight() * cost;
    }
    // TODO avoid this weirdness.
    void setModel(const Model& model) const { m_model.reset(&model); }
protected:
    virtual void calcIntegralCostImpl(const SimTK::State& state,
                                      double& integrand) const {}
    virtual void calcEndpointCostImpl(const SimTK::State& finalState,
                                      double& cost) const {}
    const Model& getModel() const { return m_model.getRef(); }
private:
    void constructProperties() {
        constructProperty_weight(1);
    }
    mutable SimTK::ReferencePtr<const Model> m_model;
};

class MocoSimpleTrackingCost : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoSimpleTrackingCost, MocoGoal);
public:
    MocoSimpleTrackingCost() {
        constructProperties();
    }
protected:
    void calcIntegralCostImpl(const SimTK::State& state,
                              double& integrand) const override {
        //getModel().realizePosition(state);
        //const auto& frame = getModel().getComponent<Frame>(get_frame_name());
        //auto actualLocation =
        //        frame.findStationLocationInGround(state,
        //                get_point_on_frame());
        const auto& time = state.getTime();
        //SimTK::Vec3 desiredLocation(
        //        (time / 5.0), 1, 0);
        //integrand = (actualLocation - desiredLocation).normSqr();
        SimTK::Vector Qdesired(2);
        Qdesired[0] = (time / 1.0) * 0.5 * SimTK::Pi;
        Qdesired[1] = (time / 1.0) * 0.5 * SimTK::Pi;
        integrand = (state.getQ() - Qdesired).normSqr();
    }
private:
    void constructProperties() {
    }
};

class MocoFinalTimeGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoFinalTimeGoal, MocoGoal);
protected:
    void calcEndpointCostImpl(const SimTK::State& finalState,
                              double& cost) const override {
        cost = finalState.getTime();
    }
};

// TODO should these be added to the model? we might need access to
// components, etc. Maybe these should have a method where they get to add
// arbitrary components to the model.
// TODO should also have an initialization routine to cache quantities.
class MocoMarkerFinalGoal : public MocoGoal {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoMarkerFinalGoal, MocoGoal);
public:
    OpenSim_DECLARE_PROPERTY(frame_name, std::string, "TODO");
    OpenSim_DECLARE_PROPERTY(point_on_frame, SimTK::Vec3, "TODO");
    OpenSim_DECLARE_PROPERTY(point_to_track, SimTK::Vec3,
            "TODO Expressed in ground.");
    MocoMarkerFinalGoal() {
        constructProperties();
    }
protected:
    void calcEndpointCostImpl(const SimTK::State& finalState,
                              double& cost) const override {
        getModel().realizePosition(finalState);
        const auto& frame = getModel().getComponent<Frame>(get_frame_name());
        auto actualLocation =
                frame.findStationLocationInGround(finalState,
                        get_point_on_frame());
        cost = (actualLocation - get_point_to_track()).normSqr();
    }
private:
    void constructProperties() {
        constructProperty_frame_name("");
        constructProperty_point_on_frame(SimTK::Vec3(0));
        constructProperty_point_to_track(SimTK::Vec3(0));
    }
};

class MocoProblem : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoProblem, Object);
public:

    OpenSim_DECLARE_PROPERTY(model_file, std::string, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_initial_bounds, double, 2,
            "TODO");
    OpenSim_DECLARE_LIST_PROPERTY_ATMOST(time_final_bounds, double, 2, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(state_info,
            MocoVariableInfo, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(control_info,
            MocoVariableInfo, "TODO");
    OpenSim_DECLARE_LIST_PROPERTY(costs, MocoGoal, "TODO");

    MocoProblem() {
        constructProperties();
    }

    /// Further changes to the model have no effect.
    void setModel(Model model) { m_model = std::move(model); }
    const Model& getModel() const { return m_model; }

    void setTimeBounds(
            const tropter::InitialBounds& initial,
            const tropter::FinalBounds& final) {
        if (initial.is_set()) {
            append_time_initial_bounds(initial.lower);
            if (initial.lower != initial.upper) {
                append_time_initial_bounds(initial.upper);
            }
        }
        if (final.is_set()) {
            append_time_final_bounds(final.lower);
            if (final.lower != final.upper) {
                append_time_final_bounds(final.upper);
            }
        }
    }

    tropter::InitialBounds getTimeInitialBounds() const {
        tropter::InitialBounds initial;
        const auto& pInitial = getProperty_time_initial_bounds();
        if (pInitial.size() >= 1) {
            initial.lower = pInitial[0];
            if (pInitial.size() == 2) initial.upper = pInitial[1];
            else                      initial.upper = pInitial[0];
        }
        return initial;
    }
    tropter::FinalBounds getTimeFinalBounds() const {
        tropter::FinalBounds final;
        const auto& pFinal = getProperty_time_final_bounds();
        if (pFinal.size() >= 1) {
            final.lower = pFinal[0];
            if (pFinal.size() == 2) final.upper = pFinal[1];
            else                    final.upper = pFinal[0];
        }
        return final;
    }

    const MocoVariableInfo& getStateInfo(
            const std::string& name) const {
        int idx = getProperty_state_info().findIndexForName(name);
        OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
                "No info provided for state '" + name + "'.");
        return get_state_info(idx);
    }
    const MocoVariableInfo& getControlInfo(
            const std::string& name) const {
        int idx = getProperty_control_info().findIndexForName(name);
        OPENSIM_THROW_IF_FRMOBJ(idx == -1, Exception,
                "No info provided for control for '" + name + "'.");
        return get_control_info(idx);
    }
private:
    void constructProperties() {
        constructProperty_model_file("");

        constructProperty_time_initial_bounds();
        constructProperty_time_final_bounds();
        constructProperty_state_info();
        constructProperty_control_info();
        constructProperty_costs();
    }
    Model m_model;

};

/*
class MocoSolution {
public:
    MocoSolution(const tropter::Solution& tropSol) {
        const auto& tropTime = tropSol.time;
        std::vector<double> time(tropTime.data(),
                tropTime.data() + tropTime.size());
        // Concatenate the state and control names in a single vector.
        std::vector<std::string> labels = tropSol.state_names;
        labels.insert(labels.begin(),
                tropSol.control_names.begin(), tropSol.control_names.end());

        int numStates = tropSol.state_names.size();
        int numControls = tropSol.control_names.size();
        SimTK::Matrix data(tropTime.size(), labels.size());
        for (int itime = 0; itime < tropTime.size(); ++itime) {
            int icol;
            for (icol = 0; icol < numStates; ++icol) {
                data(itime, icol) = tropSol.states(itime, icol);
            }
            for (int icontr = 0; icontr < numControls; ++icontr, ++icol) {
                data(itime, icol) = tropSol.controls(itime, icontr);
            }
        }
        continuous = TimeSeriesTable(time, data, labels);
    }
private:
    //SimTK::Vector parameters;
    TimeSeriesTable continuous;
    //struct Iterate {
    //    Eigen::RowVectorXd time;
    //    Eigen::MatrixXd states;
    //    Eigen::MatrixXd controls;
    //    std::vector<std::string> state_names;
    //    std::vector<std::string> control_names;
    //    /// This constructor leaves all members empty.
    //    Iterate() = default;
    //    /// Read in states and controls from a CSV file generated by calling
    //    /// write().
    //    explicit Iterate(const std::string& filepath);
    //    /// Write the states and controls trajectories to a plain-text CSV file.
    //    virtual void write(const std::string& filepath) const;
    //};
};*/
class MocoTrajectory {
public:
    /// Allow implicit conversion from tropter's iterate type to Moco's
    /// iterate type.
    MocoTrajectory(const tropter::Iterate& tropIter) {
        const auto& tropTime = tropIter.time;
        m_time = SimTK::Vector((int)tropTime.size(), tropTime.data());
        m_state_names = tropIter.state_names;
        m_control_names = tropIter.control_names;

        int numTimes = (int)m_time.size();
        int numStates = (int)tropIter.state_names.size();
        int numControls = (int)tropIter.control_names.size();
        m_states = SimTK::Matrix(numTimes, numStates);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int istate = 0; istate < numStates; ++istate) {
                m_states(itime, istate) = tropIter.states(istate, itime);
            }
        }
        m_controls = SimTK::Matrix(numTimes, numControls);
        for (int itime = 0; itime < numTimes; ++itime) {
            for (int icontrol = 0; icontrol < numControls; ++icontrol) {
                m_controls(itime, icontrol) = tropIter.controls(icontrol, itime);
            }
        }
    }
    MocoTrajectory* clone() const { return new MocoTrajectory(*this); }
    /// Resize the time vector and the time dimension of the states and
    /// controls trajectories.
    /// This may erase any data that was previously stored.
    // TODO change this to interpolate.
    void setNumTimes(int numTimes) {
        m_time.resize(numTimes);
        m_states.resize(numTimes, m_states.ncol());
        m_controls.resize(numTimes, m_controls.ncol());
    }
    void setTime(SimTK::Vector time) {
        // TODO I don't think SimTK::Vector has a mov
        OPENSIM_THROW_IF(time.size() != m_time.size(), Exception,
                "Expected " + std::to_string(m_time.size()) +
                " times but got " + std::to_string(time.size()) + ".");
        m_time = std::move(time);
    }
    void setState(const std::string& name, const SimTK::Vector& trajectory) {
        OPENSIM_THROW_IF(trajectory.size() != m_states.nrow(), Exception,
                "For state " + name + ", expected " +
                std::to_string(m_states.nrow()) +
                " elements but got " + std::to_string(trajectory.size()) + ".");

        auto it = std::find(m_state_names.cbegin(), m_state_names.cend(), name);
        OPENSIM_THROW_IF(it == m_state_names.cend(), Exception,
                "Cannot find state named " + name + ".");
        int index = (int)std::distance(m_state_names.cbegin(), it);
        m_states.updCol(index) = trajectory;
    }
    void setControl(const std::string& name, const SimTK::Vector& trajectory) {
        OPENSIM_THROW_IF(trajectory.size() != m_controls.nrow(), Exception,
                "For control " + name + ", expected " +
                std::to_string(m_controls.nrow()) +
                " elements but got " + std::to_string(trajectory.size()) + ".");

        auto it = std::find(m_control_names.cbegin(), m_control_names.cend(),
                            name);
        OPENSIM_THROW_IF(it == m_control_names.cend(), Exception,
                "Cannot find control named " + name + ".");
        int index = (int)std::distance(m_control_names.cbegin(), it);
        m_controls.updCol(index) = trajectory;
    }
    /// This variant supports use of an initializer list. Example:
    /// @code{.cpp}
    /// iterate.setTime({0, 0.5, 1.0});
    /// @endcode
    void setTime_std(const std::vector<double>& time) {
        setTime(SimTK::Vector((int)time.size(), time.data()));
    }
    /// This variant supports use of an initializer list.
    void setState_std(const std::string& name,
                  const std::vector<double>& trajectory) {
        setState(name,
                SimTK::Vector((int)trajectory.size(), trajectory.data()));
    }
    /// This variant supports use of an initializer list.
    void setControl_std(const std::string& name,
                  const std::vector<double>& trajectory) {
        setControl(name,
                SimTK::Vector((int)trajectory.size(), trajectory.data()));
    }
    // TODO this can't be here if we want to avoid putting tropter in our
    // interface.
    explicit operator tropter::Iterate() const {
        tropter::Iterate tropIter;
        using Eigen::Map;
        using Eigen::RowVectorXd;
        using Eigen::MatrixXd;
        tropIter.time = Map<const RowVectorXd>(&m_time[0], m_time.size());


        tropIter.state_names = m_state_names;
        tropIter.control_names = m_control_names;

        int numTimes = (int)m_time.size();
        int numStates = (int)m_state_names.size();
        int numControls = (int)m_control_names.size();
        // Moco's matrix is numTimes x numStates;
        // tropter's is numStates x numTimes.
        tropIter.states = Map<const MatrixXd>(
                &m_states(0, 0), numTimes, numStates).transpose();
        tropIter.controls = Map<const MatrixXd>(
                &m_controls(0, 0), numTimes, numControls).transpose();
        return tropIter;
    }
    void write(const std::string& filepath) const {
        std::vector<double> time(&m_time[0], &m_time[0] + m_time.size());

        // Concatenate the state and control names in a single vector.
        std::vector<std::string> labels = m_state_names;
        labels.insert(labels.end(),
                m_control_names.begin(), m_control_names.end());

        int numTimes = (int)m_time.size();
        int numStates = (int)m_state_names.size();
        int numControls = (int)m_control_names.size();
        SimTK::Matrix data(numTimes, (int)labels.size());
        for (int itime = 0; itime < numTimes; ++itime) {
            int icol;
            for (icol = 0; icol < numStates; ++icol) {
                data(itime, icol) = m_states(itime, icol);
            }
            for (int icontr = 0; icontr < numControls; ++icontr, ++icol) {
                data(itime, icol) = m_controls(itime, icontr);
            }
        }
        TimeSeriesTable table0(time, data, labels);
        OpenSim::DataAdapter::InputTables tables = {{"table", &table0}};
        STOFileAdapter().writeFile(tables, filepath);
    }
private:
    SimTK::Vector m_time;
    std::vector<std::string> m_state_names;
    std::vector<std::string> m_control_names;
    SimTK::Matrix m_states;
    SimTK::Matrix m_controls;
};

class MocoSolver : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(MocoSolver, Object);
public:
    OpenSim_DECLARE_PROPERTY(num_mesh_intervals, double, "TODO");
    // TODO should be a copy or reference to MocoProblem?
    MocoSolver(const MocoProblem& problem) {
        constructProperties();
        m_problem.reset(&problem);
    }

    /// Create a template for an initial guess for the related MocoProblem.
    /// The number of times in the guess is based on num_mesh_intervals, and
    /// the guess is based on the bounds on the MocoProblem.
    MocoTrajectory createGuessTemplate() const {
        // TODO throw error if MocoProblem is not set yet.
        auto ocp = getOptimalControlProblem();
        // Create a direct collocation solver to access a function that
        // allows us to get an initial guess (we are NOT going to actually
        // try solving a problem here).
        int N = get_num_mesh_intervals();
        OPENSIM_THROW_IF_FRMOBJ(N < 0, Exception,
                "Invalid number of mesh intervals (" + std::to_string(N) + ")");
        tropter::DirectCollocationSolver<double> dircol(
                ocp, "trapezoidal", "ipopt", N);
        return dircol.make_initial_guess_from_bounds();
    }
    // TODO make sure time is nondecreasing and states/controls are within
    // bounds.
    void setGuess(const MocoTrajectory& guess) {
        m_guess.reset(new MocoTrajectory(guess));
    }

    MocoTrajectory solve() const;

    const MocoProblem& getProblem() const { return m_problem.getRef(); }

    template<typename T>
    class OCProblem;
protected:
    std::shared_ptr<const tropter::Problem<double>>
    getOptimalControlProblem() const;
private:
    void constructProperties() {
        constructProperty_num_mesh_intervals(100);
    }
    SimTK::ReferencePtr<const MocoProblem> m_problem;
    SimTK::ClonePtr<MocoTrajectory> m_guess;
    mutable SimTK::ResetOnCopy<std::shared_ptr<OCProblem<double>>>
            m_ocproblem;
};

/// The map provides the index of each state variable in
/// SimTK::State::getY() from its each state variable path string.
std::vector<std::string> createStateVariableNamesInSystemOrder(
        const Model& model) {
    std::vector<std::string> svNamesInSysOrder;
    auto s = model.getWorkingState();
    const auto svNames = model.getStateVariableNames();
    s.updY() = 0;
    for (int iy = 0; iy < s.getNY(); ++iy) {
        s.updY()[iy] = SimTK::NaN;
        const auto svValues = model.getStateVariableValues(s);
        for (int isv = 0; isv < svNames.size(); ++isv) {
            if (SimTK::isNaN(svValues[isv])) {
                svNamesInSysOrder.push_back(svNames[isv]);
                s.updY()[iy] = 0;
                break;
            }
        }
    }
    SimTK_ASSERT2_ALWAYS((size_t)svNames.size() == svNamesInSysOrder.size(),
            "Expected to get %i state names but found %i.", svNames.size(),
            svNamesInSysOrder.size());
    return svNamesInSysOrder;
}

// TODO should these be copyable to support parallelization with finite
// differences?
template<typename T>
class MocoSolver::OCProblem : public tropter::Problem<T> {
public:
    OCProblem(const MocoSolver& solver)
            : m_mocoSolver(solver),
              m_mocoProb(solver.getProblem()) {
        m_model = m_mocoProb.getModel();
        m_state = m_model.initSystem();

        this->set_time(m_mocoProb.getTimeInitialBounds(),
                       m_mocoProb.getTimeFinalBounds());
        auto svNamesInSysOrder = createStateVariableNamesInSystemOrder(m_model);
        for (const auto& svName : svNamesInSysOrder) {
            const auto& info = m_mocoProb.getStateInfo(svName);
            this->add_state(svName, info.getBounds(),
                    info.getInitialBounds(), info.getFinalBounds());
        }
        for (const auto& actu : m_model.getComponentList<Actuator>()) {
            // TODO handle a variable number of control signals.
            const auto& actuName = actu.getName();
            const auto& info = m_mocoProb.getControlInfo(actuName);
            this->add_control(actuName, info.getBounds(),
                    info.getInitialBounds(), info.getFinalBounds());
        }
        const auto& costs = m_mocoProb.getProperty_costs();
        // TODO avoid this weirdness; add the costs directly to the model.
        for (int ic = 0; ic < costs.size(); ++ic) {
            costs[ic].setModel(m_model);
        }
    }
    // TODO rename argument "states" to "state".
    void calc_differential_algebraic_equations(
            const tropter::Input<T>& in,
            tropter::Output<T> out) const override {

        // TODO convert to implicit formulation.

        const auto& states = in.states;
        const auto& controls = in.controls;

        m_state.setTime(in.time);

        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        //
        // TODO do not copy? I think this will still make a copy:
        //m_state.setY(SimTK::Vector(states.size(), states.data(), true));

        auto& osimControls = m_model.updControls(m_state);
        std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);

        m_model.realizeVelocity(m_state);
        m_model.setControls(m_state, osimControls);

        // TODO Antoine and Gil said realizing Dynamics is a lot costlier than
        // realizing to Velocity and computing forces manually.
        m_model.realizeAcceleration(m_state);
        std::copy(&m_state.getYDot()[0], &m_state.getYDot()[0] + states.size(),
                out.dynamics.data());

    }
    void calc_integral_cost(const tropter::Input<T>& in, 
            T& integrand) const override {

        // Unpack variables.
        const auto& states = in.states;
        const auto& controls = in.controls;

        integrand = 0;
        m_state.setTime(time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        auto& osimControls = m_model.updControls(m_state);
        std::copy(controls.data(), controls.data() + controls.size(),
                &osimControls[0]);
        m_model.realizePosition(m_state);
        m_model.setControls(m_state, osimControls);
        for (int i = 0; i < m_mocoProb.getProperty_costs().size(); ++i) {
            integrand += m_mocoProb.get_costs(i).calcIntegralCost(m_state);
        }
    }
    void calc_endpoint_cost(const tropter::Input<T>& in, 
            T& cost) const override {
        cost = 0;
        const auto& final_time = in.time;
        const auto& states = in.states;

        m_state.setTime(final_time);
        std::copy(states.data(), states.data() + states.size(),
                &m_state.updY()[0]);
        // TODO cannot use control signals...
        for (int i = 0; i < m_mocoProb.getProperty_costs().size(); ++i) {
            cost += m_mocoProb.get_costs(i).calcEndpointCost(m_state);
        }
    }

private:
    const MocoSolver& m_mocoSolver;
    const MocoProblem& m_mocoProb;
    Model m_model;
    mutable SimTK::State m_state;
};

std::shared_ptr<const tropter::Problem<double>>
MocoSolver::getOptimalControlProblem() const {
    if (!m_ocproblem) {
        m_ocproblem.reset(new OCProblem<double>(*this));
    }
    return m_ocproblem;
}

MocoTrajectory MocoSolver::solve() const {
    // TODO
    auto ocp = getOptimalControlProblem();
    ocp->print_description();
    int N = get_num_mesh_intervals();
    OPENSIM_THROW_IF_FRMOBJ(N < 0, Exception,
            "Invalid number of mesh intervals (" + std::to_string(N) + ")");
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal", "ipopt",
            N);
    dircol.get_opt_solver().set_advanced_option_string
            ("print_timing_statistics", "yes");
    // dircol.get_opt_solver().set_hessian_approximation("limited-memory");

    using TropterIterate = tropter::Iterate;
    tropter::Solution tropterSolution =
            m_guess ? dircol.solve(TropterIterate(m_guess.getRef()))
                    : dircol.solve();

    tropterSolution.write("DEBUG_sandboxSlidingMass.csv");
    dircol.print_constraint_values(tropterSolution);

    return {tropterSolution};
}
} // namespace OpenSim

using namespace OpenSim;

Model createModel() {
    // Generate motion.
    Model model;
    model.setName("sliding_mass");
    model.set_gravity(SimTK::Vec3(0, 0, 0));
    auto* body = new Body("body", 2.0, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("position");
    model.addComponent(joint);

    auto* actu = new CoordinateActuator();
    actu->setCoordinate(&coord);
    actu->setName("actuator");
    actu->setOptimalForce(1);
    model.addComponent(actu);

    return model;
}

Model createDoublePendulum() {
    Model model;
    model.setName("dp");

    using SimTK::Vec3;
    using SimTK::Inertia;

    // Create two links, each with a mass of 1 kg, center of mass at the body's
    // origin, and moments and products of inertia of zero.
    auto* b0 = new OpenSim::Body("b0", 1, Vec3(0), Inertia(1));
    model.addBody(b0);
    auto* b1  = new OpenSim::Body("b1", 1, Vec3(0), Inertia(1));
    model.addBody(b1);

    // Connect the bodies with pin joints. Assume each body is 1 m long.
    auto* j0 = new PinJoint("j0", model.getGround(), Vec3(0), Vec3(0),
            *b0, Vec3(-1, 0, 0), Vec3(0));
    auto& q0 = j0->updCoordinate();
    q0.setName("q0");
    auto* j1 = new PinJoint("j1",
            *b0, Vec3(0), Vec3(0), *b1, Vec3(-1, 0, 0), Vec3(0));
    auto& q1 = j1->updCoordinate();
    q1.setName("q1");
    model.addJoint(j0);
    model.addJoint(j1);

    auto* tau0 = new CoordinateActuator();
    tau0->setCoordinate(&j0->updCoordinate());
    tau0->setName("tau0");
    tau0->setOptimalForce(1);
    model.addComponent(tau0);

    auto* tau1 = new CoordinateActuator();
    tau1->setCoordinate(&j1->updCoordinate());
    tau1->setName("tau1");
    tau1->setOptimalForce(1);
    model.addComponent(tau1);

    return model;
}

SimTK::Vector linspace(int length, double start, double end) {
    SimTK::Vector v(length);
    for (int i = 0; i < length; ++i) {
        v[i] = start + i * (end - start) / (length - 1);
    }
    return v;
}

int main() {

    /*
    auto ocp = std::make_shared<Problem>();
    tropter::DirectCollocationSolver<double> dircol(ocp, "trapezoidal",
            "ipopt", 20);
    dircol.get_opt_solver().set_hessian_approximation("limited-memory");
    auto solution = dircol.solve();
    solution.write("DEBUG_sandboxSlidingMass.csv");
    dircol.print_constraint_values(solution);
     */

    /*{
        MocoProblem mp;
        mp.setModel(createModel());
        mp.setTimeBounds(0, {0, 5});
        mp.append_state_info({"joint/position/value", {-10, 10}, {0}, {1}});
        mp.append_state_info({"joint/position/speed", {-10, 10}, {0}, {0}});
        mp.append_control_info({"actuator", {-50, 50}});
        //mp.setStateInfo("joint/position/value", {-10, 10}, {0}, {1});
        //mp.setStateInfo("joint/position/speed", {-10, 10}, {0}, {0});
        //mp.setControlInfo("actuator", {-50, 50});

        mp.print("DEBUG_MocoProblem.xml");

        // TODO specify guess.

        mp.append_costs(MocoFinalTimeGoal());

        MocoSolver ms(mp);
        ms.solve();
    }*/


    // TODO setting an initial guess.

    if (false) {
        MocoProblem mp;
        mp.setModel(createDoublePendulum());
        mp.setTimeBounds(0, {0, 5});
        //mp.append_state_info({"j0/q0/value", {-10, 10}, {0}});
        mp.append_state_info({"j0/q0/value", {-10, 10}, {0}});
        mp.append_state_info({"j0/q0/speed", {-50, 50}, {0}, {0}});
        mp.append_state_info({"j1/q1/value", {-10, 10}, {0}});
        mp.append_state_info({"j1/q1/speed", {-50, 50}, {0}, {0}});
        mp.append_control_info({"tau0", {-50, 50}});
        mp.append_control_info({"tau1", {-50, 50}});


        mp.print("DEBUG_MocoProblemDoublePendulumSwingUp.xml");

        MocoMarkerFinalGoal endpointCost;
        endpointCost.set_frame_name("b1");
        endpointCost.set_weight(1000.0);
        endpointCost.set_point_on_frame(SimTK::Vec3(0));
        endpointCost.set_point_to_track(SimTK::Vec3(0, 2, 0));
        mp.append_costs(endpointCost);

        MocoFinalTimeGoal ftCost;
        ftCost.set_weight(0.001);
        mp.append_costs(ftCost);

        MocoSolver ms(mp);

        // TODO this uses the bounds to fill in guesses; not necessary to
        // fill in a guess for *each* variable.
        MocoTrajectory guess = ms.createGuessTemplate();
        guess.setNumTimes(2);
        guess.setTime_std({0, 1});
        guess.setState_std("j0/q0/value", {0, -SimTK::Pi});
        guess.setState_std("j1/q1/value", {0, 2 * SimTK::Pi});
        guess.setState_std("j0/q0/speed", {0, 0});
        guess.setState_std("j1/q1/speed", {0, 0});
        guess.setControl_std("tau0", {0, 0});
        guess.setControl_std("tau1", {0, 0});
        ms.setGuess(guess);

        ms.set_num_mesh_intervals(100);
        MocoTrajectory solution0 = ms.solve();
        solution0.write("DEBUG_sandboxSlidingMass_solution.sto");

        // TODO the second problem never solves well...Ipopt keeps on iterating.
        //ms.set_num_mesh_intervals(20);
        //ms.setGuess(solution0);
        //MocoTrajectory solution1 = ms.solve();

    }

    {
        MocoProblem mp;
        mp.setModel(createDoublePendulum());
        mp.setTimeBounds(0, 1);
        mp.append_state_info({"j0/q0/value", {-10, 10}});
        mp.append_state_info({"j0/q0/speed", {-50, 50}});
        mp.append_state_info({"j1/q1/value", {-10, 10}});
        mp.append_state_info({"j1/q1/speed", {-50, 50}});
        mp.append_control_info({"tau0", {-100, 100}});
        mp.append_control_info({"tau1", {-100, 100}});

        MocoSimpleTrackingCost tracking;
        tracking.set_weight(1.0);
        mp.append_costs(tracking);

        MocoSolver ms(mp);
        ms.set_num_mesh_intervals(50);
        //MocoTrajectory guess = ms.createGuessTemplate();
        //guess.setNumTimes(2);
        //guess.setTime_std({0, 1});
        //guess.setState_std("j0/q0/value", {0, 0.5 * SimTK::Pi});
        //guess.setState_std("j1/q1/value", {0, 0.5 * SimTK::Pi});
        //guess.setState_std("j0/q0/speed", {0, 0});
        //guess.setState_std("j1/q1/speed", {0, 0});
        //guess.setControl_std("tau0", {20, 1});
        //guess.setControl_std("tau1", {20, -2});
        //ms.setGuess(guess);
        MocoTrajectory solution = ms.solve();
        solution.write("DEBUG_double_pendulum_tracking.sto");

    }

    return 0;
}


