#ifndef MUSCOLLO_CASADITRANSCRIPTION_H
#define MUSCOLLO_CASADITRANSCRIPTION_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTranscription.h                                    *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
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

#include "../MuscolloUtilities.h"
#include "../MucoProblemRep.h"
#include "MucoCasADiSolver.h"
#include <casadi/casadi.hpp>

// TODO: temporary
using namespace OpenSim;
using casadi::MX;
using casadi::DM;
using casadi::Sparsity;
using casadi::Slice;
using casadi::Callback;
using casadi::Dict;

class CasADiTranscription;

class CasADiFunction : public casadi::Callback {
public:
    CasADiFunction(
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem)
            : m_transcrip(transcrip), p(problem) {
    }
    // Concrete class constructors should invoke this before construct().
    void setCommonOptions(Dict& opts) {
        opts["enable_fd"] = true;
        opts["fd_method"] = "central";
        // Using "forward", iterations are 10x faster but problems don't
        // converge.
    }
protected:
    const CasADiTranscription& m_transcrip;
    const OpenSim::MucoProblemRep& p;

};

// TODO: Create a base class for all of these callback functions.
class EndpointCostFunction : public CasADiFunction {
public:
    EndpointCostFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts = casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
    ~EndpointCostFunction() override {}
    casadi_int get_n_in() override { return 3; }
    casadi_int get_n_out() override { return 1; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "parameters";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override {
        switch (i) {
        case 0: return "endpoint_cost";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    void init() override {}
    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        // TODO fix when using a matrix as input for states.
        // TODO detect this sparsity.
        if (i == 0) {
            return casadi::Sparsity::scalar();
        } else if (i == 1) {
            return casadi::Sparsity::dense(p.getNumStates(), 1);
        } else if (i == 2) {
            return casadi::Sparsity::dense(p.getNumParameters(), 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return casadi::Sparsity::scalar();
        else return casadi::Sparsity(0, 0);
    }
    /// Arguments:
    /// 0. time
    /// 1. final state
    /// 2. parameters
    int eval(const double** arg, double** res, casadi_int*, double*, void*)
            const override;

};

class IntegrandCostFunction : public CasADiFunction {
public:
    IntegrandCostFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MucoProblemRep& problem,
            casadi::Dict opts = casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
    ~IntegrandCostFunction() override {}
    /// 0. time
    /// 1. states
    /// 2. controls
    /// 3. parameters
    casadi_int get_n_in() override { return 4; }
    casadi_int get_n_out() override { return 1; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "parameters";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override {
        switch (i) {
        case 0: return "integral_cost_integrand";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    casadi::Sparsity get_sparsity_in(casadi_int i) override {
        if (i == 0) {
            return casadi::Sparsity::dense(1, 1);
        } else if (i == 1) {
            return casadi::Sparsity::dense(p.getNumStates(), 1);
        } else if (i == 2) {
            return casadi::Sparsity::dense(p.getNumControls(), 1);
        } else if (i == 3) {
            return casadi::Sparsity::dense(p.getNumParameters(), 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) return casadi::Sparsity::scalar();
        else return casadi::Sparsity(0, 0);
    }
    // Use the more efficient virtual function (not the eval() that uses DMs)
    // to avoid any overhead.
    int eval(const double** arg, double** res, casadi_int*, double*, void*)
            const override;

    // TODO: Should I provide a Jacobian *and* a forward?
    /*
    bool has_forward(casadi_int ifwd) const override {
        return ifwd == 1;
    }
    Function get_forward(casadi_int ifwd, const std::string& name,
            const std::vector<std::string>& inames,
            const std::vector<std::string>& onames,
            const Dict& opts) const override {
        std::cout << "DEBUGname " << name << std::endl;
        std::cout << "DEBUG " << inames.size() << std::endl;
        for (const auto& name : inames) std::cout << name << std::endl;
        std::cout << "DEBUG " << onames.size() << std::endl;
        for (const auto& name : onames) std::cout << name << std::endl;

        MX time = MX::sym(inames[0]);
        MX states = MX::sym(inames[1], p.getNumStates(), 1);
        MX controls = MX::sym(inames[2], p.getNumControls(), 1);
        MX parameters = MX::sym(inames[3], p.getNumParameters(), 1);
        MX out_integrand = MX::sym(inames[4]);
        std::vector<MX> in =
                {time, states, controls, parameters, out_integrand};
        MX fwd = MX::sym(onames[0],  )
        std::vector<MX> out =
                {};
        return Function(name, in, out, inames, onames, opts);
    }
    bool has_jacobian() const override { return true; }
    Function get_jacobian(const std::string& name,
            const std::vector<std::string>& inames,
            const std::vector<std::string>& onames,
            const Dict& opts) const override {

        std::cout << "DEBUGname " << name << std::endl;
        std::cout << "DEBUG " << inames.size() << std::endl;
        for (const auto& name : inames) std::cout << name << std::endl;
        std::cout << "DEBUG " << onames.size() << std::endl;
        for (const auto& name : onames) std::cout << name << std::endl;

        MX time = MX::sym(inames[0]);
        MX states = MX::sym(inames[1], p.getNumStates(), 1);
        MX controls = MX::sym(inames[2], p.getNumControls(), 1);
        MX parameters = MX::sym(inames[3], p.getNumParameters(), 1);
        MX out_integrand = MX::sym(inames[4]);
        std::vector<MX> in =
                {time, states, controls, parameters, out_integrand};
        MX jac = MX::sym(onames[0], )
        std::vector<MX> out =
                {};
        return Function(name, in, out, inames, onames, opts);
    }*/
};

enum Var {
    initial_time,
    final_time,
    states,
    controls,
    derivatives,
    parameters
};
template <typename T>
using CasADiVariables = std::unordered_map<Var, T, std::hash<int>>;

class CasADiTranscription {
public:
    CasADiTranscription(const MucoCasADiSolver& solver,
            const MucoProblemRep& probRep)
            : m_solver(solver), m_probRep(probRep),
              m_model(m_probRep.getModel()),
              m_state(m_model.getWorkingState()) {}

    virtual ~CasADiTranscription() = default;

    template <typename T>
    T createTimes(const T& initialTime, const T& finalTime) const {
        return (finalTime - initialTime) * m_mesh + initialTime;
    }

    /// This converts a SimTK::Matrix to a casadi::DM matrix, transposing the
    /// data in the process.
    static DM convertToCasADiDM(const SimTK::Matrix& simtkMatrix) {
        DM out(simtkMatrix.ncol(), simtkMatrix.nrow());
        for (int irow = 0; irow < simtkMatrix.nrow(); ++irow) {
            for (int icol = 0; icol < simtkMatrix.ncol(); ++icol) {
                out(icol, irow) = simtkMatrix(irow, icol);
            }
        }
        return out;
    }
    /// This converts a SimTK::RowVector to a casadi::DM column vector.
    static DM convertToCasADiDM(const SimTK::RowVector& simtkRV) {
        DM out(simtkRV.size(), 1);
        for (int i = 0; i < simtkRV.size(); ++i) {
            out(i) = simtkRV[i];
        }
        return out;
    }
    /// This resamples the iterate to obtain values that lie on the mesh.
    CasADiVariables<DM>
    convertToCasADiVariables(MucoIterate mucoIt) const {
        CasADiVariables<DM> casVars;
        casVars[Var::initial_time] = mucoIt.getInitialTime();
        casVars[Var::final_time] = mucoIt.getFinalTime();
        const auto timesValue = createTimes(casVars[Var::initial_time],
                        casVars[Var::final_time]);
        SimTK::Vector simtkTimes = convertToSimTKVector(timesValue);
        mucoIt.resample(simtkTimes);
        casVars[Var::states] = convertToCasADiDM(mucoIt.getStatesTrajectory());
        casVars[Var::controls] =
                convertToCasADiDM(mucoIt.getControlsTrajectory());
        // TODO dimensions?
        casVars[Var::parameters] = convertToCasADiDM(mucoIt.getParameters());
        return casVars;
    }
    /// This converts a casadi::DM matrix to a
    /// SimTK::Matrix, transposing the data in the process.
    SimTK::Matrix convertToSimTKMatrix(const DM& casMatrix) const {
        SimTK::Matrix simtkMatrix(
                (int)casMatrix.columns(), (int)casMatrix.rows());
        for (int irow = 0; irow < casMatrix.rows(); ++irow) {
            for (int icol = 0; icol < casMatrix.columns(); ++icol) {
                simtkMatrix(icol, irow) = double(casMatrix(irow, icol));
            }
        }
        return simtkMatrix;
    }
    template <typename VectorType = SimTK::Vector>
    VectorType convertToSimTKVector(const DM& casVector) const {
        assert(casVector.columns() == 1);
        VectorType simtkVector((int)casVector.rows());
        for (int i = 0; i < casVector.rows(); ++i) {
            simtkVector[i] = double(casVector(i));
        }
        return simtkVector;
    }
    template <typename InvokeOn>
    CasADiVariables<DM>
    convertToCasADiVariablesDM(
            const InvokeOn& obj, const CasADiVariables<MX>& varsMX) {
        CasADiVariables<DM> varsDM;
        for (const auto& kv : varsMX) {
            varsDM[kv.first] = obj.value(kv.second);
        }
        return varsDM;
    }
    template <typename TOut = MucoIterate>
    TOut convertToMucoIterate(const CasADiVariables<DM>& casVars) const {
        SimTK::Matrix simtkStates;
        if (m_numStates) {
            simtkStates = convertToSimTKMatrix(
                    m_opti.value(casVars.at(Var::states)));
        }
        SimTK::Matrix simtkControls;
        if (m_numControls) {
            simtkControls = convertToSimTKMatrix(
                    m_opti.value(casVars.at(Var::controls)));
        }
        SimTK::RowVector simtkParameters;
        if (m_numParameters) {
            const auto paramsValue = m_opti.value(casVars.at(Var::parameters));
            simtkParameters =
                    convertToSimTKVector<SimTK::RowVector>(paramsValue);
        }

        const auto timesValue = m_opti.value(createTimes(
                        casVars.at(Var::initial_time),
                        casVars.at(Var::final_time)));
        SimTK::Vector simtkTimes = convertToSimTKVector(timesValue);
        TOut mucoIterate(simtkTimes,
                m_stateNames, m_controlNames, {}, m_parameterNames,
                simtkStates, simtkControls, {}, simtkParameters);
        return mucoIterate;
    }
    /// Create an initial guess for this problem according to the
    /// following rules:
    ///   - unconstrained variable: 0.
    ///   - lower and upper bounds: midpoint of the bounds.
    ///   - only one bound: value of the bound.
    MucoIterate createInitialGuessFromBounds() const {
        auto setToMidpoint = [](DM& output, const DM& lowerDM,
                const DM& upperDM) {
            for (int irow = 0; irow < output.rows(); ++irow) {
                for (int icol = 0; icol < output.columns(); ++icol) {
                    const auto& lower = double(lowerDM(irow, icol));
                    const auto& upper = double(upperDM(irow, icol));
                    if (!std::isinf(lower) && !std::isinf(upper)) {
                        output(irow, icol) = 0.5 * (upper + lower);
                    }
                    else if (!std::isinf(lower)) output(irow, icol) = lower;
                    else if (!std::isinf(upper)) output(irow, icol) = upper;
                    else output(irow, icol) = 0;
                }
            }
        };
        CasADiVariables<DM> casGuess = m_lowerBounds;
        for (auto& kv : casGuess) {
            setToMidpoint(kv.second,
                    m_lowerBounds.at(kv.first), m_upperBounds.at(kv.first));
        }

        return convertToMucoIterate(casGuess);
    }
    /// Create a vector with random variable values within the variable
    /// bounds, potentially for use as an initial guess. If, for a given
    /// variable, either bound is infinite, then the element is a random number
    /// in [-1, 1] clamped by the bounds.
    MucoIterate createRandomIterateWithinBounds() const {
        static SimTK::Random::Uniform randGen(-1, 1);
        auto setRandom = [](DM& output, const DM& lowerDM,
                const DM& upperDM) {
            for (int irow = 0; irow < output.rows(); ++irow) {
                for (int icol = 0; icol < output.columns(); ++icol) {
                    const auto& lower = double(lowerDM(irow, icol));
                    const auto& upper = double(upperDM(irow, icol));
                    const auto rand = randGen.getValue();
                    auto value = 0.5 * (rand + 1.0) * (upper - lower) + lower;
                    if (std::isnan(value))
                        value = SimTK::clamp(lower, rand, upper);
                    output(irow, icol) = value;
                }
            }
        };
        CasADiVariables<DM> casIterate = m_lowerBounds;
        for (auto& kv : casIterate) {
            setRandom(kv.second,
                    m_lowerBounds.at(kv.first), m_upperBounds.at(kv.first));
        }

        return convertToMucoIterate(casIterate);
    }
    void initialize() {
        OPENSIM_THROW_IF(m_initialized, Exception, "Can only initialize once.");
        m_initialized = true;
        m_opti = casadi::Opti();

        m_numTimes = m_solver.get_num_mesh_points();

        // Get number and names of variables.
        // ----------------------------------
        // TODO: Add this as a method to MucoProblemRep.
        m_stateNames = createStateVariableNamesInSystemOrder(m_model);
        m_numStates = (int)m_stateNames.size();
        // TODO use getControlNames().
        m_numControls = [&]() {
            int count = 0;
            for (const auto& actuator : m_model.getComponentList<Actuator>()) {
                // TODO check if it's enabled.
                actuator.getName();
                ++count;
            }
            return count;
        }();

        m_parameterNames = m_probRep.createParameterNames();
        m_numParameters = (int)m_parameterNames.size();

        // Create variables and set bounds.
        // --------------------------------
        createVariables();
        auto initializeBounds = [&](CasADiVariables<DM>& bounds) {
            for (auto& kv : m_vars) {
                bounds[kv.first] = DM(kv.second.rows(), kv.second.columns());
            }
        };
        initializeBounds(m_lowerBounds);
        initializeBounds(m_upperBounds);

        setVariableBounds(Var::initial_time, 0, 0,
                m_probRep.getTimeInitialBounds());
        setVariableBounds(Var::final_time, 0, 0,
                m_probRep.getTimeFinalBounds());

        for (int is = 0; is < m_numStates; ++is) {
            const auto& info = m_probRep.getStateInfo(m_stateNames[is]);
            const auto& bounds = info.getBounds();
            auto initialBounds = info.getInitialBounds();
            auto finalBounds = info.getFinalBounds();
            // TODO do not specify bounds twice for endpoints.
            setVariableBounds(Var::states, is, Slice(), bounds);
            setVariableBounds(Var::states, is, 0, initialBounds);
            // Last state can be obtained via -1.
            setVariableBounds(Var::states, is, -1, finalBounds);
        }

        int ic = 0;
        for (const auto& actuator : m_model.getComponentList<Actuator>()) {
            const auto actuPath = actuator.getAbsolutePathString();
            m_controlNames.push_back(actuPath);
            const auto& info = m_probRep.getControlInfo(actuPath);
            const auto& bounds = info.getBounds();
            const auto& initialBounds = info.getInitialBounds();
            const auto& finalBounds = info.getFinalBounds();
            setVariableBounds(Var::controls, ic, Slice(), bounds);
            setVariableBounds(Var::controls, ic, 0, initialBounds);
            setVariableBounds(Var::controls, ic, -1, finalBounds);
            ++ic;
        }

        for (int iparam = 0; iparam < m_numParameters; ++iparam) {
            const auto& param =
                    m_probRep.getParameter(m_parameterNames[iparam]);
            setVariableBounds(Var::parameters,
                    iparam, 0, param.getBounds());
        }

        m_duration = m_vars[Var::final_time] - m_vars[Var::initial_time];
        DM meshIntervals = m_mesh(Slice(1, m_mesh.rows())) -
                m_mesh(Slice(0, m_mesh.rows() - 1));
        m_times =
                createTimes(m_vars[Var::initial_time], m_vars[Var::final_time]);

        addDefectConstraints();

        m_endpointCostFunction =
                make_unique<EndpointCostFunction>("endpoint_cost", *this, m_probRep);

        // TODO: Evaluate individual endpoint costs separately.
        auto endpoint_cost = m_endpointCostFunction->operator()(
                {m_vars[Var::final_time],
                 m_vars[Var::states](Slice(), -1),
                 m_vars[Var::parameters]}).at(0);

        DM quadCoeffs = createIntegralQuadratureCoefficients(meshIntervals);

        m_integrandCostFunction =
                make_unique<IntegrandCostFunction>("integrand", *this, m_probRep);
        MX integral_cost = 0;
        for (int i = 0; i < m_numTimes; ++i) {
            const auto out = m_integrandCostFunction->operator()(
                    {m_times(i, 0),
                     m_vars[Var::states](Slice(), i),
                     m_vars[Var::controls](Slice(), i),
                     m_vars[Var::parameters]});
            integral_cost += quadCoeffs(i) * out.at(0);
            // Testing the performance benefit of providing a cost with CasADi
            // directly.
            /*
            const auto& controls = m_vars[Var::controls](Slice(), i);
            integral_cost += quadCoeffs(i) * dot(controls, controls);
             */
        }
        integral_cost *= m_duration;
        m_opti.minimize(endpoint_cost + integral_cost);
    }

    void setGuess(const MucoIterate& guess) {
        const CasADiVariables<DM> casGuess =
                convertToCasADiVariables(guess);
        for (auto& kv : m_vars) {
            m_opti.set_initial(kv.second, casGuess.at(kv.first));
        }
    }

    Dict getStats() { return m_opti.stats(); }

    MucoSolution
    solve(const std::string& solver, Dict pluginOptions, Dict solverOptions) {
        m_opti.solver(solver, pluginOptions, solverOptions);
        try {
            m_opti.solve();
            return convertToMucoIterate<MucoSolution>(
                    convertToCasADiVariablesDM(m_opti, m_vars));
        } catch (const std::exception& e) {
            return convertToMucoIterate<MucoSolution>(
                    convertToCasADiVariablesDM(m_opti.debug(), m_vars));
        }
        // Print constraint violations from Opti.
        // opt.debug().show_infeasibilities();
        // DM controlValues = opt.debug().value(controls);
        // std::cout << "DEBUGg43 " << opt.debug().g_describe(43) << std::endl;
        // std::cout << "DEBUGg43 " << opt.g()(43) << std::endl;
        // std::cout << "DEBUGg43 " << opt.x() << std::endl;
        // TODO: Return a solution (sealed).
    }

    /// @precondition The following are set: m_numTimes, m_numStates,
    ///     m_numControls.
    /// @postcondition All fields in member variable m_vars are set, and
    ///     and m_mesh is set.
    virtual void createVariables() = 0;
    void addDefectConstraints() {
        if (m_numStates) addDefectConstraintsImpl();
    }
    virtual void addDefectConstraintsImpl() = 0;
    virtual DM createIntegralQuadratureCoefficients(const DM& meshIntervals)
    const = 0;

    void applyParametersToModel(const SimTK::Vector& parameters) const
    {
        if (m_numParameters) {
            m_probRep.applyParametersToModel(parameters);
            const_cast<Model&>(m_model).initSystem();
        }
    }

    const MucoCasADiSolver& m_solver;
    const MucoProblemRep& m_probRep;
    const Model& m_model;
    mutable SimTK::State m_state;

protected:
    template <typename TRow, typename TColumn>
    void setVariableBounds(Var var,
            const TRow& rowIndices, const TColumn& columnIndices,
            const MucoBounds& bounds) {
        if (bounds.isSet()) {
            const auto& lower = bounds.getLower();
            m_lowerBounds[var](rowIndices, columnIndices) = lower;
            const auto& upper = bounds.getUpper();
            m_upperBounds[var](rowIndices, columnIndices) = upper;
            /*
            auto submatrix = m_vars[var](rowIndices, columnIndices);
            for (int irow = 0; irow < submatrix.rows(); ++irow) {
                for (int icol = 0; icol < submatrix.columns(); ++icol) {
                    m_opti.subject_to(lower <= submatrix(irow, icol) <= upper);
                }
            }
            */
            m_opti.subject_to(
                    lower <= m_vars[var](rowIndices, columnIndices) <= upper);
        } else {
            m_lowerBounds[var](rowIndices, columnIndices) = -SimTK::Infinity;
            m_upperBounds[var](rowIndices, columnIndices) =  SimTK::Infinity;
        }
    }
    bool m_initialized = false;
    casadi::Opti m_opti;
    CasADiVariables<casadi::MX> m_vars;
    CasADiVariables<casadi::DM> m_lowerBounds;
    CasADiVariables<casadi::DM> m_upperBounds;
    int m_numTimes = -1;
    int m_numStates = -1;
    int m_numControls = -1;
    int m_numParameters = -1;
    MX m_times;
    MX m_duration;
    DM m_mesh;
    std::vector<std::string> m_stateNames;
    std::vector<std::string> m_controlNames;
    std::vector<std::string> m_parameterNames;

    std::unique_ptr<EndpointCostFunction> m_endpointCostFunction;
    std::unique_ptr<IntegrandCostFunction> m_integrandCostFunction;
};

// TODO: Move to a better place.
inline void convertToSimTKState(
        const double* time, const double* states, const double* controls,
        const Model& model, SimTK::State& simtkState) {
    simtkState.setTime(time[0]);
    simtkState.setY(SimTK::Vector(simtkState.getNY(), states, true));
    std::copy_n(states, simtkState.getNY(),
            simtkState.updY().updContiguousScalarData());
    auto& simtkControls = model.updControls(simtkState);
    std::copy_n(controls, simtkControls.size(),
            simtkControls.updContiguousScalarData());
    model.realizeVelocity(simtkState);
    model.setControls(simtkState, simtkControls);
}

#endif // MUSCOLLO_CASADITRANSCRIPTION_H
