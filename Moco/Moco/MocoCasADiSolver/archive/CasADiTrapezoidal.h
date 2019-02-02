#ifndef MOCO_CASADITRAPEZOIDAL_H
#define MOCO_CASADITRAPEZOIDAL_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: CasADiTrapezoidal.h                                      *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2018 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s):                                                                 *
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

#include "CasADiTranscription.h"

class CasADiTrapezoidal : public CasADiTranscription {
public:

    class DynamicsFunction;

    using CasADiTranscription::CasADiTranscription;

    DM createIntegralQuadratureCoefficients(const DM& meshIntervals)
    const override {
        DM trapezoidalQuadCoeffs(m_numTimes, 1);
        trapezoidalQuadCoeffs(Slice(0, m_numTimes - 1)) = 0.5 * meshIntervals;
        trapezoidalQuadCoeffs(Slice(1, m_numTimes)) += 0.5 * meshIntervals;
        return trapezoidalQuadCoeffs;
    }

    void addConstraintsImpl() override;

protected:
    void createVariables() override {
        m_vars[Var::initial_time] = m_opti.variable();
        m_vars[Var::final_time] = m_opti.variable();
        m_vars[Var::states] = m_opti.variable(m_numStates, m_numTimes);
        m_vars[Var::controls] = m_opti.variable(m_numControls, m_numTimes);
        m_vars[Var::parameters] = m_opti.variable(m_numParameters, 1);

        m_mesh = DM::linspace(0, 1, m_numTimes);

        // Multibody constraints.
        // ----------------------
        std::vector<std::string> mcNames =
                m_probRep.createKinematicConstraintNames();
        m_numMultipliers = 0;
        m_numMultibodyConstraintEqs = 0;
        for (const auto& mcName : mcNames) {
            const auto& mc = m_probRep.getKinematicConstraint(mcName);
            int cid = mc.getSimbodyConstraintIndex();
            int mp = mc.getNumPositionEquations();
            int mv = mc.getNumVelocityEquations();
            int ma = mc.getNumAccelerationEquations();
            // Only considering holonomic constraints for now.
            OPENSIM_THROW_IF(mv != 0, Exception,
                    "Only holonomic "
                    "(position-level) constraints are currently supported. "
                    "There are " + std::to_string(mv) + " velocity-level "
                    "scalar constraints associated with the model Constraint "
                    "at ConstraintIndex " + std::to_string(cid) + ".");
            OPENSIM_THROW_IF(ma != 0, Exception,
                    "Only holonomic "
                    "(position-level) constraints are currently supported. "
                    "There are " + std::to_string(ma) + " acceleration-level "
                    "scalar constraints associated with the model Constraint "
                    "at ConstraintIndex " + std::to_string(cid) + ".");
            m_numMultipliers += mp;
            m_numMultibodyConstraintEqs += mp; // 3 * mp + 2 * mv + ma;
        }
        // TODO: This isn't true when we enforce the derivatives of constraints.
        m_vars[Var::multipliers] =
                m_opti.variable(m_numMultipliers, m_numTimes);
        m_lowerBounds[Var::multipliers] = DM(m_numMultipliers, m_numTimes);
        m_upperBounds[Var::multipliers] = DM(m_numMultipliers, m_numTimes);

        int multIndex = 0;
        for (const auto& mcName : mcNames) {
            const auto& mc = m_probRep.getKinematicConstraint(mcName);
            const auto& multInfos = m_probRep.getMultiplierInfos(mcName);
            const int mp = mc.getNumPositionEquations();
            const auto kinLevels = mc.getKinematicLevels();

            int numEquationsEnforced = mp;
            // Loop through all scalar constraints associated with the model
            // constraint.
            //
            // We need a different index for the Lagrange multipliers since
            // they are only added if the current constraint equation is not a
            // derivative of a position- or velocity-level equation.
            int multIndexThisConstraint = 0;
            for (int i = 0; i < numEquationsEnforced; ++i) {

                // If the index for this path constraint represents an
                // a non-derivative scalar constraint equation, also add a
                // Lagrange multplier to the problem.
                if (kinLevels[i] == KinematicLevel::Position ||
                        kinLevels[i] == KinematicLevel::Velocity ||
                        kinLevels[i] == KinematicLevel::Acceleration) {

                    const auto& multInfo = multInfos[multIndexThisConstraint];
                    m_multiplierNames.push_back(multInfo.getName());
                    setVariableBounds(Var::multipliers,
                            multIndex, Slice(), multInfo.getBounds());
                    setVariableBounds(Var::multipliers,
                            multIndex, 0, multInfo.getInitialBounds());
                    setVariableBounds(Var::multipliers,
                            multIndex, -1, multInfo.getFinalBounds());
                    ++multIndex;
                    ++multIndexThisConstraint;
                }
            }
        }
    }

private:
    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;

};

/// Currently, this class only returns the right-hand-side of the generalized
/// accelerations and auxiliary differential equations. Currently, we only
/// support models for which qdot = u. That is, the N matrix (qdot = N u) is
/// identity. This allows us to compute qdot symbolically through CasADi.
/// TODO: Rename to MultibodyDAEFunction
class CasADiTrapezoidal::DynamicsFunction : public CasADiFunction {
public:
    DynamicsFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MocoProblemRep& problem,
            casadi::Dict opts=casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
    ~DynamicsFunction() override {}
    casadi_int get_n_in() override { return 5; }
    casadi_int get_n_out() override { return 2; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "multipliers";
        case 4: return "parameters";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override {
        switch (i) {
        case 0: return "dynamics";
        case 1: return "multibody_constraint_errors";
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
            return casadi::Sparsity::dense(m_transcrip.getNumMultipliers(), 1);
        } else if (i == 4) {
            return casadi::Sparsity::dense(p.getNumParameters(), 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override {
        if (i == 0) {
            int numRows = m_transcrip.m_state.getNU() +
                    m_transcrip.m_state.getNZ();
            return casadi::Sparsity::dense(numRows, 1);
        } else if (i == 1) {
            int numRows = m_transcrip.getNumMultibodyConstraintEquations();
            return casadi::Sparsity::dense(numRows, 1);
        }
        else return casadi::Sparsity(0, 0);
    }
    void init() override {}

    int eval(const double** arg, double** res, casadi_int*, double*, void*)
    const override;
private:
    // This member variable avoids unnecessary extra allocation of memory for
    // spatial accelerations, which are incidental to the computation of
    // generalized accelerations when specifying the dynamics with model
    // constraints present.
    mutable SimTK::Vector_<SimTK::SpatialVec> A_GB;
};

class CasADiTrapezoidalImplicit : public CasADiTrapezoidal {
    using CasADiTrapezoidal::CasADiTrapezoidal;
    void addConstraintsImpl() override;
    void createVariables() override {
        CasADiTrapezoidal::createVariables();
        m_vars[Var::derivatives] = m_opti.variable(m_state.getNU(), m_numTimes);
    }
private:

    class DynamicsFunction;
    class ResidualFunction;
    std::unique_ptr<DynamicsFunction> m_dynamicsFunction;
    std::unique_ptr<ResidualFunction> m_residualFunction;
};

class CasADiTrapezoidalImplicit::DynamicsFunction : public CasADiFunction {
public:
    DynamicsFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MocoProblemRep& problem,
            casadi::Dict opts=casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
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
        case 0: return "auxiliary_dynamics";
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
    casadi::Sparsity get_sparsity_out(casadi_int i) override;
    void init() override {}

    int eval(const double** arg, double** res, casadi_int*, double*, void*)
    const override;
};

class CasADiTrapezoidalImplicit::ResidualFunction : public CasADiFunction {
public:
    ResidualFunction(const std::string& name,
            const CasADiTranscription& transcrip,
            const OpenSim::MocoProblemRep& problem,
            casadi::Dict opts=casadi::Dict())
            : CasADiFunction(transcrip, problem) {
        setCommonOptions(opts);
        construct(name, opts);
    }
    ~ResidualFunction() override {}
    casadi_int get_n_in() override { return 5; }
    casadi_int get_n_out() override { return 1; }
    std::string get_name_in(casadi_int i) override {
        switch (i) {
        case 0: return "time";
        case 1: return "states";
        case 2: return "controls";
        case 3: return "accelerations";
        case 4: return "parameters";
        default:
            OPENSIM_THROW(Exception, "Internal error.");
        }
    }
    std::string get_name_out(casadi_int i) override {
        switch (i) {
        case 0: return "residual";
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
            return casadi::Sparsity::dense(m_transcrip.m_state.getNU(), 1);
        } else if (i == 4) {
            return casadi::Sparsity::dense(p.getNumParameters(), 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    }
    casadi::Sparsity get_sparsity_out(casadi_int i) override;
    void init() override {}

    int eval(const double** arg, double** res, casadi_int*, double*, void*)
    const override;
};

#endif // MOCO_CASADITRAPEZOIDAL_H
