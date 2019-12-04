/* -------------------------------------------------------------------------- *
 * OpenSim Moco: MocoCasOCFunction.cpp                                        *
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

#include "CasOCFunction.h"

#include "CasOCProblem.h"

using namespace CasOC;

casadi::Sparsity calcJacobianSparsityWithPerturbation(const VectorDM& x0s,
        int numOutputs,
        std::function<void(const casadi::DM&, casadi::DM&)> function) {

    OPENSIM_THROW_IF(x0s.size() < 1, OpenSim::Exception,
            "x0s must have at least 1 element.");
    using casadi::DM;
    using casadi::Sparsity;
    using std::isnan;
    auto sparsityForSingleX0 = [&](const casadi::DM& x0) {
        OPENSIM_THROW_IF(x0.columns() != 1, OpenSim::Exception,
                "x0 must have exactly 1 column.");
        Sparsity sparsity(numOutputs, x0.numel());
        double eps = 1e-5;
        DM x = x0;
        DM output0(numOutputs, 1);
        function(x, output0);
        DM output(numOutputs, 1);
        DM diff(numOutputs, 1);
        for (int j = 0; j < x0.numel(); ++j) {
            output = 0;
            x(j) += eps;
            function(x, output);
            x(j) = x0(j);
            diff = output - output0;
            for (int i = 0; i < (int)numOutputs; ++i) {
                if (std::isnan(diff(i).scalar())) {
                    std::cout << "[CasOC] Warning: NaN encountered when "
                                 "detecting sparsity of Jacobian; entry (";
                    std::cout << i << ", " << j;
                    std::cout << ")." << std::endl;
                    // Set non-zero here just in case this Jacobian element is
                    // important.
                    sparsity.add_nz(i, j);
                }
                if (diff(i).scalar() != 0) sparsity.add_nz(i, j);
            }
            diff = 0;
        }
        return sparsity;
    };

    Sparsity combinedSparsity(numOutputs, x0s[0].numel());
    for (const auto& x0 : x0s) {
        OPENSIM_THROW_IF(x0.numel() != x0s[0].numel(), OpenSim::Exception,
                "x0s contains vectors of different sizes.");
        combinedSparsity = combinedSparsity + sparsityForSingleX0(x0);
    }
    return combinedSparsity;
}

casadi::Sparsity Function::get_jacobian_sparsity() const {
    using casadi::DM;
    using casadi::Slice;

    auto function = [this](const casadi::DM& x, casadi::DM& y) {
        // Split input into separate DMs.
        std::vector<casadi::DM> in(this->n_in());
        {
            int offset = 0;
            for (int iin = 0; iin < this->n_in(); ++iin) {
                OPENSIM_THROW_IF(this->size2_in(iin) != 1, OpenSim::Exception,
                        "Internal error.");
                const auto size = this->size1_in(iin);
                in[iin] = x(Slice(offset, offset + size));
                offset += size;
            }
        }

        // Evaluate the function.
        std::vector<casadi::DM> out = this->eval(in);

        // Create output.
        y = casadi::DM::veccat(out);
    };

    const VectorDM x0s = getSubsetPointsForSparsityDetection();

    return calcJacobianSparsityWithPerturbation(
            x0s, (int)this->nnz_out(), function);
}

void Function::constructFunction(const Problem* casProblem,
        const std::string& name, const std::string& finiteDiffScheme,
        std::shared_ptr<const std::vector<VariablesDM>>
                pointsForSparsityDetection) {
    m_casProblem = casProblem;
    m_finite_difference_scheme = finiteDiffScheme;
    m_fullPointsForSparsityDetection = pointsForSparsityDetection;
    casadi::Dict opts;
    setCommonOptions(opts);
    this->construct(name, opts);
}

casadi::Sparsity Function::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumMultipliers(), 1);
    } else if (i == 4) {
        return casadi::Sparsity::dense(m_casProblem->getNumDerivatives(), 1);
    } else if (i == 5) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

VectorDM PathConstraint::eval(const VectorDM& args) const {
    Problem::ContinuousInput input{args.at(0).scalar(), args.at(1), args.at(2),
            args.at(3), args.at(4), args.at(5)};
    VectorDM out{casadi::DM(sparsity_out(0))};
    m_casProblem->calcPathConstraint(m_index, input, out[0]);
    return out;
}

VectorDM CostIntegrand::eval(const VectorDM& args) const {
    Problem::ContinuousInput input{args.at(0).scalar(), args.at(1), args.at(2),
            args.at(3), args.at(4), args.at(5)};
    VectorDM out{casadi::DM(casadi::Sparsity::scalar())};
    m_casProblem->calcCostIntegrand(m_index, input, *out[0].ptr());
    return out;
}

VectorDM EndpointConstraintIntegrand::eval(const VectorDM& args) const {
    Problem::ContinuousInput input{args.at(0).scalar(), args.at(1), args.at(2),
                                   args.at(3), args.at(4), args.at(5)};
    VectorDM out{casadi::DM(casadi::Sparsity::scalar())};
    m_casProblem->calcEndpointConstraintIntegrand(
            m_index, input, *out[0].ptr());
    return out;
}

casadi::Sparsity Endpoint::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumMultipliers(), 1);
    } else if (i == 4) {
        return casadi::Sparsity::dense(m_casProblem->getNumDerivatives(), 1);
    } else if (i == 5) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 6) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 7) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 8) {
        return casadi::Sparsity::dense(m_casProblem->getNumMultipliers(), 1);
    } else if (i == 9) {
        return casadi::Sparsity::dense(m_casProblem->getNumDerivatives(), 1);
    } else if (i == 10) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else if (i == 11) {
        return casadi::Sparsity::dense(1, 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}
VectorDM Cost::eval(const VectorDM& args) const {
    Problem::CostInput input{args.at(0).scalar(), args.at(1), args.at(2),
            args.at(3), args.at(4), args.at(5).scalar(), args.at(6), args.at(7),
            args.at(8), args.at(9), args.at(10), args.at(11).scalar()};
    VectorDM out{casadi::DM(sparsity_out(0))};
    m_casProblem->calcCost(m_index, input, out.at(0));
    return out;
}
VectorDM EndpointConstraint::eval(const VectorDM& args) const {
    Problem::CostInput input{args.at(0).scalar(), args.at(1), args.at(2),
            args.at(3), args.at(4), args.at(5).scalar(), args.at(6), args.at(7),
            args.at(8), args.at(9), args.at(10), args.at(11).scalar()};
    VectorDM out{casadi::DM(sparsity_out(0))};
    m_casProblem->calcEndpointConstraint(m_index, input, out.at(0));
    return out;
}

template <bool CalcKCErrors>
casadi::Sparsity MultibodySystemExplicit<CalcKCErrors>::get_sparsity_out(
        casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumMultibodyDynamicsEquations(), 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryResidualEquations(), 1);
    } else if (i == 3) {
        if (CalcKCErrors) {
            int numRows = m_casProblem->getNumKinematicConstraintEquations();
            return casadi::Sparsity::dense(numRows, 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    } else {
        return casadi::Sparsity(0, 0);
    }
}

template <bool CalcKCErrors>
VectorDM MultibodySystemExplicit<CalcKCErrors>::eval(
        const VectorDM& args) const {
    Problem::ContinuousInput input{args.at(0).scalar(), args.at(1), args.at(2),
            args.at(3), args.at(4), args.at(5)};
    VectorDM out((int)n_out());
    for (casadi_int i = 0; i < n_out(); ++i) {
        out[i] = casadi::DM(sparsity_out(i));
    }
    Problem::MultibodySystemExplicitOutput output{out[0], out[1], out[2],
            out[3]};
    m_casProblem->calcMultibodySystemExplicit(input, CalcKCErrors, output);
    return out;
}

template class CasOC::MultibodySystemExplicit<false>;
template class CasOC::MultibodySystemExplicit<true>;

casadi::Sparsity VelocityCorrection::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumStates() -
                        m_casProblem->getNumAuxiliaryStates(),
                1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumSlacks(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity VelocityCorrection::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(m_casProblem->getNumSpeeds(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::DM VelocityCorrection::getSubsetPoint(
        const VariablesDM& fullPoint) const {
    int itime = 0;
    using casadi::Slice;
    const int NMBS = m_casProblem->getNumStates() -
                     m_casProblem->getNumAuxiliaryStates();
    return casadi::DM::vertcat({fullPoint.at(initial_time),
            fullPoint.at(states)(Slice(0, NMBS), itime),
            fullPoint.at(slacks)(Slice(), itime), fullPoint.at(parameters)});
}

VectorDM VelocityCorrection::eval(const VectorDM& args) const {
    VectorDM out{casadi::DM(sparsity_out(0))};
    m_casProblem->calcVelocityCorrection(
            args.at(0).scalar(), args.at(1), args.at(2), args.at(3), out[0]);
    return out;
}

template <bool CalcKCErrors>
casadi::Sparsity MultibodySystemImplicit<CalcKCErrors>::get_sparsity_out(
        casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumMultibodyDynamicsEquations(), 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryResidualEquations(), 1);
    } else if (i == 3) {
        if (CalcKCErrors) {
            int numRows = m_casProblem->getNumKinematicConstraintEquations();
            return casadi::Sparsity::dense(numRows, 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    } else {
        return casadi::Sparsity(0, 0);
    }
}

template <bool CalcKCErrors>
VectorDM MultibodySystemImplicit<CalcKCErrors>::eval(
        const VectorDM& args) const {
    Problem::ContinuousInput input{args.at(0).scalar(), args.at(1), args.at(2),
            args.at(3), args.at(4), args.at(5)};
    VectorDM out((int)n_out());
    for (casadi_int i = 0; i < n_out(); ++i) {
        out[i] = casadi::DM(sparsity_out(i));
    }

    Problem::MultibodySystemImplicitOutput output{out[0], out[1], out[2], 
            out[3]};
    m_casProblem->calcMultibodySystemImplicit(input, CalcKCErrors, output);
    return out;
}

template class CasOC::MultibodySystemImplicit<false>;
template class CasOC::MultibodySystemImplicit<true>;
