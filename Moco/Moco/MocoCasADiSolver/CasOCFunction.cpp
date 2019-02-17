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

casadi::Sparsity calcJacobianSparsityWithPerturbation(const casadi::DM& x0,
        int numOutputs,
        std::function<void(const casadi::DM&, casadi::DM&)> function) {
    using casadi::DM;
    using casadi::Sparsity;
    using std::isnan;
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
                std::cout << "[tropter] Warning: NaN encountered when "
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
}

/*
casadi::Sparsity Function::get_jacobian_sparsity() const {
    using casadi::DM;
    using casadi::Slice;

    const DM x0 = getPointForSparsityDetection();

    auto function = [this](const casadi::DM& x, casadi::DM& y) {
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

        std::vector<casadi::DM> out = this->eval(in);

        {
            int offset = 0;
            for (int iout = 0; iout < this->n_out(); ++iout) {
                OPENSIM_THROW_IF(this->size2_in(iout) != 1, OpenSim::Exception,
                        "Internal error.");
                const auto size = this->size1_out(iout);
                y(Slice(offset, offset + size)) = out;
                offset += size;
            }
        }
    };

    return calcJacobianSparsityWithPerturbation(
            x0, (int)this->nnz_out(), function);
}
*/

casadi::Sparsity PathConstraint::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity IntegralCostIntegrand::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity EndpointCost::get_sparsity_in(casadi_int i) {
    // TODO: Detect this sparsity.
    if (i == 0) {
        return casadi::Sparsity::scalar();
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

template <bool CalcKinConErrors>
casadi::Sparsity MultibodySystem<CalcKinConErrors>::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumMultipliers(), 1);
    } else if (i == 4) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

template <bool CalcKinConErrors>
casadi::Sparsity MultibodySystem<CalcKinConErrors>::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(m_casProblem->getNumSpeeds(), 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryStates(), 1);
    } else if (i == 2) {
        if (CalcKinConErrors) {
            int numRows = m_casProblem->getNumKinematicConstraintEquations();
            return casadi::Sparsity::dense(numRows, 1);
        } else {
            return casadi::Sparsity(0, 0);
        }
    } else {
        return casadi::Sparsity(0, 0);
    }
}

template class CasOC::MultibodySystem<false>;
template class CasOC::MultibodySystem<true>;

casadi::Sparsity VelocityCorrection::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumSlacks(), 1);
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

casadi::Sparsity MultibodySystemImplicit::get_sparsity_in(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(1, 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(m_casProblem->getNumStates(), 1);
    } else if (i == 2) {
        return casadi::Sparsity::dense(m_casProblem->getNumControls(), 1);
    } else if (i == 3) {
        return casadi::Sparsity::dense(m_casProblem->getNumMultipliers(), 1);
    } else if (i == 4) {
        return casadi::Sparsity::dense(m_casProblem->getNumSpeeds(), 1);
    } else if (i == 5) {
        return casadi::Sparsity::dense(m_casProblem->getNumParameters(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}

casadi::Sparsity MultibodySystemImplicit::get_sparsity_out(casadi_int i) {
    if (i == 0) {
        return casadi::Sparsity::dense(m_casProblem->getNumSpeeds(), 1);
    } else if (i == 1) {
        return casadi::Sparsity::dense(
                m_casProblem->getNumAuxiliaryStates(), 1);
    } else {
        return casadi::Sparsity(0, 0);
    }
}
