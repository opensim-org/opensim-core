/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: DeGrooteFregly2016Muscle.cpp                             *
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

#include "DeGrooteFregly2016Muscle.h"

using namespace OpenSim;

const std::string DeGrooteFregly2016Muscle::ACTIVATION("activation");
const std::string DeGrooteFregly2016Muscle::
        NORM_FIBER_LENGTH("norm_fiber_length");

void DeGrooteFregly2016Muscle::extendFinalizeFromProperties() {
    Super::extendFinalizeFromProperties();
    OPENSIM_THROW_IF_FRMOBJ(
        !getProperty_optimal_force().getValueIsDefault(), Exception,
        "The optimal_force property is ignored for this Force; "
        "use max_isometric_force instead.");

    OPENSIM_THROW_IF_FRMOBJ(get_tendon_strain_at_one_norm_force() <= 0,
        Exception,
        "Expected the tendon_strain_at_one_norm_force property to be "
        "positive, but got " +
        std::to_string(get_tendon_strain_at_one_norm_force()) + ".");

    OPENSIM_THROW_IF_FRMOBJ(get_ignore_activation_dynamics(), Exception,
        "Not supported yet.");

    OPENSIM_THROW_IF_FRMOBJ(get_pennation_angle_at_optimal() != 0,
        Exception, "Not supported yet.");

    // TODO validate properties (nonnegative, etc.).

    m_maxContractionVelocityInMeters =
            get_max_contraction_velocity() * get_optimal_fiber_length();
    m_kT = log((1.0 + c3) / c1) /
            (1.0 + get_tendon_strain_at_one_norm_force() - c2);
}

void DeGrooteFregly2016Muscle::
extendAddToSystem(SimTK::MultibodySystem& system) const {
    Super::extendAddToSystem(system);
    addStateVariable(ACTIVATION, SimTK::Stage::Dynamics);
    if (!get_ignore_tendon_compliance()) {
        addStateVariable(NORM_FIBER_LENGTH, SimTK::Stage::Position);
    }
}

void DeGrooteFregly2016Muscle::
extendInitStateFromProperties(SimTK::State& s) const {
    Super::extendInitStateFromProperties(s);
    setStateVariableValue(s, ACTIVATION, get_default_activation());
    if (!get_ignore_tendon_compliance()) {
        setStateVariableValue(s, NORM_FIBER_LENGTH,
                get_default_norm_fiber_length());
    }
}

void DeGrooteFregly2016Muscle::
extendSetPropertiesFromState(const SimTK::State& s) {
    Super::extendSetPropertiesFromState(s);
    // TODO getActivation(s)?
    set_default_activation(getStateVariableValue(s, ACTIVATION));
    if (!get_ignore_tendon_compliance()) {
        set_default_norm_fiber_length(
                getStateVariableValue(s, NORM_FIBER_LENGTH));
    }
}

void DeGrooteFregly2016Muscle::
computeStateVariableDerivatives(const SimTK::State& s) const {
    const auto& activation = getActivation(s);

    // On a simple hanging muscle minimum time problem, I got quicker
    // convergence using the nonlinear activation dynamics from the paper.
    // const auto& tau = get_activation_time_constant();
    // const SimTK::Real activationDot = (excitation - activation) / tau;
    {
        const auto& excitation = getControl(s);
        static const double actTimeConst   = get_activation_time_constant();
        static const double deactTimeConst = get_deactivation_time_constant();
        static const double tanhSteepness  = 0.1;
        //     f = 0.5 tanh(b(e - a))
        //     z = 0.5 + 1.5a
        // da/dt = [(f + 0.5)/(tau_a * z) + (-f + 0.5)*z/tau_d] * (e - a)
        const SimTK::Real timeConstFactor = 0.5 + 1.5 * activation;
        const SimTK::Real tempAct = 1.0 / (actTimeConst * timeConstFactor);
        const SimTK::Real tempDeact = timeConstFactor / deactTimeConst;
        const SimTK::Real f =
                0.5 * tanh(tanhSteepness * (excitation - activation));
        const SimTK::Real timeConst =
                tempAct * (f + 0.5) + tempDeact * (-f + 0.5);
        setStateVariableDerivativeValue(s, ACTIVATION,
                timeConst * (excitation - activation));
    }

    if (!get_ignore_tendon_compliance()) {

        // Root-solve the following for v:
        // a f_L f_V(v) + beta * v + f_P = f_T

        const SimTK::Real muscleTendonLength = getLength(s);
        const SimTK::Real normFiberLength = calcNormalizedFiberLength(s);

        // TODO: this is inefficient because it's recalculating a lot of
        // terms that don't actually vary with normFiberVelocity (actually,
        // only one term depends on fiber velocity).
        const SimTK::Real activeForceLengthMult =
                calcActiveForceLengthMultiplier(normFiberLength);
        const SimTK::Real cosPenn = 1.0; // TODO
        const SimTK::Real activationActiveForceLengthMultiplierCosPenn =
                activation * activeForceLengthMult * cosPenn;
        const SimTK::Real passiveForceMult =
                calcPassiveForceMultiplier(normFiberLength);
        const SimTK::Real fiberLength =
                get_optimal_fiber_length() * normFiberLength;
        const SimTK::Real normTendonLength = calcNormalizedTendonLength(
                muscleTendonLength, fiberLength);
        const SimTK::Real normTendonForce =
                calcTendonForceMultiplier(normTendonLength);
        const SimTK::Real constant =
                passiveForceMult * cosPenn - normTendonForce;

        auto calcResidual = [this,
                &activationActiveForceLengthMultiplierCosPenn,
                &constant](
                const SimTK::Real& normFiberVelocity) {
            return activationActiveForceLengthMultiplierCosPenn
                    * calcForceVelocityMultiplier(normFiberVelocity)
                    + get_fiber_damping() * normFiberVelocity
                    + constant;
        };
        /*
        std::cout << "DEBUG excitation " << excitation << std::endl;
         */
        // In explicit dynamics mode and during trial integration steps,
        // the equilibrium solution for normFiberVelocity is not within
        // [-1, 1].
        const double velocityBound = 200000;

        // TODO bounds of -1 and 1?
        SimTK::Real equilNormFiberVelocity;
        try {
            equilNormFiberVelocity =
                    solveBisection(calcResidual, -velocityBound, velocityBound);
        } catch (const Exception& e) {
            std::cout << format("DEBUG computeStateVariableDerivatives"
                            "\n\ttime: %g"
                            "\n\tactivation: %g"
                            "\n\tmuscleTendonLength: %g"
                            "\n\tnormFiberLength: %g"
                            "\n\tactiveForceLengthMult: %g"
                            "\n\tpassiveForceMult: %g"
                            "\n\tnormTendonLength: %g"
                            "\n\tnormTendonForce: %g"
                            "\n\tscale: %g"
                            "\n\toffset: %g"
                    ,
                    s.getTime(),
                    activation, muscleTendonLength, normFiberLength,
                    activeForceLengthMult, passiveForceMult,
                    normTendonLength, normTendonForce,
                    activationActiveForceLengthMultiplierCosPenn,
                    constant)
                    << std::endl;
            throw;
        }

        /*
        std::cout << format("DEBUG derivatives t: %f normFiberVelocity: %f",
                s.getTime(), equilNormFiberVelocity) << std::endl;
                */

        // norm_fiber_length/second = norm_fiber_length/second * unitless
        const SimTK::Real normFiberLengthDot =
                get_max_contraction_velocity() * equilNormFiberVelocity;
        setStateVariableDerivativeValue(s, NORM_FIBER_LENGTH,
                normFiberLengthDot);
    }
}

double DeGrooteFregly2016Muscle::computeActuation(const SimTK::State& s) const {
    // TODO use fiber or tendon force?
    const SimTK::Real& activation = getActivation(s);
    const SimTK::Real normFiberLength = calcNormalizedFiberLength(s);
    const SimTK::Real normFiberVelocity = calcNormalizedFiberVelocity(s);

    const SimTK::Real normFiberForce = calcNormFiberForceAlongTendon(
            activation, normFiberLength, normFiberVelocity);

    return get_max_isometric_force() * normFiberForce;
}

void DeGrooteFregly2016Muscle::
computeInitialFiberEquilibrium(SimTK::State& s) const {

    if (get_ignore_tendon_compliance()) return;

    const SimTK::Real activation = getActivation(s);
    const SimTK::Real muscleTendonLength = getLength(s);
    const SimTK::Real normFiberVelocity = 0.0;

    auto calcResidual = [this, &activation,
            &muscleTendonLength,
            &normFiberVelocity](const SimTK::Real& normFiberLength) {
        return calcFiberEquilibriumResidual(activation,
                muscleTendonLength, normFiberLength, normFiberVelocity);
    };

    const SimTK::Real equilNormFiberLength =
            solveBisection(calcResidual, m_minNormFiberLength,
                    m_maxNormFiberLength);

    // TODO create setNormFiberLength().
    setStateVariableValue(s, "norm_fiber_length", equilNormFiberLength);
}

SimTK::Real DeGrooteFregly2016Muscle::solveBisection(
        std::function<SimTK::Real(const SimTK::Real&)> calcResidual,
        SimTK::Real left, SimTK::Real right,
        const SimTK::Real& xTolerance,
        const SimTK::Real& yTolerance,
        int maxIterations) const {
    SimTK::Real midpoint = left;

    OPENSIM_THROW_IF_FRMOBJ(maxIterations < 0, Exception,
            format("Expected maxIterations to be positive, but got %i.",
                    maxIterations));
    if (calcResidual(left) * calcResidual(right) >= 0) {
        std::cout << "DEBUG solveBisection() SAME SIGN" << std::endl;
        const auto x = createVectorLinspace(1000, left, right);
        TimeSeriesTable table;
        table.setColumnLabels({"residual"});
        SimTK::RowVector row(1);
        for (int i = 0; i < x.nrow(); ++i) {
            row[0] = calcResidual(x[i]);
            table.appendRow(x[i], row);
        }
        STOFileAdapter::write(table, "DEBUG_solveBisection_residual.sto");
    }
    OPENSIM_THROW_IF_FRMOBJ(calcResidual(left) * calcResidual(right) >= 0,
            Exception, format(
            "Function has same sign at bounds of %f and %f.", left, right));

    // TODO use error in residual as tolerance?
    SimTK::Real residualMidpoint;
    SimTK::Real residualLeft = calcResidual(left);
    int iterCount = 0;
    while (iterCount < maxIterations && (right - left) >= xTolerance) {
        midpoint = 0.5 * (left + right);
        residualMidpoint = calcResidual(midpoint);
        if (std::abs(residualMidpoint) < yTolerance) {
            break;
        } else if (residualMidpoint * residualLeft < 0) {
            // The solution is to the left of the current midpoint.
            right = midpoint;
        } else {
            left = midpoint;
            residualLeft = calcResidual(left);
        }
        ++iterCount;
    }
    if (iterCount == maxIterations)
        printMessage("Warning: bisection reached max iterations "
                        "at x = %g (%s %s).\n", midpoint,
                getConcreteClassName(), getName());
    /*
    std::cout << "DEBUG solveBisection iterCount "
            << iterCount << std::endl;
            */
    return midpoint;
}

SimTK::Real DeGrooteFregly2016Muscle::calcFiberEquilibriumResidual(
        const SimTK::Real& activation,
        const SimTK::Real& muscleTendonLength,
        const SimTK::Real& normFiberLength,
        const SimTK::Real& normFiberVelocity) const {

    const SimTK::Real normFiberForce = calcNormFiberForceAlongTendon(
            activation, normFiberLength, normFiberVelocity);

    const SimTK::Real fiberLength =
            get_optimal_fiber_length() * normFiberLength;
    const SimTK::Real normTendonLength = calcNormalizedTendonLength(
            muscleTendonLength, fiberLength);
    const SimTK::Real normTendonForce =
            calcTendonForceMultiplier(normTendonLength);

    /*
    std::cout << format("DEBUG equil"
                    "\n\tactivation: %f"
                    "\n\tmuscleTendonLength: %f"
                    "\n\tnormFiberLength: %f"
                    "\n\tnormFiberVelocity: %f"
                    "\n\tnormFiberForce: %f"
                    "\n\tnormTendonLength: %f"
                    "\n\tnormTendonForce: %f",
            activation, muscleTendonLength, normFiberLength,
            normFiberVelocity, normFiberForce, normTendonLength,
            normTendonForce)
            << std::endl;
            */
    return normFiberForce - normTendonForce;
}
