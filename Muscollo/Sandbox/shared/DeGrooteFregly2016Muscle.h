#ifndef MUSCOLLO_DEGROOTEFREGLY2016MUSCLE_H
#define MUSCOLLO_DEGROOTEFREGLY2016MUSCLE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: DeGrooteFregly2016Muscle.h                               *
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

#include <OpenSim/Simulation/Model/PathActuator.h>

namespace OpenSim {

/// TODO handle the singularity with activation = 0. Add damping? how to handle
/// this during time stepping?
/// TODO prohibit fiber length from going below 0.2.
/// This muscle model was published in De Groote et al. 2016.
/// The parameters of the active force-length and force-velocity curves have
/// been slightly modified from what was published to ensure the curves go
/// through key points:
///   - Active force-length curve goes through (1, 1).
///   - Force-velocity curve goes through (-1, 0) and (0, 1).
/// The default tendon force curve parameters are modified from that in De
/// Groote et al., 2016: the curve is parameterized by the strain at 1 norm
/// force (rather than "kT"), and the default value for this parameter is
/// 0.049 (same as in TendonForceLengthCurve) rather than 0.0474.
///
/// The fiber damping helps with numerically solving for fiber velocity at low
/// activations or with low force-length multipliers, and is likely to be more
/// useful with explicit fiber dynamics than implicit fiber dynamics.
///
/// Groote, F., Kinney, A. L., Rao, A. V., & Fregly, B. J. (2016). Evaluation of
/// Direct Collocation Optimal Control Problem Formulations for Solving the
/// Muscle Redundancy Problem. Annals of Biomedical Engineering, 44(10), 1–15.
/// http://doi.org/10.1007/s10439-016-1591-9
class /*OSIMMUSCOLLO_API*/DeGrooteFregly2016Muscle : public PathActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(DeGrooteFregly2016Muscle, PathActuator);
public:
    OpenSim_DECLARE_PROPERTY(max_isometric_force, double,
    "Maximum isometric force that the fibers can generate.");
    OpenSim_DECLARE_PROPERTY(optimal_fiber_length, double,
    "Optimal length of the muscle fibers.");
    OpenSim_DECLARE_PROPERTY(tendon_slack_length, double,
    "Resting length of the tendon.");
    OpenSim_DECLARE_PROPERTY(pennation_angle_at_optimal, double,
    "Angle between tendon and fibers at optimal fiber length, in radians.");
    OpenSim_DECLARE_PROPERTY(max_contraction_velocity, double,
    "Maximum contraction velocity of the fibers, in "
    "optimal_fiber_length/second.");
    // TODO avoid checking this everywhere.
    OpenSim_DECLARE_PROPERTY(ignore_tendon_compliance, bool,
    "Compute muscle dynamics ignoring tendon compliance. "
    "Tendon is assumed to be rigid.");
    OpenSim_DECLARE_PROPERTY(default_norm_fiber_length, double,
    "Assumed initial normalized fiber length if none is assigned.");
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
    "Smaller value means activation can change more rapidly (units: seconds).");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
    "Value of activation in the default state returned by initSystem().");

    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
    "The linear damping of the fiber (default: 0.01).");
    OpenSim_DECLARE_PROPERTY(tendon_strain_at_one_norm_force, double,
    "Tendon strain at a tension of 1 normalized force.");

    DeGrooteFregly2016Muscle() {
        constructProperties();
    }

    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        OPENSIM_THROW_IF_FRMOBJ(
            !getProperty_optimal_force().getValueIsDefault(),
            Exception,
            "The optimal_force property is ignored for this Force; "
            "use max_isometric_force instead.");

        OPENSIM_THROW_IF_FRMOBJ(get_tendon_strain_at_one_norm_force() <= 0,
            Exception,
            "Expected the tendon_strain_at_one_norm_force property to be "
            "positive, but got " +
            std::to_string(get_tendon_strain_at_one_norm_force()) + ".");

        // TODO validate properties (nonnegative, etc.).

        m_maxContractionVelocityInMeters =
                get_max_contraction_velocity() * get_optimal_fiber_length();
        m_kT = log((1.0 + c3) / c1) /
                (1.0 + get_tendon_strain_at_one_norm_force() - c2);
    }

    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        Super::extendAddToSystem(system);
        addStateVariable(ACTIVATION, SimTK::Stage::Dynamics);
        if (!get_ignore_tendon_compliance()) {
            addStateVariable(NORM_FIBER_LENGTH, SimTK::Stage::Position);
        }
    }

    void extendInitStateFromProperties(SimTK::State& s) const override {
        Super::extendInitStateFromProperties(s);
        setStateVariableValue(s, ACTIVATION, get_default_activation());
        if (!get_ignore_tendon_compliance()) {
            setStateVariableValue(s, NORM_FIBER_LENGTH,
                    get_default_norm_fiber_length());
        }
    }

    void extendSetPropertiesFromState(const SimTK::State& s) override {
        Super::extendSetPropertiesFromState(s);
        // TODO getActivation(s)?
        set_default_activation(getStateVariableValue(s, ACTIVATION));
        if (!get_ignore_tendon_compliance()) {
            set_default_norm_fiber_length(
                    getStateVariableValue(s, NORM_FIBER_LENGTH));
        }
    }

    // TODO no need to do clamping, etc; CoordinateActuator is bidirectional.
    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        const auto& tau = get_activation_time_constant();
        const auto& excitation = getControl(s);
        const auto& activation = getActivation(s);
        const SimTK::Real activationDot = (excitation - activation) / tau;
        setStateVariableDerivativeValue(s, ACTIVATION, activationDot);

        if (!get_ignore_tendon_compliance()) {
            // TODO if not ignoring fiber dynamics.
            // TODO explain these equations.

            /*
            std::cout << format("DEBUG %f %f", s.getTime(), getLength(s))
                    << std::endl;

            // TODO should use getNormalizedFiberLength():
            const SimTK::Real normFiberLength =
                    calcNormalizedFiberLength(s);

            const SimTK::Real passiveForceMult =
                    calcPassiveForceMultiplier(normFiberLength);
            const SimTK::Real activeForceMult =
                    calcActiveForceLengthMultiplier(normFiberLength);

            const SimTK::Real normTendonLength = calcNormalizedTendonLength(s);
            const SimTK::Real normTendonForce =
                    calcTendonForceMultiplier(normTendonLength);


            // TODO handle pennation.
            const SimTK::Real normFiberForce = normTendonForce;
            const SimTK::Real fiberVelocityMult =
                    (normFiberForce - passiveForceMult) /
                            (activation * activeForceMult);

            // TODO maximum fiber velocity should be max_contraction_velocity,
            // but then fiber equilibrium is violated.

            // normFiberVelocity is unitless but we need units of 1/second.
            const SimTK::Real normFiberVelocity =
                    calcForceVelocityInverseCurve(fiberVelocityMult);
            // norm_fiber_length/second = norm_fiber_length/second * unitless
            const SimTK::Real normFiberLengthDot =
                   get_max_contraction_velocity() * normFiberVelocity;
            if (std::abs(normFiberVelocity) > 1.0) {
                std::cout << format("DEBUG normFiberVelocity out of range "
                                "t: %f"
                                "\n\tnormFiberVelocity: %f"
                                "\n\tmuscleTendonLength: %f"
                                "\n\tnormFiberLength: %f"
                                "\n\tnormTendonLength: %f"
                                "\n\tnormFiberForce: %f"
                                "\n\tpassiveForceMult: %f"
                                "\n\tactivation: %f"
                                "\n\tactiveForceMult: %f"
                                "\n\tfiberVelocityMult: %f",
                        s.getTime(),
                        normFiberVelocity,
                        getLength(s),
                        normFiberLength, normTendonLength,
                        normFiberForce, passiveForceMult,
                        activation, activeForceMult, fiberVelocityMult)
                        << std::endl;
            }
             */

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
            /*
            const auto x = createVectorLinspace(1000, -TODO, TODO);
            TimeSeriesTable table;
            table.setColumnLabels({"residual"});
            SimTK::RowVector row(1);
            for (int i = 0; i < x.nrow(); ++i) {
                row[0] = calcResidual(x[i]);
                table.appendRow(x[i], row);
            }
            STOFileAdapter::write(table, "DEBUG_equilibrium_residual.sto");
            */

            /*
            std::cout << format("DEBUG computeStateVariableDerivatives"
                            "\n\tactivation: %f"
                            "\n\tmuscleTendonLength: %f"
                            "\n\tnormFiberLength: %f"
                            "\n\tactiveForceLengthMult: %f"
                            "\n\tnormTendonLength: %f"
                            "\n\tnormTendonForce: %f",
                    activation, muscleTendonLength, normFiberLength,
                    activeForceLengthMult, normTendonLength, normTendonForce)
                    << std::endl;
            */
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

    double computeActuation(const SimTK::State& s) const override {
        // TODO use fiber or tendon force?
        const SimTK::Real& activation = getActivation(s);
        const SimTK::Real normFiberLength = calcNormalizedFiberLength(s);
        const SimTK::Real normFiberVelocity = calcNormalizedFiberVelocity(s);

        const SimTK::Real normFiberForce = calcNormFiberForceAlongTendon(
                activation, normFiberLength, normFiberVelocity);

        return get_max_isometric_force() * normFiberForce;
    }

    void computeInitialFiberEquilibrium(SimTK::State& s) const /*TODO override */ {

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

    /// Solve for the root of the provided function calcResidual using
    /// bisection, starting with bounds left and right. Iteration stops when
    /// maxIterations is reached, the width of the search interal is less than
    /// the xTolerance, or the value of the function is less than the
    /// yTolerance.
    SimTK::Real solveBisection(
            std::function<SimTK::Real(const SimTK::Real&)> calcResidual,
            SimTK::Real left, SimTK::Real right,
            const SimTK::Real& xTolerance = 1e-7,
            const SimTK::Real& yTolerance = 1e-7,
            int maxIterations = 100) const {
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

    SimTK::Real calcFiberEquilibriumResidual(
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

    /// TODO no pennation yet.
    SimTK::Real calcNormFiberForceAlongTendon(const SimTK::Real& activation,
            const SimTK::Real& normFiberLength,
            const SimTK::Real& normFiberVelocity) const {
        // TODO consider pennation.
        return calcNormFiberForce(activation, normFiberLength,
                normFiberVelocity);
    }

    SimTK::Real calcNormFiberForce(const SimTK::Real& activation,
            const SimTK::Real& normFiberLength,
            const SimTK::Real& normFiberVelocity) const {

        const SimTK::Real activeForceLengthMult =
                calcActiveForceLengthMultiplier(normFiberLength);
        const SimTK::Real forceVelocityMult =
                calcForceVelocityMultiplier(normFiberVelocity);

        const SimTK::Real passiveForceMult =
                calcPassiveForceMultiplier(normFiberLength);

        const SimTK::Real normActiveForce =
                activation * activeForceLengthMult * forceVelocityMult;
        // std::cout << "DEBUG " << s.getTime() << " "
        //         << normFiberLength << " "
        //         << normActiveForce << " "
        //         << passiveForceMult
        //         << std::endl;
        return normActiveForce + passiveForceMult +
                + get_fiber_damping() * normFiberVelocity;

    }

    SimTK::Real getActivation(const SimTK::State& s) const {
        return getStateVariableValue(s, ACTIVATION);
    }

    // TODO replace with getNormalizedFiberLength from Muscle.
    // TODO these next few methods are convoluted; using MuscleLengthInfo will
    // be better!
    SimTK::Real calcNormalizedFiberLength(const SimTK::State& s) const {
        // TODO get from state variable.
        if (get_ignore_tendon_compliance()) {
            return calcFiberLength(s) / get_optimal_fiber_length();
        } else {
            return getStateVariableValue(s, NORM_FIBER_LENGTH);
        }
    }

    SimTK::Real calcFiberLength(const SimTK::State& s) const {
        if (get_ignore_tendon_compliance()) {
            return getLength(s) - get_tendon_slack_length();
        } else {
            return get_optimal_fiber_length() * calcNormalizedFiberLength(s);
        }
    }

    SimTK::Real calcNormalizedTendonLength(const SimTK::State& s) const {
        if (get_ignore_tendon_compliance())
            return 1.0;
        else
            return calcNormalizedTendonLength(getLength(s), calcFiberLength(s));
    }
    SimTK::Real calcNormalizedTendonLength(
            const SimTK::Real& muscleTendonLength,
            const SimTK::Real& fiberLength) const {
        if (get_ignore_tendon_compliance()) {
            // TODO handle muscle-tendon length is less than tendon slack length
            // (buckling).
            return 1.0;
        } else {
            return (muscleTendonLength - fiberLength) /
                    get_tendon_slack_length();
        }
    }

    /// This returns the fiber velocity normalized by the max contraction
    /// velocity. The normalized fiber velocity is unitless and should be in
    /// [-1, 1].
    // TODO replace with getNormalizedFiberVelocity from Muscle.
    SimTK::Real calcNormalizedFiberVelocity(const SimTK::State& s) const {
        const SimTK::Real& fiberVelocity = getLengtheningSpeed(s);
        return fiberVelocity / m_maxContractionVelocityInMeters;
    }

    /// @name Calculate multipliers.
    /// @{
    static SimTK::Real calcActiveForceLengthMultiplier(
            const SimTK::Real& normFiberLength) {
        // Values are taken from https://simtk.org/projects/optcntrlmuscle
        // rather than the paper supplement. b11 was modified to ensure that
        // f(1) = 1.
        static const double b11 =  0.8150671134243542;
        static const double b21 =  1.055033428970575;
        static const double b31 =  0.162384573599574;
        static const double b41 =  0.063303448465465;
        static const double b12 =  0.433004984392647;
        static const double b22 =  0.716775413397760;
        static const double b32 = -0.029947116970696;
        static const double b42 =  0.200356847296188;
        static const double b13 =  0.1;
        static const double b23 =  1.0;
        static const double b33 =  0.5 * sqrt(0.5);
        static const double b43 =  0.0;
        return calcGaussian(normFiberLength, b11, b21, b31, b41) +
                calcGaussian(normFiberLength, b12, b22, b32, b42) +
                calcGaussian(normFiberLength, b13, b23, b33, b43);
    }
    /// Domain: [-1, 1]
    /// Range: [0, 1.789]
    static SimTK::Real calcForceVelocityMultiplier(
            const SimTK::Real& normFiberVelocity) {
        using SimTK::square;
        const SimTK::Real tempV = d2 * normFiberVelocity + d3;
        const SimTK::Real tempLogArg = tempV + sqrt(square(tempV) + 1.0);
        return d1 * log(tempLogArg) + d4;
    }

    /// This is the inverse of the force-velocity multiplier function, and
    /// returns the normalized fiber velocity (in [-1, 1]) as a function of
    /// the force-velocity multiplier.
    static SimTK::Real calcForceVelocityInverseCurve(
            const SimTK::Real& forceVelocityMult) {
        // The version of this equation in the supplementary materials of De
        // Groote et al., 2016 has an error (it's missing a "-d3" before
        // dividing by "d2").
        return (sinh(1.0 / d1 * (forceVelocityMult - d4)) - d3) / d2;
    }

    /// This is the passive force-length curve. The curve becomes negative below
    /// the minNormFiberLength.
    SimTK::Real calcPassiveForceMultiplier(
            const SimTK::Real& normFiberLength) const {
        // Passive force-length curve.
        // TODO turn some of these into properties:
        static const double kPE = 4.0;
        static const double e0  = 0.6;
        static const double denom = exp(kPE) - 1;
        static const double numer_offset =
                exp(kPE * (m_minNormFiberLength - 1)/e0);
        // The version of this equation in the supplementary materials of De
        // Groote et al., 2016 has an error. The correct equation passes
        // through y = 0 at x = 0.2, and therefore is never negative within
        // the allowed range of the optimal fiber length. The version in the
        // supplementary materials allows for negative forces.
        return (exp(kPE * (normFiberLength - 1.0) / e0) - numer_offset) / denom;
    }

    /// The normalized tendon force as a function of normalized tendon length.
    /// Note that this curve does not go through (1, 0); when normTendonLength=1,
    /// this function returns a slightly negative number.
    SimTK::Real calcTendonForceMultiplier(
            const SimTK::Real& normTendonLength) const {
        return c1 * exp(m_kT * (normTendonLength - c2)) - c3;
    }

    /// Export the active force-length multiplier and passive force multiplier
    /// curves to a DataTable. If the normFiberLengths argument is omitted, we
    /// use createVectorLinspace(200, minNormFiberLength, maxNormFiberLength).
    DataTable exportFiberLengthCurvesToTable(
            const SimTK::Vector& normFiberLengths = SimTK::Vector()) const {
        SimTK::Vector def;
        const SimTK::Vector* x = nullptr;
        if (normFiberLengths.nrow()) {
            x = &normFiberLengths;
        } else {
            def = createVectorLinspace(200, m_minNormFiberLength,
                    m_maxNormFiberLength);
            x = &def;
        }

        DataTable table;
        table.setColumnLabels(
                {"active_force_length_multiplier", "passive_force_multiplier"});
        SimTK::RowVector row(2);
        for (int irow = 0; irow < x->nrow(); ++irow) {
            const auto& normFiberLength = x->get(irow);
            row[0] = calcActiveForceLengthMultiplier(normFiberLength);
            row[1] = calcPassiveForceMultiplier(normFiberLength);
            table.appendRow(normFiberLength, row);
        }
        return table;
    }
    /// Export the fiber force-velocity multiplier curve to a DataTable. If
    /// the normFiberVelocities argument is omitted, we use
    /// createVectorLinspace(200, -1.1, 1.1).
    DataTable exportFiberVelocityMultiplierToTable(
            const SimTK::Vector& normFiberVelocities = SimTK::Vector()) const {
        SimTK::Vector def;
        const SimTK::Vector* x = nullptr;
        if (normFiberVelocities.nrow()) {
            x = &normFiberVelocities;
        } else {
            def = createVectorLinspace(200, -1.1, 1.1);
            x = &def;
        }

        DataTable table;
        table.setColumnLabels({"force_velocity_multiplier"});
        SimTK::RowVector row(1);
        for (int irow = 0; irow < x->nrow(); ++irow) {
            const auto& normFiberVelocity = x->get(irow);
            row[0] = calcForceVelocityMultiplier(normFiberVelocity);
            table.appendRow(normFiberVelocity, row);
        }
        return table;
    }
    /// Export the fiber tendon force multiplier curve to a DataTable. If
    /// the normFiberVelocities argument is omitted, we use
    /// createVectorLinspace(200, 0.95, <1 + strain at 1 norm force>)
    DataTable exportTendonForceMultiplierToTable(
            const SimTK::Vector& normTendonLengths = SimTK::Vector()) const {
        SimTK::Vector def;
        const SimTK::Vector* x = nullptr;
        if (normTendonLengths.nrow()) {
            x = &normTendonLengths;
        } else {
            // Evaluate the inverse of the tendon curve at y = 1.
            def = createVectorLinspace(200, 0.95,
                    1.0 + get_tendon_strain_at_one_norm_force());
            x = &def;
        }

        DataTable table;
        table.setColumnLabels({"tendon_force_multiplier"});
        SimTK::RowVector row(1);
        for (int irow = 0; irow < x->nrow(); ++irow) {
            const auto& normTendonLength = x->get(irow);
            row[0] = calcTendonForceMultiplier(normTendonLength);
            table.appendRow(normTendonLength, row);
        }
        return table;
    }
    /// Print the muscle curves to STO files. The files will be named as
    /// `<muscle-name>_<curve_type>.sto`.
    ///
    /// @param directory
    ///     The directory to which the data files should be written. Do NOT
    ///     include the filename. By default, the files are printed to the
    ///     current working directory.
    void printCurvesToSTOFiles(const std::string& directory = ".") const {
        std::string prefix = directory + SimTK::Pathname::getPathSeparator() +
                getName();

        STOFileAdapter::write(exportFiberLengthCurvesToTable(),
                prefix + "_fiber_length_curves.sto");

        STOFileAdapter::write(exportFiberVelocityMultiplierToTable(),
                prefix + "_fiber_velocity_multiplier.sto");

        STOFileAdapter::write(exportTendonForceMultiplierToTable(),
                prefix + "_tendon_force_multiplier.sto");
    }
    /// @}
private:
    void constructProperties() {
        constructProperty_max_isometric_force(1000);
        constructProperty_optimal_fiber_length(0.1);
        constructProperty_tendon_slack_length(0.2);
        constructProperty_max_contraction_velocity(10);
        constructProperty_ignore_tendon_compliance(false);
        constructProperty_default_norm_fiber_length(1.0);
        constructProperty_activation_time_constant(0.010);
        constructProperty_default_activation(0.5);

        constructProperty_tendon_strain_at_one_norm_force(0.049);
        constructProperty_fiber_damping(0.01);
    }
    /// This is a Gaussian-like function used in the active force-length curve.
    /// A proper Gaussian function does not have the variable in the denominator
    /// of the exponent.
    /// The supplement for De Groote et al., 2016 has a typo:
    /// the denominator should be squared.
    static SimTK::Real calcGaussian(const SimTK::Real& x, const double& b1,
            const double& b2, const double& b3, const double& b4) {
        using SimTK::square;
        return b1 * exp(-0.5 * square(x - b2) / square(b3 + b4 * x));
    }

    // Curve parameters.
    // Notation comes from De Groote et al, 2016 (supplement).

    // Parameters for the tendon force curve.
    // TODO allow users to set kT; it has a huge effect on the runtime of
    // INDYGO.
    constexpr static double c1 = 0.200;
    constexpr static double c2 = 0.995; // TODO 1.0?
    constexpr static double c3 = 0.250; // TODO 0.2 (c3)?

    // Parameters for the force-velocity curve.
    // The parameters from the paper supplement are rounded/truncated and cause
    // the curve to not go through the points (-1, 0) and (0, 1). We solved for
    // different values of d1 and d4 so that the curve goes through (-1, 0) and
    // (0, 1).
    // The values from the code at https://simtk.org/projects/optcntrlmuscle
    // also do not go through (-1, 0) and (0, 1).
    constexpr static double d1 = -0.3211346127989808;
    constexpr static double d2 = -8.149;
    constexpr static double d3 = -0.374;
    constexpr static double d4 =  0.8825327733249912;

    constexpr static double m_minNormFiberLength = 0.2;
    constexpr static double m_maxNormFiberLength = 1.8;

    static const std::string ACTIVATION;
    static const std::string NORM_FIBER_LENGTH;

    SimTK::Real m_maxContractionVelocityInMeters = SimTK::NaN;
    // Tendon stiffness parameter from De Groote et al., 2016.
    SimTK::Real m_kT = SimTK::NaN;
};

// TODO these should not be in a header.
const std::string DeGrooteFregly2016Muscle::ACTIVATION("activation");
const std::string DeGrooteFregly2016Muscle::NORM_FIBER_LENGTH("norm_fiber_length");

} // namespace OpenSim

#endif // MUSCOLLO_DEGROOTEFREGLY2016MUSCLE_H
