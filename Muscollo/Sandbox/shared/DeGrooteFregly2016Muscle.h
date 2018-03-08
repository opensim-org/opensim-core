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

#include <Muscollo/MuscolloUtilities.h>

#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>

namespace OpenSim {

/// TODO handle the singularity with activation = 0. Add damping? how to handle
/// this during time stepping?
/// TODO avoid checking ignore_tendon_compliance() in each function; might be
/// slow.
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
/// @subsection Departures from the Muscle base class
///
/// The documentation for Muscle::MuscleLengthInfo states that the
/// optimalFiberLength of a muscle is also its resting length, but this is not
/// true for this muscle: there is a non-zero passive fiber force at the
/// optimal fiber length.
///
/// In the Muscle class, setIgnoreTendonCompliance() and
/// setIngoreActivationDynamics() control modeling options, meaning these settings
/// could theoretically be changed. However, for this class, the modeling option
/// is ignored and the values of the ignore_tendon_compliance and
/// ignore_activation_dynamics properties are used directly.
///
/// Groote, F., Kinney, A. L., Rao, A. V., & Fregly, B. J. (2016). Evaluation of
/// Direct Collocation Optimal Control Problem Formulations for Solving the
/// Muscle Redundancy Problem. Annals of Biomedical Engineering, 44(10), 1–15.
/// http://doi.org/10.1007/s10439-016-1591-9
class /*OSIMMUSCOLLO_API*/DeGrooteFregly2016Muscle : public Muscle {
    OpenSim_DECLARE_CONCRETE_OBJECT(DeGrooteFregly2016Muscle, Muscle);
public:

    OpenSim_DECLARE_PROPERTY(default_norm_fiber_length, double,
    "Assumed initial normalized fiber length if none is assigned.");
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
    "Smaller value means activation can change more rapidly (units: seconds).");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
    "Smaller value means activation can decrease more rapidly "
    "(units: seconds).");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
    "Value of activation in the default state returned by initSystem().");

    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
    "The linear damping of the fiber (default: 0.01).");
    OpenSim_DECLARE_PROPERTY(tendon_strain_at_one_norm_force, double,
    "Tendon strain at a tension of 1 normalized force.");

    DeGrooteFregly2016Muscle() {
        constructProperties();
    }

protected:

    //--------------------------------------------------------------------------
    // COMPONENT INTERFACE
    //--------------------------------------------------------------------------
    /// @name Component interface
    /// @{
    void extendFinalizeFromProperties() override;

    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

    void extendInitStateFromProperties(SimTK::State& s) const override;

    void extendSetPropertiesFromState(const SimTK::State& s) override;

    void computeStateVariableDerivatives(const SimTK::State& s) const override;
    /// @}

    //--------------------------------------------------------------------------
    // ACTUATOR INTERFACE
    //--------------------------------------------------------------------------
    /// @name Actuator interface
    /// @{
    double computeActuation(const SimTK::State& s) const override;
    /// @}

public:

    //--------------------------------------------------------------------------
    // MUSCLE INTERFACE
    //--------------------------------------------------------------------------
    /// @name Muscle interface
    /// @{
    double getActivation(const SimTK::State& s) const override {
        // We override the Muscle's implementation because Muscle requires
        // realizing to Dynamics to access activation from MuscleDynamicsInfo,
        // which is unnecessary if the activation is a state.
        // TODO handle ignore_activation_dynamics
        return getStateVariableValue(s, ACTIVATION);
    }

    void setActivation(SimTK::State& s, double activation) const override {
        setStateVariableValue(s, ACTIVATION, activation);
    }
protected:
    // virtual double calcInextensibleTendonActiveFiberForce(SimTK::State& s,
    //         double aActivation) const;
    // virtual void calcMuscleLengthInfo(const SimTK::State& s,
    //         MuscleLengthInfo& mli) const;
    // virtual void calcFiberVelocityInfo(const SimTK::State& s,
    //         FiberVelocityInfo& fvi) const;
    // virtual void  calcMuscleDynamicsInfo(const SimTK::State& s,
    //         MuscleDynamicsInfo& mdi) const;
    // virtual void calcMusclePotentialEnergyInfo(const SimTK::State& s,
    //         MusclePotentialEnergyInfo& mpei) const;

public:
    /// Fiber velocity is assumed to be 0.
    void computeInitialFiberEquilibrium(SimTK::State& s) const override;
    /// @}

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
            int maxIterations = 100) const;

    SimTK::Real calcFiberEquilibriumResidual(
            const SimTK::Real& activation,
            const SimTK::Real& muscleTendonLength,
            const SimTK::Real& normFiberLength,
            const SimTK::Real& normFiberVelocity) const;

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
    // TODO: In explicit mode, do not allow negative tendon forces?
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
        constructProperty_default_norm_fiber_length(1.0);
        constructProperty_activation_time_constant(0.015);
        constructProperty_deactivation_time_constant(0.060);
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

} // namespace OpenSim

#endif // MUSCOLLO_DEGROOTEFREGLY2016MUSCLE_H
