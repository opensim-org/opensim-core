#ifndef MOCO_DEGROOTEFREGLY2016MUSCLE_H
#define MOCO_DEGROOTEFREGLY2016MUSCLE_H
/* -------------------------------------------------------------------------- *
 * OpenSim Moco: DeGrooteFregly2016Muscle.h                                   *
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

#include "../MocoUtilities.h"
#include "../osimMocoDLL.h"

#include <OpenSim/Common/DataTable.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/Muscle.h>

namespace OpenSim {

// TODO avoid checking ignore_tendon_compliance() in each function; might be
// slow.
// TODO prohibit fiber length from going below 0.2.

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
/// useful with explicit fiber dynamics than implicit fiber dynamics (when
/// support for fiber dynamics is added).
///
/// @note This class does not yet support tendon compliance (fiber dynamics).
///
/// @underdevelopment
///
/// @subsection Departures from the Muscle base class
///
/// The documentation for Muscle::MuscleLengthInfo states that the
/// optimalFiberLength of a muscle is also its resting length, but this is not
/// true for this muscle: there is a non-zero passive fiber force at the
/// optimal fiber length.
///
/// In the Muscle class, setIgnoreTendonCompliance() and
/// setIngoreActivationDynamics() control modeling options, meaning these
/// settings could theoretically be changed. However, for this class, the
/// modeling option is ignored and the values of the ignore_tendon_compliance
/// and ignore_activation_dynamics properties are used directly (though, as
/// mentioned earlier, tendon compliance is not yet supported).
///
/// Groote, F., Kinney, A. L., Rao, A. V., & Fregly, B. J. (2016). Evaluation of
/// Direct Collocation Optimal Control Problem Formulations for Solving the
/// Muscle Redundancy Problem. Annals of Biomedical Engineering, 44(10), 1–15.
/// http://doi.org/10.1007/s10439-016-1591-9
class OSIMMOCO_API DeGrooteFregly2016Muscle : public Muscle {
    OpenSim_DECLARE_CONCRETE_OBJECT(DeGrooteFregly2016Muscle, Muscle);

public:
    OpenSim_DECLARE_PROPERTY(default_normalized_fiber_length, double,
            "Assumed initial normalized fiber length if none is assigned.");
    OpenSim_DECLARE_PROPERTY(activation_time_constant, double,
            "Smaller value means activation can change more rapidly (units: "
            "seconds).");
    OpenSim_DECLARE_PROPERTY(deactivation_time_constant, double,
            "Smaller value means activation can decrease more rapidly "
            "(units: seconds).");
    OpenSim_DECLARE_PROPERTY(default_activation, double,
            "Value of activation in the default state returned by "
            "initSystem().");

    OpenSim_DECLARE_PROPERTY(active_force_width_scale, double,
            "Scale factor for the width of the active force-length curve. "
            "Larger values make the curve wider. "
            "(default: 1.0).");
    OpenSim_DECLARE_PROPERTY(fiber_damping, double,
            "The linear damping of the fiber (default: 0.01).");
    OpenSim_DECLARE_PROPERTY(tendon_strain_at_one_norm_force, double,
            "Tendon strain at a tension of 1 normalized force.");

    OpenSim_DECLARE_PROPERTY(ignore_passive_fiber_force, bool,
            "Make the passive fiber force 0 (default: false).");

    DeGrooteFregly2016Muscle() { constructProperties(); }

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

    /// If ignore_activation_dynamics is true, this gets excitation instead.
    double getActivation(const SimTK::State& s) const override {
        // We override the Muscle's implementation because Muscle requires
        // realizing to Dynamics to access activation from MuscleDynamicsInfo,
        // which is unnecessary if the activation is a state.
        if (get_ignore_activation_dynamics()) {
            return getControl(s);
        } else {
            return getStateVariableValue(s, STATE_ACTIVATION_NAME);
        }
    }

    /// If ignore_activation_dynamics is true, this sets excitation instead.
    void setActivation(SimTK::State& s, double activation) const override {
        if (get_ignore_activation_dynamics()) {
            SimTK::Vector& controls(getModel().updControls(s));
            setControls(SimTK::Vector(1, activation), controls);
            getModel().setControls(s, controls);
        } else {
            setStateVariableValue(s, STATE_ACTIVATION_NAME, activation);
        }
    }

protected:
    double calcInextensibleTendonActiveFiberForce(
            SimTK::State&, double) const override {
        OPENSIM_THROW_FRMOBJ(Exception, "Not implemented.");
    }
    void calcMuscleLengthInfo(
            const SimTK::State& s, MuscleLengthInfo& mli) const override;
    void calcFiberVelocityInfo(
            const SimTK::State& s, FiberVelocityInfo& fvi) const override;
    /// This function does not yet compute stiffness or power.
    void calcMuscleDynamicsInfo(
            const SimTK::State& s, MuscleDynamicsInfo& mdi) const override;
    /// This function does not yet compute potential energy.
    void calcMusclePotentialEnergyInfo(const SimTK::State& s,
            MusclePotentialEnergyInfo& mpei) const override;

public:
    /// Fiber velocity is assumed to be 0.
    void computeInitialFiberEquilibrium(SimTK::State& s) const override;
    /// @}

    /// @name Calculate multipliers.
    /// These functions compute the values of normalized/dimensionless curves,
    /// and do not depend on a SimTK::State.
    /// @{

    /// The active force-length curve is the sum of 3 Gaussian-like curves. The
    /// width of the curve can be adjusted via the active_force_width_scale
    /// property.
    SimTK::Real calcActiveForceLengthMultiplier(
            const SimTK::Real& normFiberLength) const {
        // Values are taken from https://simtk.org/projects/optcntrlmuscle
        // rather than the paper supplement. b11 was modified to ensure that
        // f(1) = 1.
        static const double b11 = 0.8150671134243542;
        static const double b21 = 1.055033428970575;
        static const double b31 = 0.162384573599574;
        static const double b41 = 0.063303448465465;
        static const double b12 = 0.433004984392647;
        static const double b22 = 0.716775413397760;
        static const double b32 = -0.029947116970696;
        static const double b42 = 0.200356847296188;
        static const double b13 = 0.1;
        static const double b23 = 1.0;
        static const double b33 = 0.5 * sqrt(0.5);
        static const double b43 = 0.0;
        const double& scale = get_active_force_width_scale();
        // Shift the curve so its peak is at the origin, scale it
        // horizontally, then shift it back so its peak is still at x = 1.0.
        const double x = (normFiberLength - 1.0) / scale + 1.0;
        return calcGaussianLikeCurve(x, b11, b21, b31, b41) +
               calcGaussianLikeCurve(x, b12, b22, b32, b42) +
               calcGaussianLikeCurve(x, b13, b23, b33, b43);
    }

    /// The parameters of this curve are not modifiable, so this function is
    /// static.
    /// Domain: [-1, 1]
    /// Range: [0, 1.794]
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
        static const double e0 = 0.6;
        static const double denom = exp(kPE) - 1;
        static const double numer_offset =
                exp(kPE * (m_minNormFiberLength - 1) / e0);
        // The version of this equation in the supplementary materials of De
        // Groote et al., 2016 has an error. The correct equation passes
        // through y = 0 at x = 0.2, and therefore is never negative within
        // the allowed range of the fiber length. The version in the
        // supplementary materials allows for negative forces.

        if (get_ignore_passive_fiber_force()) return 0;
        return (exp(kPE * (normFiberLength - 1.0) / e0) - numer_offset) / denom;
    }

    /// The normalized tendon force as a function of normalized tendon length.
    /// Note that this curve does not go through (1, 0); when
    /// normTendonLength=1, this function returns a slightly negative number.
    // TODO: In explicit mode, do not allow negative tendon forces?
    SimTK::Real calcTendonForceMultiplier(
            const SimTK::Real& normTendonLength) const {
        return c1 * exp(m_kT * (normTendonLength - c2)) - c3;
    }

    /// @name Utilities
    /// @{

    /// Export the active force-length multiplier and passive force multiplier
    /// curves to a DataTable. If the normFiberLengths argument is omitted, we
    /// use createVectorLinspace(200, minNormFiberLength, maxNormFiberLength).
    DataTable exportFiberLengthCurvesToTable(
            const SimTK::Vector& normFiberLengths = SimTK::Vector()) const;
    /// Export the fiber force-velocity multiplier curve to a DataTable. If
    /// the normFiberVelocities argument is omitted, we use
    /// createVectorLinspace(200, -1.1, 1.1).
    DataTable exportFiberVelocityMultiplierToTable(
            const SimTK::Vector& normFiberVelocities = SimTK::Vector()) const;
    /// Export the fiber tendon force multiplier curve to a DataTable. If
    /// the normFiberVelocities argument is omitted, we use
    /// createVectorLinspace(200, 0.95, 1 + <strain at 1 norm force>)
    DataTable exportTendonForceMultiplierToTable(
            const SimTK::Vector& normTendonLengths = SimTK::Vector()) const;
    /// Print the muscle curves to STO files. The files will be named as
    /// `<muscle-name>_<curve_type>.sto`.
    ///
    /// @param directory
    ///     The directory to which the data files should be written. Do NOT
    ///     include the filename. By default, the files are printed to the
    ///     current working directory.
    void printCurvesToSTOFiles(const std::string& directory = ".") const;

    /// Replace muscles of other types in the model with muscles of this type.
    /// Currently, only Millard2012EquilibriumMuscles are replaced.
    /// If the model has muscles of other types, an exception is thrown unless
    /// allowUnsupportedMuscles is true.
    /// The resulting muscles will have ignore_tendon_compliance set as true,
    /// regardless of the values in the original muscles, as this muscle class
    /// does not yet support tendon compliance.
    static void replaceMuscles(
            Model& model, bool allowUnsupportedMuscles = false);

    /// @}

private:
    void constructProperties();

    /// This is a Gaussian-like function used in the active force-length curve.
    /// A proper Gaussian function does not have the variable in the denominator
    /// of the exponent.
    /// The supplement for De Groote et al., 2016 has a typo:
    /// the denominator should be squared.
    static SimTK::Real calcGaussianLikeCurve(const SimTK::Real& x,
            const double& b1, const double& b2, const double& b3,
            const double& b4) {
        using SimTK::square;
        return b1 * exp(-0.5 * square(x - b2) / square(b3 + b4 * x));
    }

    // Curve parameters.
    // Notation comes from De Groote et al., 2016 (supplement).

    // Parameters for the tendon force curve.
    // --------------------------------------
    constexpr static double c1 = 0.200;
    // Horizontal asymptote as x -> -inf is -c3.
    // Normalized force at 0 strain is c1 * exp(-c2) - c3.
    // This parameter is 0.995 in De Groote et al., which causes the y-value at
    // 0 strain to be negative. We use 1.0 so that the y-value at 0 strain is 0
    // (since c2 == c3).
    constexpr static double c2 = 1.0;
    // This parameter is 0.250 in De Groote et al., which causes
    // lim(x->-inf) = -0.25 instead of -0.20.
    constexpr static double c3 = 0.200;

    // Parameters for the force-velocity curve.
    // ----------------------------------------
    // The parameters from the paper supplement are rounded/truncated and cause
    // the curve to not go through the points (-1, 0) and (0, 1). We solved for
    // different values of d1 and d4 so that the curve goes through (-1, 0) and
    // (0, 1).
    // The values from the code at https://simtk.org/projects/optcntrlmuscle
    // also do not go through (-1, 0) and (0, 1).
    constexpr static double d1 = -0.3211346127989808;
    constexpr static double d2 = -8.149;
    constexpr static double d3 = -0.374;
    constexpr static double d4 = 0.8825327733249912;

    constexpr static double m_minNormFiberLength = 0.2;
    constexpr static double m_maxNormFiberLength = 1.8;

    static const std::string STATE_ACTIVATION_NAME;

    // Computed from properties.
    // -------------------------

    // The square of (fiber_width / optimal_fiber_length).
    SimTK::Real m_fiberWidth = SimTK::NaN;
    SimTK::Real m_squareFiberWidth = SimTK::NaN;
    SimTK::Real m_maxContractionVelocityInMetersPerSecond = SimTK::NaN;
    // Tendon stiffness parameter from De Groote et al., 2016. Instead of
    // kT, users specify tendon strain at 1 norm force, which is more intuitive.
    SimTK::Real m_kT = SimTK::NaN;
};

} // namespace OpenSim

#endif // MOCO_DEGROOTEFREGLY2016MUSCLE_H
