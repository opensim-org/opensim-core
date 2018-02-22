/* -------------------------------------------------------------------------- *
 * OpenSim Muscollo: sandboxMuscle.cpp                                        *
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

// Some of this code is based on testSingleMuscle,
// testSingleMuscleDeGrooteFregly2016.

#include <Muscollo/osimMuscollo.h>

#include <MuscolloSandboxShared.h>

#include <OpenSim/Simulation/Model/PathActuator.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Actuators/Millard2012EquilibriumMuscle.h>

using namespace OpenSim;

/// TODO handle the singularity with activation = 0. Add damping? how to handle
/// this during time stepping?
/// TODO prohibit fiber length from going below 0.2.
/// This muscle model was published in De Groote et al. 2016.
/// The parameters of the active force-length and force-velocity curves have
/// been slightly modified from what was published to ensure the curves go
/// through key points:
///   - Active force-length curve goes through (1, 1).
///   - Force-velocity curve goes through (-1, 0) and (0, 1).
/// DGF stands for DeGroote-Fregly. The abbreviation is temporary, and is used
/// to avoid a name conflict with the existing DeGrooteFregly2016Muscle class
/// (which doesn't inherit from OpenSim::Force).
/// Groote, F., Kinney, A. L., Rao, A. V., & Fregly, B. J. (2016). Evaluation of
/// Direct Collocation Optimal Control Problem Formulations for Solving the
/// Muscle Redundancy Problem. Annals of Biomedical Engineering, 44(10), 1–15.
/// http://doi.org/10.1007/s10439-016-1591-9
class /*OSIMMUSCOLLO_API*/DGF2016Muscle : public PathActuator {
    OpenSim_DECLARE_CONCRETE_OBJECT(DGF2016Muscle, PathActuator);
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

    DGF2016Muscle() {
        constructProperties();
    }

    void extendFinalizeFromProperties() override {
        Super::extendFinalizeFromProperties();
        OPENSIM_THROW_IF(!getProperty_optimal_force().getValueIsDefault(),
        Exception, "The optimal_force property is ignored for this Force; "
        "use max_isometric_force instead.");

        // TODO validate properties (nonnegative, etc.).

        m_maxContractionVelocityInMeters =
                get_max_contraction_velocity() * get_optimal_fiber_length();
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
            auto calcResidual = [this, &activation,
                    &muscleTendonLength,
                    &normFiberLength](const SimTK::Real& normFiberVelocity) {
                return calcFiberEquilibriumResidual(activation,
                        muscleTendonLength, normFiberLength, normFiberVelocity);
            };
            const double TODO = 10;
            const auto x = createVectorLinspace(1000, -TODO, TODO);
            TimeSeriesTable table;
            table.setColumnLabels({"residual"});
            SimTK::RowVector row(1);
            for (int i = 0; i < x.nrow(); ++i) {
                row[0] = calcResidual(x[i]);
                table.appendRow(x[i], row);
            }
            STOFileAdapter::write(table, "DEBUG_equilibrium_residual.sto");
            // TODO bounds of -1 and 1?
            const SimTK::Real equilNormFiberVelocity =
                    solveBisection(calcResidual, -TODO, TODO); // TODO -1, 1);

            std::cout << format("DEBUG derivatives t: %f normFiberVelocity: %f",
                    s.getTime(), equilNormFiberVelocity) << std::endl;

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
            const SimTK::Real& xTolerance = 1e-10,
            const SimTK::Real& yTolerance = 1e-10,
            int maxIterations = 100) const {
        SimTK::Real midpoint = left;

        assert(maxIterations > 0);
        OPENSIM_THROW_IF(calcResidual(left) * calcResidual(right) >= 0,
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
        std::cout << "DEBUG " << iterCount << std::endl;
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
                << std::endl;*/
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
        return normActiveForce + passiveForceMult;

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

    /// This is the passive force-length curve.
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
        return c1 * exp(kT * (normTendonLength - c2)) - c3;
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
    constexpr static double kT = 35;
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

    SimTK::Real m_maxContractionVelocityInMeters;
};

const std::string DGF2016Muscle::ACTIVATION("activation");
const std::string DGF2016Muscle::NORM_FIBER_LENGTH("norm_fiber_length");

Model createHangingMuscleModel() {
    Model model;
    model.setName("isometric_muscle");
    model.set_gravity(SimTK::Vec3(9.81, 0, 0));
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);

    // Allows translation along x.
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("height");
    model.addComponent(joint);

    auto* actu = new DGF2016Muscle();
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_tendon_slack_length(0.05);
    //actu->set_pennation_angle_at_optimal(0.1);
    //actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);


    /*
    auto* actu = new Millard2012EquilibriumMuscle();
    actu->set_fiber_damping(0); // TODO
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_ignore_tendon_compliance(true);
    actu->set_tendon_slack_length(0.05);
    actu->set_pennation_angle_at_optimal(0.1);
    actu->set_max_contraction_velocity(10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(actu);
    */

    /*
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("actuator");
    actu->setCoordinate(&coord);
    actu->set_activation_time_constant(0.001);
    actu->set_optimal_force(30);
    model.addComponent(actu);

    auto* contr = new PrescribedController();
    contr->setName("controller");
    contr->addActuator(*actu);
    contr->prescribeControlForActuator("actuator", new Constant(1.0));
    model.addComponent(contr);
    */

    body->attachGeometry(new Sphere(0.05));

    return model;
}

int main() {

    DGF2016Muscle m;
    printMessage("%f %f %f %f %f %f\n",
            m.calcTendonForceMultiplier(1),
            m.calcPassiveForceMultiplier(1),
            m.calcActiveForceLengthMultiplier(1),
            m.calcForceVelocityMultiplier(-1),
            m.calcForceVelocityMultiplier(0),
            m.calcForceVelocityMultiplier(1));


    MucoTool muco;
    MucoProblem& mp = muco.updProblem();
    Model model = createHangingMuscleModel();

    // auto* controller = new PrescribedController();
    // controller->addActuator(model.getComponent<Actuator>("actuator"));
    // controller->prescribeControlForActuator("actuator", new Constant(0.5));
    // model.addController(controller);

    SimTK::State state = model.initSystem();
    const auto& actuator = model.getComponent("actuator");
    const DGF2016Muscle* dgf = dynamic_cast<const DGF2016Muscle*>(&actuator);
    bool usingDGF = dgf != nullptr;
    bool hasFiberDynamics = !(
            usingDGF ? dgf->get_ignore_tendon_compliance() :
            dynamic_cast<const Muscle&>(actuator).get_ignore_tendon_compliance());


    if (hasFiberDynamics) {
        model.setStateVariableValue(state, "actuator/activation", 0.01);
        model.setStateVariableValue(state, "joint/height/value", 0.15);
        if (usingDGF) {
            model.realizePosition(state);
            model.getComponent<DGF2016Muscle>("actuator").computeInitialFiberEquilibrium(state);
            std::cout << "DEBUG " << model.getStateVariableValue(state, "actuator/norm_fiber_length")
                    << std::endl;
        } else {
            model.equilibrateMuscles(state);
        }
    }

    // TODO
    // Manager manager(model, state);
    // manager.integrate(2.0);
    // visualize(model, manager.getStateStorage());
    // std::exit(-1);

    //std::cout << "DEBUG " <<
    //        model.getStateVariableValue(state, "actuator/fiber_length")
    //        << std::endl;
    //model.equilibrateMuscles(state);
    //std::cout << "DEBUG " <<
    //        model.getStateVariableValue(state, "actuator/fiber_length")
    //        << std::endl;
    mp.setModel(model);
    mp.setTimeBounds(0, {0.05, 1.0});
    // TODO this might have been the culprit when using the Millard muscle:
    // TODO TODO TODO
    mp.setStateInfo("joint/height/value", {0.10, 0.20}, 0.15, 0.14);
    mp.setStateInfo("joint/height/speed", {-10, 10}, 0, 0);
    // TODO initial fiber length?
    // TODO how to enforce initial equilibrium?
    if (hasFiberDynamics) {
        if (usingDGF) {
            std::cout << "DEBUG " <<
                    model.getStateVariableValue(state, "actuator/norm_fiber_length") << std::endl;
            mp.setStateInfo("actuator/norm_fiber_length", {0.2, 1.8},
                    model.getStateVariableValue(state, "actuator/norm_fiber_length"));
        } else {
            mp.setStateInfo("actuator/fiber_length", {0, 0.3},
                    model.getStateVariableValue(state, "actuator/fiber_length"));
        }
    }
    // OpenSim might not allow activations of 0.
    mp.setStateInfo("actuator/activation", {0.01, 1}, 0.01);
    mp.setControlInfo("actuator", {0.01, 1});

    mp.addCost(MucoFinalTimeCost());

    // TODO try ActivationCoordinateActuator first.
    // TODO i feel like the force-velocity effect is much more strict than it
    // should be.

    MucoTropterSolver& solver = muco.initSolver();
    // TODO set initial guess from forward simulation.
    solver.setGuess("forward-simulation");

    MucoSolution solution = muco.solve().unseal();
    solution.write("sandboxMuscle_solution.sto");
    std::cout << "DEBUG " << solution.getState("joint/height/value") << std::endl;
    std::cout << "DEBUG " << solution.getState("joint/height/speed") << std::endl;

    // TODO perform forward simulation using optimized controls; see if we
    // end up at the correct final state.
    {

        // Add a controller to the model.
        const SimTK::Vector& time = solution.getTime();
        const auto control = solution.getControl("actuator");
        auto* controlFunction = new GCVSpline(5, time.nrow(), &time[0],
                &control[0]);
        auto* controller = new PrescribedController();
        controller->addActuator(model.getComponent<Actuator>("actuator"));
        controller->prescribeControlForActuator("actuator", controlFunction);
        model.addController(controller);

        // Set the initial state.
        SimTK::State state = model.initSystem();
        model.setStateVariableValue(state, "joint/height/value", 0.15);
        model.setStateVariableValue(state, "actuator/activation", 0);

        // Integrate.
        Manager manager(model, state);
        SimTK::State finalState = manager.integrate(time[time.nrow() - 1]);
        std::cout << "DEBUG "
                << model.getStateVariableValue(finalState, "joint/height/value")
                << std::endl;
        SimTK_TEST_EQ_TOL(
                model.getStateVariableValue(finalState, "joint/height/value"),
                0.14, 1e-4);
        manager.getStateStorage().print("sandboxMuscle_timestepping.sto");
    }

    // Test that the force-velocity curve inverse is correct.
    // TODO move into the actual test case.
    const auto normFiberVelocity = createVectorLinspace(100, -1, 1);
    DGF2016Muscle muscle;
    for (int i = 0; i < normFiberVelocity.nrow(); ++i) {
        const SimTK::Real& vMTilde = normFiberVelocity[i];
        SimTK_TEST_EQ(muscle.calcForceVelocityInverseCurve(
                muscle.calcForceVelocityMultiplier(vMTilde)), vMTilde);
    }


    // TODO support constraining initial fiber lengths to their equilibrium
    // lengths in Tropter!!!!!!!!!!!!!! (in explicit mode).

    return EXIT_SUCCESS;
}

