/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testMocoActuators.cpp                                        *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
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

#include <OpenSim/Actuators/ActivationCoordinateActuator.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Moco/osimMoco.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

// Function to compute fiber force (or fiber force along tendon) versus fiber
// length (or fiber length along tendon). This checks fiber stiffness
// calculations in DeGrooteFregly2016Muscle.
class FiberForceFunction : public SimTK::Differentiator::ScalarFunction {

public:
    FiberForceFunction(const DeGrooteFregly2016Muscle& muscle,
            const SimTK::State& state, bool alongTendon)
            : SimTK::Differentiator::ScalarFunction(), m_muscle(&muscle),
              m_state(&state), m_alongTendon(alongTendon) {}

    int f(SimTK::Real x, SimTK::Real& fx) const override {
        using SimTK::square;

        const auto& s = *m_state;
        const auto& optimalFiberLength = m_muscle->get_optimal_fiber_length();
        const auto fiberWidth =
                sin(m_muscle->get_pennation_angle_at_optimal()) *
                optimalFiberLength;
        SimTK::Real fiberLength = 0.0;
        SimTK::Real fiberLengthAlongTendon = 0.0;
        SimTK::Real cosPennationAngle = 0.0;
        if (m_alongTendon) {
            fiberLengthAlongTendon = x;
            fiberLength = sqrt(square(fiberWidth) + square(x));
            cosPennationAngle = fiberLengthAlongTendon / fiberLength;
        } else {
            fiberLength = x;
            fiberLengthAlongTendon = sqrt(square(x) - square(fiberWidth));
        }

        const auto& normFiberVelocity = m_muscle->getNormalizedFiberVelocity(s);
        const auto& normFiberLength = fiberLength / optimalFiberLength;

        const auto& activation = m_muscle->getActivation(s);
        const auto& activeForceLengthMultiplier =
                m_muscle->calcActiveForceLengthMultiplier(normFiberLength);
        const auto& forceVelocityMultiplier =
                m_muscle->calcForceVelocityMultiplier(normFiberVelocity);
        const auto& normPassiveFiberForce =
                m_muscle->calcPassiveForceMultiplier(normFiberLength);

        SimTK::Real activeFiberForce;
        SimTK::Real conPassiveFiberForce;
        SimTK::Real nonConPassiveFiberForce;
        SimTK::Real totalFiberForce;
        m_muscle->calcFiberForce(activation, activeForceLengthMultiplier,
                forceVelocityMultiplier, normPassiveFiberForce,
                normFiberVelocity, activeFiberForce, conPassiveFiberForce,
                nonConPassiveFiberForce, totalFiberForce);

        if (m_alongTendon) {
            fx = cosPennationAngle * totalFiberForce;
        } else {
            fx = totalFiberForce;
        }

        return EXIT_SUCCESS;
    }

private:
    SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_muscle;
    SimTK::ReferencePtr<const SimTK::State> m_state;
    bool m_alongTendon = false;
};

// Function to compute tendon force versus tendon length. This checks tendon
// stiffness calculations in DeGrooteFregly2016Muscle.
class TendonForceFunction : public SimTK::Differentiator::ScalarFunction {

public:
    TendonForceFunction(const DeGrooteFregly2016Muscle& muscle)
            : SimTK::Differentiator::ScalarFunction(), m_muscle(&muscle) {}

    int f(SimTK::Real x, SimTK::Real& fx) const override {

        const auto& tendonLength = x;
        const auto& normTendonLength =
                tendonLength / m_muscle->get_tendon_slack_length();
        const auto& normTendonForce =
                m_muscle->calcTendonForceMultiplier(normTendonLength);
        const auto& tendonForce =
                m_muscle->get_max_isometric_force() * normTendonForce;

        fx = tendonForce;

        return EXIT_SUCCESS;
    }

private:
    SimTK::ReferencePtr<const DeGrooteFregly2016Muscle> m_muscle;
};

TEST_CASE("DeGrooteFregly2016Muscle basics") {

    Model model;
    model.setName("muscle");
    auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
    model.addComponent(body);
    auto* joint = new SliderJoint("joint", model.getGround(), *body);
    auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
    coord.setName("x");
    model.addComponent(joint);
    auto* musclePtr = new DeGrooteFregly2016Muscle();
    musclePtr->set_ignore_tendon_compliance(true);
    musclePtr->set_fiber_damping(0);
    musclePtr->setName("muscle");
    musclePtr->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    musclePtr->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addComponent(musclePtr);
    auto& muscle = model.updComponent<DeGrooteFregly2016Muscle>("muscle");

    SECTION("Property value bounds") {
        DeGrooteFregly2016Muscle musc = muscle;

        SECTION("optimal_force") {
            musc.set_optimal_force(1.5);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(), Exception);
        }
        SECTION("activation_time_constant") {
            musc.set_activation_time_constant(0);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("deactivation_time_constant") {
            musc.set_deactivation_time_constant(0);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("default_activation") {
            musc.set_default_activation(-0.0001);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("default_normalized_tendon_force") {
            musc.set_default_normalized_tendon_force(5.00001);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("active_force_width_scale") {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_active_force_width_scale(0.99999999);
            SimTK_TEST_MUST_THROW_EXC(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("fiber_damping") {
            DeGrooteFregly2016Muscle musc = muscle;
            musc.set_fiber_damping(-0.0001);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("tendon_strain_at_one_norm_force") {
            musc.set_tendon_strain_at_one_norm_force(0);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("passive_fiber_strain_at_one_norm_force") {
            musc.set_passive_fiber_strain_at_one_norm_force(0);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
    }

    SECTION("printCurvesToSTOFiles") { muscle.printCurvesToSTOFiles(); }

    SECTION("Curve values") {
        CHECK(muscle.calcTendonForceMultiplier(1) == 0);

        CHECK(muscle.calcTendonForceMultiplier(
                      1 + muscle.get_tendon_strain_at_one_norm_force()) ==
                Approx(1).epsilon(1e-10));

        CHECK(muscle.calcPassiveForceMultiplier(1) ==
                Approx(0.0182288).epsilon(1e-4));
        CHECK(muscle.calcPassiveForceMultiplier(0.2) ==
                Approx(0).epsilon(1e-4));
        CHECK(muscle.calcPassiveForceMultiplier(
                      1 +
                      muscle.get_passive_fiber_strain_at_one_norm_force()) ==
                Approx(1).epsilon(1e-4));

        CHECK(muscle.calcActiveForceLengthMultiplier(1) == Approx(1));
        CHECK(muscle.calcForceVelocityMultiplier(-1) == 0);
        CHECK(muscle.calcForceVelocityMultiplier(0) == Approx(1));
        CHECK(muscle.calcForceVelocityMultiplier(1) ==
                Approx(1.794).epsilon(1e-3));
    }

    SECTION("Verify computed values") {
        auto state = model.initSystem();
        SECTION("(length) = (optimal fiber length) + (tendon slack length)") {

            double activation = 1.0;
            double fiberLength = muscle.get_optimal_fiber_length();
            double tendonLength = muscle.get_tendon_slack_length();
            double speed = 0.0;

            muscle.setActivation(state, activation);
            coord.setValue(state, fiberLength + tendonLength);
            coord.setSpeedValue(state, speed);

            model.realizePosition(state);
            CHECK(muscle.getFiberLength(state) ==
                    Approx(muscle.get_optimal_fiber_length()));
            CHECK(muscle.getNormalizedFiberLength(state) == Approx(1.0));
            CHECK(muscle.getPennationAngle(state) == Approx(0.0));
            CHECK(muscle.getCosPennationAngle(state) == 1.0);
            CHECK(muscle.getTendonLength(state) ==
                    muscle.get_tendon_slack_length());
            CHECK(muscle.getFiberLengthAlongTendon(state) ==
                    Approx(muscle.get_optimal_fiber_length()));
            CHECK(muscle.getTendonStrain(state) == 0.0);
            CHECK(muscle.getPassiveForceMultiplier(state) ==
                    Approx(muscle.calcPassiveForceMultiplier(1.0)));
            CHECK(muscle.getActiveForceLengthMultiplier(state) == Approx(1.0));
            const auto fiberPotentialEnergy =
                    muscle.calcPassiveForceMultiplierIntegral(1.0) *
                    muscle.get_optimal_fiber_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getFiberPotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy));
            const auto tendonPotentialEnergy = 0.0;
            CHECK(muscle.getTendonPotentialEnergy(state) ==
                    Approx(tendonPotentialEnergy));
            CHECK(muscle.getMusclePotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy + tendonPotentialEnergy));

            model.realizeVelocity(state);
            CHECK(muscle.getFiberVelocity(state) == 0);
            CHECK(muscle.getNormalizedFiberVelocity(state) == 0);
            CHECK(muscle.getFiberVelocityAlongTendon(state) == 0);
            CHECK(muscle.getPennationAngularVelocity(state) == 0);
            CHECK(muscle.getTendonVelocity(state) == 0);
            CHECK(muscle.getForceVelocityMultiplier(state) == Approx(1.0));

            model.realizeDynamics(state);
            const auto Fmax = muscle.getMaxIsometricForce();
            const auto fpass = muscle.calcPassiveForceMultiplier(1.0);
            CHECK(muscle.getActiveFiberForce(state) == Approx(Fmax));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) == Approx(Fmax));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            const auto fiberForce = Fmax * (1 + fpass);
            CHECK(muscle.getFiberForce(state) == Approx(fiberForce));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (1 + fpass)));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (1 + fpass)));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness = diffFiberStiffness.calcDerivative(
                    muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                            muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            SimTK::Real tendonStiffness = SimTK::Infinity;
            CHECK(muscle.getTendonStiffness(state) == SimTK::Infinity);
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) == Approx(0.0));
            CHECK(muscle.getFiberPassivePower(state) == Approx(0.0));
            CHECK(muscle.getTendonPower(state) == Approx(0.0));
            CHECK(muscle.getMusclePower(state) == Approx(0.0));
            CHECK(muscle.getStress(state) == Approx(1 + fpass));

            activation = 0.38;
            muscle.setActivation(state, activation);
            model.realizeDynamics(state);
            CHECK(muscle.getActiveFiberForce(state) ==
                    Approx(activation * Fmax));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(activation * Fmax));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getFiberForce(state) ==
                    Approx(Fmax * (activation + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (activation + fpass)));
            CHECK(muscle.getTendonForce(state) ==
                    Approx(Fmax * (activation + fpass)));
        }

        SECTION("(length) = 0.5*(optimal fiber length) + (tendon slack "
                "length)") {
            double activation = 1.0;
            double fiberLength = 0.5 * muscle.get_optimal_fiber_length();
            double tendonLength = muscle.get_tendon_slack_length();
            double speed = 0.0;

            muscle.setActivation(state, activation);
            coord.setValue(state, fiberLength + tendonLength);
            coord.setSpeedValue(state, speed);

            model.realizePosition(state);
            CHECK(muscle.getFiberLength(state) ==
                    Approx(0.5 * muscle.get_optimal_fiber_length()));
            CHECK(muscle.getNormalizedFiberLength(state) == Approx(0.5));
            CHECK(muscle.getPennationAngle(state) == 0.0);
            CHECK(muscle.getCosPennationAngle(state) == Approx(1.0));
            CHECK(muscle.getTendonLength(state) ==
                    muscle.get_tendon_slack_length());
            CHECK(muscle.getFiberLengthAlongTendon(state) ==
                    Approx(0.5 * muscle.get_optimal_fiber_length()));
            CHECK(muscle.getTendonStrain(state) == 0.0);
            CHECK(muscle.getPassiveForceMultiplier(state) ==
                    Approx(muscle.calcPassiveForceMultiplier(0.5)));
            CHECK(muscle.getActiveForceLengthMultiplier(state) ==
                    Approx(muscle.calcActiveForceLengthMultiplier(0.5)));
            const auto fiberPotentialEnergy =
                    muscle.calcPassiveForceMultiplierIntegral(0.5) *
                    muscle.get_optimal_fiber_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getFiberPotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy));
            const auto tendonPotentialEnergy = 0.0;
            CHECK(muscle.getTendonPotentialEnergy(state) ==
                    Approx(tendonPotentialEnergy));
            CHECK(muscle.getMusclePotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy + tendonPotentialEnergy));

            model.realizeVelocity(state);
            CHECK(muscle.getFiberVelocity(state) == 0);
            CHECK(muscle.getNormalizedFiberVelocity(state) == 0);
            CHECK(muscle.getFiberVelocityAlongTendon(state) == 0);
            CHECK(muscle.getPennationAngularVelocity(state) == 0);
            CHECK(muscle.getTendonVelocity(state) == 0);
            CHECK(muscle.getForceVelocityMultiplier(state) == Approx(1.0));

            model.realizeDynamics(state);
            const auto Fmax = muscle.getMaxIsometricForce();
            const auto fl = muscle.calcActiveForceLengthMultiplier(0.5);
            const auto fpass = muscle.calcPassiveForceMultiplier(0.5);
            CHECK(muscle.getActiveFiberForce(state) == Approx(Fmax * fl));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fl));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (fl + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (fl + fpass)));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (fl + fpass)));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness = diffFiberStiffness.calcDerivative(
                    0.5 * muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                            0.5 * muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            SimTK::Real tendonStiffness = SimTK::Infinity;
            CHECK(muscle.getTendonStiffness(state) == SimTK::Infinity);
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) == Approx(0.0));
            CHECK(muscle.getFiberPassivePower(state) == Approx(0.0));
            CHECK(muscle.getTendonPower(state) == Approx(0.0));
            CHECK(muscle.getMusclePower(state) == Approx(0.0));

            CHECK(muscle.getStress(state) == Approx(fl + fpass));
        }

        SECTION("(normalized fiber velocity) = 0.21") {
            muscle.setActivation(state, 1.0);
            coord.setValue(state, muscle.get_optimal_fiber_length() +
                                          muscle.get_tendon_slack_length());
            const double Vmax = muscle.get_optimal_fiber_length() *
                                muscle.get_max_contraction_velocity();
            coord.setSpeedValue(state, 0.21 * Vmax);
            model.realizePosition(state);
            CHECK(muscle.getFiberLength(state) ==
                    Approx(muscle.get_optimal_fiber_length()));
            CHECK(muscle.getNormalizedFiberLength(state) == Approx(1.0));
            CHECK(muscle.getPennationAngle(state) == 0.0);
            CHECK(muscle.getCosPennationAngle(state) == Approx(1.0));
            CHECK(muscle.getTendonLength(state) ==
                    muscle.get_tendon_slack_length());
            CHECK(muscle.getFiberLengthAlongTendon(state) ==
                    Approx(muscle.get_optimal_fiber_length()));
            CHECK(muscle.getTendonStrain(state) == 0.0);
            CHECK(muscle.getPassiveForceMultiplier(state) ==
                    Approx(muscle.calcPassiveForceMultiplier(1.0)));
            CHECK(muscle.getActiveForceLengthMultiplier(state) == Approx(1.0));
            const auto fiberPotentialEnergy =
                    muscle.calcPassiveForceMultiplierIntegral(1.0) *
                    muscle.get_optimal_fiber_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getFiberPotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy));
            const auto tendonPotentialEnergy = 0.0;
            CHECK(muscle.getTendonPotentialEnergy(state) ==
                    Approx(tendonPotentialEnergy));
            CHECK(muscle.getMusclePotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy + tendonPotentialEnergy));

            model.realizeVelocity(state);
            CHECK(muscle.getFiberVelocity(state) == 0.21 * Vmax);
            CHECK(muscle.getNormalizedFiberVelocity(state) == 0.21);
            CHECK(muscle.getFiberVelocityAlongTendon(state) == 0.21 * Vmax);
            CHECK(muscle.getPennationAngularVelocity(state) == 0);
            CHECK(muscle.getTendonVelocity(state) == 0);
            CHECK(muscle.getForceVelocityMultiplier(state) ==
                    muscle.calcForceVelocityMultiplier(0.21));

            model.realizeDynamics(state);
            const auto Fmax = muscle.getMaxIsometricForce();
            const auto fv = muscle.calcForceVelocityMultiplier(0.21);
            const auto fpass = muscle.calcPassiveForceMultiplier(1.0);
            CHECK(muscle.getActiveFiberForce(state) == Approx(Fmax * fv));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fv));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (fv + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (fv + fpass)));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (fv + fpass)));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness = diffFiberStiffness.calcDerivative(
                    muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                            muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            SimTK::Real tendonStiffness = SimTK::Infinity;
            CHECK(muscle.getTendonStiffness(state) == SimTK::Infinity);
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) ==
                    Approx(-0.21 * Vmax * Fmax * fv));
            CHECK(muscle.getFiberPassivePower(state) ==
                    Approx(-0.21 * Vmax * Fmax * fpass));
            CHECK(muscle.getTendonPower(state) == Approx(0.0));
            CHECK(muscle.getMusclePower(state) ==
                    Approx(-0.21 * Vmax * Fmax * (fv + fpass)));
            CHECK(muscle.getStress(state) == Approx(fv + fpass));

            double a = 0.38;
            muscle.setActivation(state, a);
            model.realizeDynamics(state);
            CHECK(muscle.getActiveFiberForce(state) == Approx(a * fv * Fmax));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(a * fv * Fmax));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getFiberForce(state) ==
                    Approx(Fmax * (a * fv + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (a * fv + fpass)));
            CHECK(muscle.getTendonForce(state) ==
                    Approx(Fmax * (a * fv + fpass)));

            double damping = 0.03;
            muscle.set_fiber_damping(damping);
            auto stateDamped = model.initSystem();
            stateDamped.updY() = state.getY();
            model.realizeDynamics(stateDamped);
            CHECK(muscle.getPassiveFiberDampingForce(stateDamped) ==
                    Approx(Fmax * damping * 0.21));
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(stateDamped) ==
                    Approx(Fmax * damping * 0.21));
        }

        SECTION("(normalized fiber velocity) = -1") {
            muscle.setActivation(state, 1.0);
            coord.setValue(state, muscle.get_optimal_fiber_length() +
                                          muscle.get_tendon_slack_length());
            const double Vmax = muscle.get_optimal_fiber_length() *
                                muscle.get_max_contraction_velocity();
            coord.setSpeedValue(state, -1.0 * Vmax);
            model.realizePosition(state);
            CHECK(muscle.getFiberLength(state) ==
                    Approx(muscle.get_optimal_fiber_length()));
            CHECK(muscle.getNormalizedFiberLength(state) == Approx(1.0));
            CHECK(muscle.getPennationAngle(state) == 0.0);
            CHECK(muscle.getCosPennationAngle(state) == 1.0);
            CHECK(muscle.getTendonLength(state) ==
                    muscle.get_tendon_slack_length());
            CHECK(muscle.getFiberLengthAlongTendon(state) ==
                    Approx(muscle.get_optimal_fiber_length()));
            CHECK(muscle.getTendonStrain(state) == 0.0);
            CHECK(muscle.getPassiveForceMultiplier(state) ==
                    Approx(muscle.calcPassiveForceMultiplier(1.0)));
            CHECK(muscle.getActiveForceLengthMultiplier(state) == Approx(1.0));
            const auto fiberPotentialEnergy =
                    muscle.calcPassiveForceMultiplierIntegral(1.0) *
                    muscle.get_optimal_fiber_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getFiberPotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy));
            const auto tendonPotentialEnergy = 0.0;
            CHECK(muscle.getTendonPotentialEnergy(state) ==
                    Approx(tendonPotentialEnergy));
            CHECK(muscle.getMusclePotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy + tendonPotentialEnergy));

            model.realizeVelocity(state);
            CHECK(muscle.getFiberVelocity(state) == -1.0 * Vmax);
            CHECK(muscle.getNormalizedFiberVelocity(state) == -1);
            CHECK(muscle.getFiberVelocityAlongTendon(state) == -1 * Vmax);
            CHECK(muscle.getPennationAngularVelocity(state) == 0);
            CHECK(muscle.getTendonVelocity(state) == 0);
            CHECK(muscle.getForceVelocityMultiplier(state) == 0);

            model.realizeDynamics(state);
            const auto Fmax = muscle.getMaxIsometricForce();
            const auto fpass = muscle.calcPassiveForceMultiplier(1.0);
            CHECK(muscle.getActiveFiberForce(state) == 0);
            CHECK(muscle.getActiveFiberForceAlongTendon(state) == 0);
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * fpass));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness = diffFiberStiffness.calcDerivative(
                    muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                            muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            SimTK::Real tendonStiffness = SimTK::Infinity;
            CHECK(muscle.getTendonStiffness(state) == SimTK::Infinity);
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) == Approx(0.0));
            CHECK(muscle.getFiberPassivePower(state) ==
                    Approx(Vmax * Fmax * fpass));
            CHECK(muscle.getTendonPower(state) == Approx(0.0));
            CHECK(muscle.getMusclePower(state) == Approx(Vmax * Fmax * fpass));
            CHECK(muscle.getStress(state) == Approx(fpass));

            double damping = 0.03;
            muscle.set_fiber_damping(damping);
            auto stateDamped = model.initSystem();
            stateDamped.updY() = state.getY();
            model.realizeDynamics(stateDamped);
            CHECK(muscle.getPassiveFiberDampingForce(stateDamped) ==
                    Approx(Fmax * damping * -1.0));
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(stateDamped) ==
                    Approx(Fmax * damping * -1.0));
        }

        SECTION("(active force width scale) = 1.2") {
            auto& mutMuscle =
                    model.updComponent<DeGrooteFregly2016Muscle>("muscle");
            mutMuscle.set_active_force_width_scale(1.2);
            muscle.setActivation(state, 1.0);
            const double normFiberLength = 0.8;
            coord.setValue(state, 
                    normFiberLength * muscle.get_optimal_fiber_length() +
                                      muscle.get_tendon_slack_length());
            coord.setSpeedValue(state, 0.0);
            model.realizePosition(state);

            model.realizeDynamics(state);
            const auto Fmax = muscle.getMaxIsometricForce();
            const auto fpass =
                    muscle.calcPassiveForceMultiplier(normFiberLength);
            const auto fal =
                    muscle.calcActiveForceLengthMultiplier(normFiberLength);
            CHECK(muscle.getActiveFiberForce(state) == Approx(Fmax * fal));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fal));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (fal + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (fal + fpass)));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (fal + fpass)));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness = diffFiberStiffness.calcDerivative(
                   normFiberLength * muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                        normFiberLength * muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            SimTK::Real tendonStiffness = SimTK::Infinity;
            CHECK(muscle.getTendonStiffness(state) == SimTK::Infinity);
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) == Approx(0.0));
            CHECK(muscle.getFiberPassivePower(state) == Approx(0.0));
            CHECK(muscle.getTendonPower(state) == Approx(0.0));
            CHECK(muscle.getMusclePower(state) == Approx(0.0));
            CHECK(muscle.getStress(state) == Approx(fal + fpass));
        }

        SECTION("pennation") {
            auto& mutMuscle =
                    model.updComponent<DeGrooteFregly2016Muscle>("muscle");
            const double pennOpt = 0.12;
            const double cosPenn = cos(pennOpt);
            mutMuscle.set_pennation_angle_at_optimal(pennOpt);
            state = model.initSystem();
            muscle.setActivation(state, 1.0);
            coord.setValue(state, muscle.get_optimal_fiber_length() * cosPenn +
                                          muscle.get_tendon_slack_length());
            const double Vmax = muscle.get_optimal_fiber_length() *
                                muscle.get_max_contraction_velocity();
            coord.setSpeedValue(state, -1.0 * Vmax);

            model.realizePosition(state);
            CHECK(muscle.getFiberLength(state) ==
                    muscle.get_optimal_fiber_length());
            CHECK(muscle.getNormalizedFiberLength(state) == 1.0);
            CHECK(muscle.getPennationAngle(state) == Approx(pennOpt));
            CHECK(muscle.getCosPennationAngle(state) == Approx(cosPenn));
            CHECK(muscle.getTendonLength(state) ==
                    muscle.get_tendon_slack_length());
            CHECK(muscle.getFiberLengthAlongTendon(state) ==
                    Approx(muscle.get_optimal_fiber_length() * cosPenn));
            CHECK(muscle.getTendonStrain(state) == 0.0);
            CHECK(muscle.getPassiveForceMultiplier(state) ==
                    Approx(muscle.calcPassiveForceMultiplier(1.0)));
            CHECK(muscle.getActiveForceLengthMultiplier(state) == Approx(1.0));
            const auto fiberPotentialEnergy =
                    muscle.calcPassiveForceMultiplierIntegral(1.0) *
                    muscle.get_optimal_fiber_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getFiberPotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy));
            const auto tendonPotentialEnergy = 0.0;
            CHECK(muscle.getTendonPotentialEnergy(state) ==
                    Approx(tendonPotentialEnergy));
            CHECK(muscle.getMusclePotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy + tendonPotentialEnergy));

            model.realizeVelocity(state);
            CHECK(muscle.getFiberVelocity(state) == -Vmax * cosPenn);
            CHECK(muscle.getNormalizedFiberVelocity(state) == -cosPenn);
            CHECK(muscle.getFiberVelocityAlongTendon(state) == -Vmax); // VMT
            CHECK(muscle.getPennationAngularVelocity(state) ==
                    Approx(muscle.get_max_contraction_velocity() * cosPenn *
                            tan(pennOpt)));
            CHECK(muscle.getTendonVelocity(state) == 0);
            CHECK(muscle.getForceVelocityMultiplier(state) ==
                    Approx(muscle.calcForceVelocityMultiplier(-cosPenn)));

            model.realizeDynamics(state);
            const auto Fmax = muscle.getMaxIsometricForce();
            const auto fv = muscle.calcForceVelocityMultiplier(-cosPenn);
            const auto fpass = muscle.calcPassiveForceMultiplier(1.0);
            CHECK(muscle.getActiveFiberForce(state) == Approx(Fmax * fv));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fv * cosPenn));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass * cosPenn));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(Fmax * fpass * cosPenn));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (fv + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (fv + fpass) * cosPenn));
            CHECK(muscle.getTendonForce(state) ==
                    Approx(Fmax * (fv + fpass) * cosPenn));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness = diffFiberStiffness.calcDerivative(
                    muscle.get_optimal_fiber_length());
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                            muscle.get_optimal_fiber_length() * cosPenn);
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            SimTK::Real tendonStiffness = SimTK::Infinity;
            CHECK(muscle.getTendonStiffness(state) == SimTK::Infinity);
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) ==
                    Approx(Vmax * cosPenn * Fmax * fv));
            CHECK(muscle.getFiberPassivePower(state) ==
                    Approx(Vmax * cosPenn * Fmax * fpass));
            CHECK(muscle.getTendonPower(state) == Approx(0.0));
            CHECK(muscle.getMusclePower(state) ==
                    Approx(Vmax * cosPenn * Fmax * (fv + fpass)));
            CHECK(muscle.getStress(state) == Approx((fv + fpass) * cosPenn));

            double damping = 0.015;
            muscle.set_fiber_damping(damping);
            auto stateDamped = model.initSystem();
            stateDamped.updY() = state.getY();
            model.realizeDynamics(stateDamped);
            CHECK(muscle.getPassiveFiberDampingForce(stateDamped) ==
                    Approx(Fmax * damping * -cosPenn));
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(stateDamped) ==
                    Approx(Fmax * damping * -cosPenn * cosPenn));
        }

        SECTION("initial equilibrium") {
            auto& mutMuscle =
                    model.updComponent<DeGrooteFregly2016Muscle>("muscle");
            mutMuscle.set_ignore_tendon_compliance(false);
            mutMuscle.set_tendon_compliance_dynamics_mode("explicit");

            const double pennOpt = 0.12;
            double cosPenn = cos(pennOpt);
            mutMuscle.set_pennation_angle_at_optimal(pennOpt);
            state = model.initSystem();
            muscle.setActivation(state, 1.0);
            const double muscleLength =
                    muscle.get_optimal_fiber_length() * cosPenn +
                    muscle.get_tendon_slack_length();
            coord.setValue(state, muscleLength);
            const double Vmax = muscle.get_optimal_fiber_length() *
                                muscle.get_max_contraction_velocity();
            const double muscleTendonVelocity = -0.21 * Vmax;
            coord.setSpeedValue(state, muscleTendonVelocity);

            model.realizeDynamics(state);
            muscle.computeInitialFiberEquilibrium(state);

            model.realizeDynamics(state);
            CHECK(muscle.getNormalizedTendonForceDerivative(state) == 
                    Approx(0.0).margin(1e-6));

            mutMuscle.set_tendon_compliance_dynamics_mode("implicit");
            model.realizeDynamics(state);
            CHECK(muscle.getEquilibriumResidual(state) == Approx(0.0));
        }

        SECTION("tendon compliance") {
            auto& mutMuscle =
                    model.updComponent<DeGrooteFregly2016Muscle>("muscle");
            mutMuscle.set_ignore_tendon_compliance(false);
            mutMuscle.set_tendon_compliance_dynamics_mode("implicit");

            const double pennOpt = 0.12;
            double cosPenn = cos(pennOpt);
            mutMuscle.set_pennation_angle_at_optimal(pennOpt);
            state = model.initSystem();
            muscle.setActivation(state, 1.0);
            const double muscleLength =
                    muscle.get_optimal_fiber_length() * cosPenn +
                    muscle.get_tendon_slack_length();
            coord.setValue(state, muscleLength);
            const double Vmax = muscle.get_optimal_fiber_length() *
                                muscle.get_max_contraction_velocity();
            const double muscleTendonVelocity = -0.21 * Vmax;
            coord.setSpeedValue(state, muscleTendonVelocity);

            model.realizeDynamics(state);
            muscle.computeInitialFiberEquilibrium(state);

            model.realizePosition(state);
            const auto& normFiberLength =
                    muscle.getNormalizedFiberLength(state);
            const auto& fiberLength =
                    normFiberLength * muscle.get_optimal_fiber_length();
            const auto& pennationAngle = muscle.getPennationAngle(state);
            const auto& cosPennationAngle = cos(pennationAngle);
            const auto& fiberLengthAlongTendon =
                    fiberLength * cosPennationAngle;
            const auto& tendonLength = muscleLength - fiberLengthAlongTendon;
            const auto& normTendonLength =
                    tendonLength / muscle.get_tendon_slack_length();
            const auto& tendonStrain = normTendonLength - 1.0;
            const auto fpass =
                    muscle.calcPassiveForceMultiplier(normFiberLength);
            const auto& fal =
                    muscle.calcActiveForceLengthMultiplier(normFiberLength);

            CHECK(muscle.getFiberLength(state) == Approx(fiberLength));
            CHECK(muscle.getNormalizedFiberLength(state) == Approx(0.9305004));
            CHECK(muscle.getPennationAngle(state) == Approx(pennationAngle));
            CHECK(muscle.getCosPennationAngle(state) ==
                    Approx(cosPennationAngle));
            CHECK(muscle.getTendonLength(state) == Approx(tendonLength));
            CHECK(muscle.getFiberLengthAlongTendon(state) ==
                    Approx(fiberLengthAlongTendon));
            CHECK(muscle.getTendonStrain(state) == Approx(tendonStrain));
            CHECK(muscle.getPassiveForceMultiplier(state) == Approx(fpass));
            CHECK(muscle.getActiveForceLengthMultiplier(state) == Approx(fal));
            const auto fiberPotentialEnergy =
                    muscle.calcPassiveForceMultiplierIntegral(normFiberLength) *
                    muscle.get_optimal_fiber_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getFiberPotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy));
            const auto tendonPotentialEnergy =
                    muscle.calcTendonForceMultiplierIntegral(normTendonLength) *
                    muscle.get_tendon_slack_length() *
                    muscle.get_max_isometric_force();
            CHECK(muscle.getTendonPotentialEnergy(state) ==
                    Approx(tendonPotentialEnergy));
            CHECK(muscle.getMusclePotentialEnergy(state) ==
                    Approx(fiberPotentialEnergy + tendonPotentialEnergy));

            model.realizeVelocity(state);
            const auto& normFiberVelocity =
                    muscle.getNormalizedFiberVelocity(state);
            const auto& fiberVelocity = Vmax * normFiberVelocity;
            const auto& fiberVelocityAlongTendon =
                    fiberVelocity / cosPennationAngle;
            const auto& tendonVelocity =
                    muscleTendonVelocity - fiberVelocityAlongTendon;
            const auto& fv =
                    muscle.calcForceVelocityMultiplier(normFiberVelocity);

            CHECK(muscle.getFiberVelocity(state) == Approx(fiberVelocity));
            CHECK(muscle.getNormalizedFiberVelocity(state) ==
                    Approx(normFiberVelocity));
            CHECK(muscle.getFiberVelocityAlongTendon(state) ==
                    Approx(fiberVelocityAlongTendon));
            CHECK(muscle.getPennationAngularVelocity(state) ==
                    Approx(-fiberVelocity / fiberLength * tan(pennationAngle)));
            CHECK(muscle.getTendonVelocity(state) == Approx(tendonVelocity));
            CHECK(muscle.getForceVelocityMultiplier(state) == Approx(fv));

            model.realizeDynamics(state);
            const auto& Fmax = muscle.getMaxIsometricForce();
            const auto& activeFiberForce = Fmax * fal * fv;
            const auto& passiveFiberForce = Fmax * fpass;
            const auto& fiberForce = activeFiberForce + passiveFiberForce;
            const auto& fiberForceAlongTendon = fiberForce * cosPennationAngle;
            const auto& tendonForce = fiberForceAlongTendon;
            CHECK(muscle.getActiveFiberForce(state) ==
                    Approx(activeFiberForce));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(activeFiberForce * cosPennationAngle));
            CHECK(muscle.getPassiveFiberForce(state) ==
                    Approx(passiveFiberForce));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(passiveFiberForce * cosPennationAngle));
            CHECK(muscle.getPassiveFiberElasticForce(state) ==
                    Approx(passiveFiberForce));
            CHECK(muscle.getPassiveFiberElasticForceAlongTendon(state) ==
                    Approx(passiveFiberForce * cosPennationAngle));
            CHECK(muscle.getPassiveFiberDampingForce(state) == 0);
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(state) == 0);
            CHECK(muscle.getFiberForce(state) == Approx(fiberForce));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(fiberForceAlongTendon));
            CHECK(muscle.getTendonForce(state) == Approx(tendonForce));

            FiberForceFunction fiberForceFunc(muscle, state, false);
            SimTK::Differentiator diffFiberStiffness(fiberForceFunc);
            SimTK::Real fiberStiffness =
                    diffFiberStiffness.calcDerivative(fiberLength);
            CHECK(muscle.getFiberStiffness(state) == Approx(fiberStiffness));

            FiberForceFunction fiberForceFuncAlongTendon(muscle, state, true);
            SimTK::Differentiator diffFiberStiffnessAlongTendon(
                    fiberForceFuncAlongTendon);
            SimTK::Real fiberStiffnessAlongTendon =
                    diffFiberStiffnessAlongTendon.calcDerivative(
                            fiberLengthAlongTendon);
            CHECK(muscle.getFiberStiffnessAlongTendon(state) ==
                    Approx(fiberStiffnessAlongTendon));

            TendonForceFunction tendonForceFunc(muscle);
            SimTK::Differentiator diffTendonStiffness(tendonForceFunc);
            SimTK::Real tendonStiffness =
                    diffTendonStiffness.calcDerivative(tendonLength);
            CHECK(muscle.getTendonStiffness(state) == Approx(tendonStiffness));
            CHECK(muscle.getMuscleStiffness(state) ==
                    Approx(muscle.calcMuscleStiffness(
                            tendonStiffness, fiberStiffnessAlongTendon)));

            CHECK(muscle.getFiberActivePower(state) ==
                    Approx(-activeFiberForce * fiberVelocity));
            // No damping, so we can use the total passive fiber force we
            // computed previously.
            CHECK(muscle.getFiberPassivePower(state) ==
                    Approx(-passiveFiberForce * fiberVelocity));
            CHECK(muscle.getTendonPower(state) ==
                    Approx(-tendonForce * tendonVelocity));
            CHECK(muscle.getMusclePower(state) ==
                    Approx(-tendonForce * muscleTendonVelocity));
            CHECK(muscle.getStress(state) == Approx(tendonForce / Fmax));

            double damping = 0.012;
            muscle.set_fiber_damping(damping);
            auto stateDamped = model.initSystem();
            stateDamped.updY() = state.getY();
            model.realizeDynamics(stateDamped);
            CHECK(muscle.getPassiveFiberDampingForce(stateDamped) ==
                    Approx(Fmax * damping * normFiberVelocity));
            CHECK(muscle.getPassiveFiberDampingForceAlongTendon(stateDamped) ==
                    Approx(Fmax * damping * normFiberVelocity * cosPennationAngle));
        }
    }

    SECTION("Force-velocity curve inverse") {

        // Test that the force-velocity curve inverse is correct.
        // ------------------------------------------------------
        const auto normFiberVelocity = createVectorLinspace(100, -1, 1);
        for (int i = 0; i < normFiberVelocity.nrow(); ++i) {
            const SimTK::Real& vMTilde = normFiberVelocity[i];
            CHECK(muscle.calcForceVelocityInverseCurve(
                          muscle.calcForceVelocityMultiplier(vMTilde)) ==
                    Approx(vMTilde));
        }
    }

    // getActivation(), setActivation()
    // --------------------------------
    SECTION("getActivation(), setActivation()") {
        SimTK::State state = model.initSystem();
        CHECK(muscle.getActivation(state) == muscle.get_default_activation());
        muscle.setActivation(state, 0.451);
        CHECK(muscle.getActivation(state) == Approx(0.451));
        CHECK(state.getY()[2] == Approx(0.451));
    }
}

Model createHangingMuscleModel(
        bool ignoreActivationDynamics, bool ignoreTendonCompliance, 
        bool isTendonDynamicsExplicit) {
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

    auto* actu = new DeGrooteFregly2016Muscle();
    actu->setName("actuator");
    actu->set_max_isometric_force(30.0);
    actu->set_optimal_fiber_length(0.10);
    actu->set_tendon_slack_length(0.05);
    actu->set_tendon_strain_at_one_norm_force(0.10);
    actu->set_ignore_activation_dynamics(ignoreActivationDynamics);
    actu->set_ignore_tendon_compliance(ignoreTendonCompliance);
    actu->set_fiber_damping(0.01);
    if (!isTendonDynamicsExplicit) {
        actu->set_tendon_compliance_dynamics_mode("implicit");
    }
    actu->set_max_contraction_velocity(10);
    actu->set_pennation_angle_at_optimal(0.10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);

    body->attachGeometry(new Sphere(0.05));

    return model;
}

TEMPLATE_TEST_CASE(
        "Hanging muscle minimum time", "[casadi]", MocoCasADiSolver) {
    auto ignoreActivationDynamics = GENERATE(true, false);
    auto ignoreTendonCompliance = GENERATE(true, false);
    auto isTendonDynamicsExplicit = GENERATE(false);

    // TODO: Some problem has a bad initial guess and the constraint violation
    // goes to 1e+14. Maybe the bounds on the coordinate should be tighter.

    CAPTURE(ignoreActivationDynamics);
    CAPTURE(ignoreTendonCompliance);
    CAPTURE(isTendonDynamicsExplicit);

    SimTK::Real initHeight = 0.15;
    SimTK::Real finalHeight = 0.14;

    Model model = createHangingMuscleModel(
            ignoreActivationDynamics, ignoreTendonCompliance,
            isTendonDynamicsExplicit);

    SimTK::State state = model.initSystem();

    // Minimum time trajectory optimization.
    // -------------------------------------
    const auto svn = model.getStateVariableNames();
    MocoSolution solutionTrajOpt;
    {
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelCopy(model);
        problem.setTimeBounds(0, {0.05, 1.0});
        problem.setStateInfo(
                "/joint/height/value", {0.14, 0.16}, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-1, 1}, 0, 0);
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        // Initial state constraints/costs.
        if (!ignoreActivationDynamics) {
            auto* initial_activation =
                    problem.addGoal<MocoInitialActivationGoal>();
            initial_activation->setName("initial_activation");
        }
        if (!ignoreTendonCompliance) {
            if (isTendonDynamicsExplicit) {
                auto* initial_equilibrium = 
                    problem.addGoal<MocoInitialForceEquilibriumGoal>();
                initial_equilibrium->setName("initial_force_equilibrium");
                // problem.setStateInfo("/forceset/actuator/normalized_tendon_force",
                // {0, 2});
            } else {
                // The problem performs better when this is in cost mode.
                auto* initial_equilibrium = problem.addGoal<
                        MocoInitialVelocityEquilibriumDGFGoal>();
                initial_equilibrium->setName("initial_velocity_equilibrium");
                initial_equilibrium->setMode("cost");
                initial_equilibrium->setWeight(0.001);
            }
        }

        problem.addGoal<MocoFinalTimeGoal>();

        auto& solver = study.initSolver<TestType>();
        solver.set_num_mesh_intervals(25);
        solver.set_multibody_dynamics_mode("implicit");
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-4);

        solutionTrajOpt = study.solve();
        std::string solutionFilename = "testDeGrooteFregly2016Muscle_solution";
        if (!ignoreActivationDynamics) solutionFilename += "_actdyn";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        if (isTendonDynamicsExplicit) solutionFilename += "_exptendyn";
        solutionFilename += ".sto";
        solutionTrajOpt.write(solutionFilename);
    }

    // Perform time stepping forward simulation using optimized controls.
    // ------------------------------------------------------------------
    // See if we end up at the correct final state.
    {
        // We need to temporarily switch the muscle in the model to explicit
        // tendon compliance dynamics mode to perform time stepping.
        auto* mutableDGFMuscle = dynamic_cast<DeGrooteFregly2016Muscle*>(
                &model.updComponent("forceset/actuator"));
        if (!ignoreTendonCompliance && !isTendonDynamicsExplicit) {
            mutableDGFMuscle->set_tendon_compliance_dynamics_mode("explicit");
        }
        const auto trajSim =
                simulateTrajectoryWithTimeStepping(solutionTrajOpt, model);
        std::string trajFilename =
                "testDeGrooteFregly2016Muscle_timestepping";
        if (!ignoreActivationDynamics) trajFilename += "_actdyn";
        if (ignoreTendonCompliance) trajFilename += "_rigidtendon";
        trajFilename += ".sto";
        trajSim.write(trajFilename);

        const double error = trajSim.compareContinuousVariablesRMS(
                solutionTrajOpt, {{"states", {}}, {"controls", {}}});
        CHECK(error < 0.05);
        if (!ignoreTendonCompliance && !isTendonDynamicsExplicit) {
            mutableDGFMuscle->set_tendon_compliance_dynamics_mode("implicit");
        }
    }

    // Track the kinematics from the trajectory optimization.
    // ------------------------------------------------------
    // We will try to recover muscle activity.
    if (!isTendonDynamicsExplicit) {
        std::cout << "Tracking the trajectory optimization coordinate solution."
                  << std::endl;
        MocoStudy study;
        MocoProblem& problem = study.updProblem();
        problem.setModelCopy(model);
        // Using an equality constraint for the time bounds was essential for
        // recovering the correct excitation.
        const double finalTime =
                solutionTrajOpt.getTime()[solutionTrajOpt.getNumTimes() - 1];
        problem.setTimeBounds(0, finalTime);
        problem.setStateInfo(
                "/joint/height/value", {0.14, 0.16}, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-1, 1}, 0, 0);
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        // Initial state constraints/costs.
        if (!ignoreActivationDynamics) {
            auto* initial_activation =
                    problem.addGoal<MocoInitialActivationGoal>();
            initial_activation->setName("initial_activation");
        }
        if (!ignoreTendonCompliance) {
            if (isTendonDynamicsExplicit) {
                auto* initial_equilibrium =
                        problem.addGoal<MocoInitialForceEquilibriumGoal>();
                initial_equilibrium->setName("initial_force_equilibrium");
            } else {
                // The problem performs better when this is in cost mode.
                auto* initial_equilibrium = problem.addGoal<
                        MocoInitialVelocityEquilibriumDGFGoal>();
                initial_equilibrium->setName("initial_velocity_equilibrium");
                initial_equilibrium->setMode("cost");
                initial_equilibrium->setWeight(0.001);
            }
        }

        auto* tracking = problem.addGoal<MocoStateTrackingGoal>();
        tracking->setName("tracking");

        auto states = solutionTrajOpt.exportToStatesTable();
        TimeSeriesTable ref(states.getIndependentColumn());
        ref.appendColumn("/joint/height/value",
                states.getDependentColumn("/joint/height/value"));
        // Tracking speed has a huge effect on getting a good solution for the
        // control signal.
        ref.appendColumn("/joint/height/speed",
                states.getDependentColumn("/joint/height/speed"));
        // Tracking joint/height/speed slightly increases the
        // iterations to converge, and tracking activation cuts the iterations
        // in half.
        tracking->setReference(ref);
        tracking->setAllowUnusedReferences(true);

        auto& solver = study.initSolver<TestType>();
        solver.set_num_mesh_intervals(25);
        solver.set_multibody_dynamics_mode("implicit");

        MocoSolution solutionTrack = study.solve();
        std::string solutionFilename =
                "testDeGrooteFregly2016Muscle_track_solution";
        if (!ignoreActivationDynamics) solutionFilename += "_actdyn";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        if (isTendonDynamicsExplicit) solutionFilename += "_exptendyn";
        solutionFilename += ".sto";
        solutionTrack.write(solutionFilename);
        double error =
                solutionTrack.compareContinuousVariablesRMS(solutionTrajOpt);
        CHECK(error < 0.015);
    }
}

TEST_CASE("ActivationCoordinateActuator") {
    // Create a problem with ACA and ensure the activation bounds are
    // set as expected.
    auto model = ModelFactory::createSlidingPointMass();
    auto* actu = new ActivationCoordinateActuator();
    actu->setName("aca");
    actu->setCoordinate(&model.updCoordinateSet().get("position"));
    actu->setMinControl(-0.31);
    actu->setMaxControl(0.78);
    model.addForce(actu);
    MocoStudy study;
    auto& problem = study.updProblem();
    problem.setModelCopy(model);
    auto rep = problem.createRep();
    CHECK(rep.getStateInfo("/forceset/aca/activation").getBounds().getLower() ==
            Approx(-0.31));
    CHECK(rep.getStateInfo("/forceset/aca/activation").getBounds().getUpper() ==
            Approx(0.78));
}
