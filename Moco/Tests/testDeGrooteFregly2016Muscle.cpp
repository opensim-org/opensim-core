/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testDeGrooteFregly2016Muscle.cpp                             *
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

#include <Moco/osimMoco.h>

#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#define CATCH_CONFIG_MAIN
#include "Testing.h"

using namespace OpenSim;

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
    auto& muscle = model.getComponent<DeGrooteFregly2016Muscle>("muscle");

    SECTION("Property value bounds") {
        DeGrooteFregly2016Muscle musc = muscle;

        SECTION("optimal_force") {
            musc.set_optimal_force(1.5);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(), Exception);
        }
        SECTION("default_normalized_fiber_length min") {
            musc.set_default_normalized_fiber_length(0.1999);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("default_normalized_fiber_length max") {
            musc.set_default_normalized_fiber_length(1.800001);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
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
        SECTION("fiber_damping") {
            musc.set_fiber_damping(-0.0001);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
        SECTION("tendon_strain_at_one_norm_force") {
            musc.set_tendon_strain_at_one_norm_force(0);
            REQUIRE_THROWS_AS(musc.finalizeFromProperties(),
                    SimTK::Exception::ErrorCheck);
        }
    }

    SECTION("printCurvesToSTOFiles") { muscle.printCurvesToSTOFiles(); }

    SECTION("Curve values") {
        CHECK(muscle.calcTendonForceMultiplier(1) == 0);
        CHECK(muscle.calcPassiveForceMultiplier(1) ==
                Approx(0.018567).epsilon(1e-4));
        CHECK(muscle.calcActiveForceLengthMultiplier(1) == Approx(1));
        CHECK(muscle.calcForceVelocityMultiplier(-1) == 0);
        CHECK(muscle.calcForceVelocityMultiplier(0) == Approx(1));
        CHECK(muscle.calcForceVelocityMultiplier(1) ==
                Approx(1.794).epsilon(1e-3));
    }

    SECTION("Verify computed values") {
        auto state = model.initSystem();
        SECTION("(length) = (optimal fiber length) + (tendon slack length)") {
            muscle.setActivation(state, 1.0);
            coord.setValue(state, muscle.get_optimal_fiber_length() +
                                          muscle.get_tendon_slack_length());
            coord.setSpeedValue(state, 0.0);
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
            CHECK(SimTK::isNaN(muscle.getFiberPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePotentialEnergy(state)));

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
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (1 + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (1 + fpass)));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (1 + fpass)));

            CHECK(SimTK::isNaN(muscle.getFiberStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberStiffnessAlongTendon(state)));
            CHECK(SimTK::isNaN(muscle.getTendonStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getMuscleStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberActivePower(state)));
            CHECK(SimTK::isNaN(muscle.getFiberPassivePower(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPower(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePower(state)));
            CHECK(muscle.getStress(state) == Approx(1 + fpass));

            double a = 0.38;
            muscle.setActivation(state, a);
            model.realizeDynamics(state);
            CHECK(muscle.getActiveFiberForce(state) == Approx(a * Fmax));
            CHECK(muscle.getActiveFiberForceAlongTendon(state) ==
                    Approx(a * Fmax));
            CHECK(muscle.getPassiveFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getPassiveFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (a + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (a + fpass)));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (a + fpass)));
        }

        SECTION("(length) = 0.5*(optimal fiber length) + (tendon slack "
                "length)") {
            muscle.setActivation(state, 1.0);
            coord.setValue(state, 0.5 * muscle.get_optimal_fiber_length() +
                                          muscle.get_tendon_slack_length());
            coord.setSpeedValue(state, 0.0);
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
            CHECK(SimTK::isNaN(muscle.getFiberPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePotentialEnergy(state)));

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
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (fl + fpass)));

            CHECK(SimTK::isNaN(muscle.getFiberStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberStiffnessAlongTendon(state)));
            CHECK(SimTK::isNaN(muscle.getTendonStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getMuscleStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberActivePower(state)));
            CHECK(SimTK::isNaN(muscle.getFiberPassivePower(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPower(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePower(state)));

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
            CHECK(SimTK::isNaN(muscle.getFiberPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePotentialEnergy(state)));

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
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (fv + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (fv + fpass)));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * (fv + fpass)));

            CHECK(SimTK::isNaN(muscle.getFiberStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberStiffnessAlongTendon(state)));
            CHECK(SimTK::isNaN(muscle.getTendonStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getMuscleStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberActivePower(state)));
            CHECK(SimTK::isNaN(muscle.getFiberPassivePower(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPower(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePower(state)));
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
            CHECK(muscle.getFiberForce(state) ==
                    Approx(Fmax * (a * fv + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (a * fv + fpass)));
            CHECK(muscle.getTendonForce(state) ==
                    Approx(Fmax * (a * fv + fpass)));
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
            CHECK(SimTK::isNaN(muscle.getFiberPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePotentialEnergy(state)));

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
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * fpass));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * fpass));
            CHECK(muscle.getTendonForce(state) == Approx(Fmax * fpass));

            CHECK(SimTK::isNaN(muscle.getFiberStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberStiffnessAlongTendon(state)));
            CHECK(SimTK::isNaN(muscle.getTendonStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getMuscleStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberActivePower(state)));
            CHECK(SimTK::isNaN(muscle.getFiberPassivePower(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPower(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePower(state)));
            CHECK(muscle.getStress(state) == Approx(fpass));
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
                    muscle.calcPassiveForceMultiplier(1.0));
            CHECK(muscle.getActiveForceLengthMultiplier(state) == 1.0);
            CHECK(SimTK::isNaN(muscle.getFiberPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPotentialEnergy(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePotentialEnergy(state)));

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
            CHECK(muscle.getFiberForce(state) == Approx(Fmax * (fv + fpass)));
            CHECK(muscle.getFiberForceAlongTendon(state) ==
                    Approx(Fmax * (fv + fpass) * cosPenn));
            CHECK(muscle.getTendonForce(state) ==
                    Approx(Fmax * (fv + fpass) * cosPenn));

            CHECK(SimTK::isNaN(muscle.getFiberStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberStiffnessAlongTendon(state)));
            CHECK(SimTK::isNaN(muscle.getTendonStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getMuscleStiffness(state)));
            CHECK(SimTK::isNaN(muscle.getFiberActivePower(state)));
            CHECK(SimTK::isNaN(muscle.getFiberPassivePower(state)));
            CHECK(SimTK::isNaN(muscle.getTendonPower(state)));
            CHECK(SimTK::isNaN(muscle.getMusclePower(state)));
            CHECK(muscle.getStress(state) == Approx((fv + fpass) * cosPenn));
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

    /* TODO solveBisection() is temporarily removed.
    SECTION("solveBisection()") {

        // solveBisection().
        // -----------------
        {
            auto calcResidual = [](const SimTK::Real& x) { return x - 3.78; };
            {
                const auto root =
                        muscle.solveBisection(calcResidual, -5, 5, 1e-6, 1e-12);
                SimTK_TEST_EQ_TOL(root, 3.78, 1e-6);
                // Make sure the x tolerance has an effect.
                SimTK_TEST_NOTEQ_TOL(root, 3.78, 1e-10);
            }
            {
                const auto root = muscle.solveBisection(
                        calcResidual, -5, 5, 1e-10, 1e-12);
                SimTK_TEST_EQ_TOL(root, 3.78, 1e-10);
            }
            // Make sure the y tolerance has an effect.
            {
                const auto root =
                        muscle.solveBisection(calcResidual, -5, 5, 1e-12, 1e-4);
                const auto residual = calcResidual(root);
                SimTK_TEST_EQ_TOL(residual, 0, 1e-4);
                // Make sure the x tolerance has an effect.
                SimTK_TEST_NOTEQ_TOL(residual, 0, 1e-10);
            }
            {
                const auto root = muscle.solveBisection(
                        calcResidual, -5, 5, 1e-12, 1e-10);
                const auto residual = calcResidual(root);
                SimTK_TEST_EQ_TOL(residual, 0, 1e-10);
            }
        }
        // Multiple roots.
        {
            auto parabola = [](const SimTK::Real& x) {
                return SimTK::square(x - 2.5);
            };
            REQUIRE_THROWS_AS(
                    muscle.solveBisection(parabola, -5, 5), Exception);
        }

    }
     */

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
        bool ignoreActivationDynamics, bool ignoreTendonCompliance) {
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
    actu->set_max_contraction_velocity(10);
    actu->set_pennation_angle_at_optimal(0.10);
    actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
    actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
    model.addForce(actu);

    body->attachGeometry(new Sphere(0.05));

    return model;
}

TEMPLATE_TEST_CASE("Hanging muscle minimum time", "", MocoCasADiSolver) {
    auto ignoreActivationDynamics = GENERATE(true, false);
    auto ignoreTendonCompliance = GENERATE(true);

    CAPTURE(ignoreActivationDynamics);
    CAPTURE(ignoreTendonCompliance);

    SimTK::Real initActivation = 0.01;
    SimTK::Real initHeight = 0.15;
    SimTK::Real finalHeight = 0.14;

    Model model = createHangingMuscleModel(
            ignoreActivationDynamics, ignoreTendonCompliance);

    SimTK::State state = model.initSystem();
    const auto& actuator = model.getComponent("forceset/actuator");

    const auto* muscle = dynamic_cast<const Muscle*>(&actuator);

    if (!ignoreTendonCompliance) {
        model.setStateVariableValue(state, "joint/height/value", initHeight);
        model.realizeVelocity(state);
        muscle->setActivation(state, initActivation);
        model.equilibrateMuscles(state);
        std::cout << "Equilibrium norm fiber length: "
                  << model.getStateVariableValue(
                             state, "forceset/actuator/norm_fiber_length")
                  << std::endl;
    }

    // Minimum time trajectory optimization.
    // -------------------------------------
    const auto svn = model.getStateVariableNames();
    MocoSolution solutionTrajOpt;
    {
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
        problem.setModelCopy(model);
        problem.setTimeBounds(0, {0.05, 1.0});
        problem.setStateInfo(
                "/joint/height/value", {0.10, 0.16}, initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-10, 10}, 0, 0);
        // TODO initial fiber length?
        // TODO how to enforce initial equilibrium with explicit dynamics?
        if (!ignoreTendonCompliance) {
            // We would prefer to use a range of [0.2, 1.8] but then IPOPT
            // tries very small fiber lengths that cause tendon stretch to
            // be HUGE, causing insanely high tendon forces.
            // TODO Try minimizing fiber velocity in the objective.
            // Set the initial value to be the equilibrium value.
            problem.setStateInfo("/forceset/actuator/norm_fiber_length",
                    {0.8, 1.8},
                    model.getStateVariableValue(
                            state, "forceset/actuator/norm_fiber_length"));
        }
        // OpenSim might not allow activations of 0.
        if (!ignoreActivationDynamics) {
            problem.setStateInfo(
                    "/forceset/actuator/activation", {0.01, 1}, initActivation);
        }
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        problem.addCost<MocoFinalTimeCost>();

        auto& solver = moco.initSolver<TestType>();
        solver.set_num_mesh_points(40);
        solver.set_dynamics_mode("implicit");
        solver.set_optim_convergence_tolerance(1e-4);
        solver.set_optim_constraint_tolerance(1e-3);

        solutionTrajOpt = moco.solve();
        std::string solutionFilename = "testDeGrooteFregly2016Muscle_solution";
        if (!ignoreActivationDynamics) solutionFilename += "_actdyn";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        solutionFilename += ".sto";
        solutionTrajOpt.write(solutionFilename);
        std::cout << "Solution joint/height/value trajectory: "
                  << solutionTrajOpt.getState("/joint/height/value")
                  << std::endl;
        std::cout << "Solution joint/height/speed trajectory: "
                  << solutionTrajOpt.getState("/joint/height/speed")
                  << std::endl;
    }

    // Perform time stepping forward simulation using optimized controls.
    // ------------------------------------------------------------------
    // See if we end up at the correct final state.
    {
        const auto iterateSim =
                simulateIterateWithTimeStepping(solutionTrajOpt, model);
        std::string iterateFilename =
                "testDeGrooteFregly2016Muscle_timestepping";
        if (!ignoreActivationDynamics) iterateFilename += "_actdyn";
        if (ignoreTendonCompliance) iterateFilename += "_rigidtendon";
        iterateFilename += ".sto";
        iterateSim.write(iterateFilename);

        const double error = iterateSim.compareContinuousVariablesRMS(
                solutionTrajOpt, {{"states", {}}, {"controls", {}}});
        CHECK(error < 0.05);
    }

    // Track the kinematics from the trajectory optimization.
    // ------------------------------------------------------
    // We will try to recover muscle activity.
    {
        std::cout << "Tracking the trajectory optimization coordinate solution."
                  << std::endl;
        MocoTool moco;
        MocoProblem& problem = moco.updProblem();
        problem.setModelCopy(model);
        // Using an equality constraint for the time bounds was essential for
        // recovering the correct excitation.
        const double finalTime =
                solutionTrajOpt.getTime()[solutionTrajOpt.getNumTimes() - 1];
        const double slop = 0; // TODO 1e-4;
        problem.setTimeBounds(0 + slop, finalTime - slop);
        problem.setStateInfo("/joint/height/value",
                {0.10, 0.16}); // , initHeight, finalHeight);
        problem.setStateInfo("/joint/height/speed", {-10, 10}); // , 0, 0);
        if (!ignoreTendonCompliance) {
            // We would prefer to use a range of [0.2, 1.8] but then IPOPT
            // tries very small fiber lengths that cause tendon stretch to
            // be HUGE, causing insanely high tendon forces.
            problem.setStateInfo("/forceset/actuator/norm_fiber_length",
                    {0.8, 1.8},
                    model.getStateVariableValue(
                            state, "forceset/actuator/norm_fiber_length"));
        }
        // OpenSim might not allow activations of 0.
        if (!ignoreActivationDynamics) {
            problem.setStateInfo(
                    "/forceset/actuator/activation", {0.01, 1}, initActivation);
        }
        problem.setControlInfo("/forceset/actuator", {0.01, 1});

        auto* tracking = problem.addCost<MocoStateTrackingCost>();

        auto states = solutionTrajOpt.exportToStatesStorage().exportToTable();
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

        auto& solver = moco.initSolver<TestType>();
        solver.set_num_mesh_points(40);
        solver.set_dynamics_mode("implicit");

        MocoSolution solutionTrack = moco.solve();
        std::string solutionFilename =
                "testDeGrooteFregly2016Muscle_track_solution";
        if (!ignoreActivationDynamics) solutionFilename += "_actdyn";
        if (ignoreTendonCompliance) solutionFilename += "_rigidtendon";
        solutionFilename += ".sto";
        solutionTrack.write(solutionFilename);
        double error =
                solutionTrack.compareContinuousVariablesRMS(solutionTrajOpt);
        CHECK(error < 0.015);
    }
    // TODO: Support constraining initial fiber lengths to their equilibrium
    // lengths (in explicit mode).
}
