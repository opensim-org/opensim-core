/* -------------------------------------------------------------------------- *
 * OpenSim Moco: testPolynomialApproximation.cpp                              *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017-19 Stanford University and the Authors                  *
 *                                                                            *
 * Author(s): Antoine Falisse                                                 *
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

#define CATCH_CONFIG_MAIN
#include "../../Test/Testing.h"
#include <Moco/osimMoco.h>

using namespace OpenSim;

// Create a single leg 2D model with two muscles to test the estimation of
// muscle-tendon lengths, lengthening speeds, and moment arms based on
// polynomial approximation of joint coordinates. We optimized the polynomial
// coefficients using custom MATLAB code to fit muscle-tendon lengths and
// moment arms (maximal root mean square deviation: 3 mm) obtained from OpenSim
// using a wide range of coordinate values.
Model createModel() {
    using SimTK::Vec3;
    using SimTK::Inertia;
    // Create Model
    Model model;
    model.setName("singleLeg2DModel");
    // Add bodies
    auto* pelvis = new OpenSim::Body("pelvis", 9.7143336091724,
        Vec3(-0.0682778, 0, 0), Inertia(0.0814928846050306, 0.0814928846050306,
        0.0445427591530667, 0, 0, 0));
    model.addBody(pelvis);

    auto* femur = new OpenSim::Body("femur", 7.67231915023828,
        Vec3(0, -0.170467, 0), Inertia(0.111055472890139, 0.0291116288158616,
        0.117110028170931, 0, 0, 0));
    model.addBody(femur);

    auto* tibia = new OpenSim::Body("tibia", 3.05815503574821,
        Vec3(0, -0.180489, 0), Inertia(0.0388526996597354, 0.00393152317985418,
        0.0393923204883429, 0, 0, 0));
    model.addBody(tibia);
    // Add joints
    auto* ground_pelvis = new PlanarJoint("ground_pelvis", model.getGround(),
        Vec3(0), Vec3(0), *pelvis, Vec3(0), Vec3(0));
    auto& pelvis_tilt =
        ground_pelvis->updCoordinate(PlanarJoint::Coord::RotationZ);
    pelvis_tilt.setRangeMin(-1.570796327);
    pelvis_tilt.setRangeMax(1.570796327);
    pelvis_tilt.setName("pelvis_tilt");
    auto& pelvis_tx =
        ground_pelvis->updCoordinate(PlanarJoint::Coord::TranslationX);
    pelvis_tx.setRangeMin(-5);
    pelvis_tx.setRangeMax(5);
    pelvis_tx.setName("pelvis_tx");
    auto& pelvis_ty =
        ground_pelvis->updCoordinate(PlanarJoint::Coord::TranslationY);
    pelvis_ty.setRangeMin(-1);
    pelvis_ty.setRangeMax(2);
    pelvis_ty.setName("pelvis_ty");
    model.addJoint(ground_pelvis);

    auto* hip = new PinJoint("hip", *pelvis, Vec3(-0.0682778001711179,
        -0.0638353973311301, 0.0823306940058688), Vec3(0), *femur, Vec3(0),
        Vec3(0));
    auto& hip_flexion = hip->updCoordinate();
    hip_flexion.setRangeMin(-1.221730476);
    hip_flexion.setRangeMax(1.570796327);
    hip_flexion.setName("hip_flexion");
    model.addJoint(hip);

    auto* knee = new PinJoint("knee", *femur, Vec3(-0.00451221232146798,
        -0.396907245921447, 0), Vec3(0), *tibia, Vec3(0), Vec3(0));
    auto& knee_angle = knee->updCoordinate();
    knee_angle.setRangeMin(-2.094395102);
    knee_angle.setRangeMax(0.174532925);
    knee_angle.setName("knee_angle");
    model.addJoint(knee);
    // Add muscles
    int dimHamstrings = 2;
    int orderHamstrings = 3;
    const int nCoeffHamstrings = 10;
    double coeffHamstrings[nCoeffHamstrings] = {0.420495248805280,
        0.028755254940480, -0.018300167519638, -0.006995443639179,
        0.053690588126276, 0.002092858497112, 0.001164874424056,
        0.016799961996946, 1.886801073680048e-04, -0.009576337870147};
    auto* hamstrings_f = new MultivariatePolynomialFunction();
    hamstrings_f->setDimension(dimHamstrings);
    hamstrings_f->setOrder(orderHamstrings);
    hamstrings_f->setCoefficients(
            SimTK::Vector(nCoeffHamstrings,coeffHamstrings));
    auto* hamstrings = new DeGrooteFregly2016Muscle();
    auto& hamstrings_p = hamstrings->updGeometryPath();
    hamstrings_p.set_use_approximation(true);
    hamstrings_p.append_length_approximation(*hamstrings_f);
    hamstrings_p.append_approximation_coordinates("/jointset/hip/hip_flexion");
    hamstrings_p.append_approximation_coordinates("/jointset/knee/knee_angle");
    hamstrings->set_ignore_tendon_compliance(true);
    hamstrings->addNewPathPoint("hamstrings-P1", *pelvis,
            Vec3(-0.121645, -0.0990559, 0.0684676));
    hamstrings->addNewPathPoint("hamstrings-P2", *tibia,
            Vec3(-0.0290986, -0.0348024, 0.0284509));
    hamstrings->addNewPathPoint("hamstrings-P3", *tibia,
            Vec3(-0.0226215, -0.054427, 0.0331589));
    hamstrings->setName("hamstrings");
    model.addComponent(hamstrings);

    int dimRF = 2;
    int orderRF = 5;
    const int nCoeffRF = 21;
    double coeffRF[nCoeffRF] = {0.411041216316924, -0.041901285727185,
        0.015455075788113, 0.035343831737396, 0.019052568915166,
        0.003490516739746, -0.041198556780086, -0.002495169893594,
        -0.001467645187366, -8.011395917540590e-04, -2.062908241814378e-04,
        -0.014671630141187, 0.001472708873289, 4.702571981803344e-04,
        2.278861920386511e-05, 0.005544930296365, 2.921882702941324e-04,
        6.068963635841811e-05, 0.001214860648500, -4.506103418022895e-05,
        -1.110736612700566e-04};
    auto* RF_f = new MultivariatePolynomialFunction();
    RF_f->setDimension(dimRF);
    RF_f->setOrder(orderRF);
    RF_f->setCoefficients(SimTK::Vector(nCoeffRF,coeffRF));
    auto* RF = new DeGrooteFregly2016Muscle();
    auto& RF_p = RF->updGeometryPath();
    RF_p.set_use_approximation(true);
    RF_p.append_length_approximation(*RF_f);
    RF_p.append_approximation_coordinates("/jointset/hip/hip_flexion");
    RF_p.append_approximation_coordinates("/jointset/knee/knee_angle");
    RF->set_ignore_tendon_compliance(true);
    RF->addNewPathPoint("RF-P1", *pelvis,
            Vec3(-0.0284893, -0.0300345, 0.0954444));
    ConditionalPathPoint* RF_P2 = new ConditionalPathPoint();
    RF_P2->setName("RF-P2");
    RF_P2->setCoordinate(knee_angle);
    RF_P2->setParentFrame(*femur);
    RF_P2->setLocation(Vec3(0.0334917, -0.404106, 0.00190522));
    RF_P2->setRangeMin(-2.61799);
    RF_P2->setRangeMax(-1.45997);
    RF->updGeometryPath().updPathPointSet().adoptAndAppend(RF_P2);
    OpenSim::MovingPathPoint* RF_P3 = new MovingPathPoint();
    RF_P3->setName("RF-P3");
    RF_P3->setParentFrame(*tibia);
    double scale = 0.966732152034662;
    int np_RFX = 17;
    double RFX_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472,
            -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453,
            0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
    double RFX_y[] = { 0.0155805, 0.0179938, 0.0275081, 0.0296564, 0.0307615,
            0.0365695, 0.0422074, 0.0450902, 0.048391, 0.0534299, 0.0617576,
            0.0617669, 0.0617762, 0.0633083, 0.066994, 0.0733035, 0.0573481 };
    SimTK::Vector RFX_y_s(np_RFX, &RFX_y[0]);
    RFX_y_s = scale * RFX_y_s;
    SimmSpline SS_RF_x(np_RFX, RFX_x, &RFX_y_s[0]);
    RF_P3->set_x_location(SS_RF_x);
    RF_P3->setXCoordinate(knee_angle);
    int np_RFY = 17;
    double RFY_x[] = { -2.0944, -1.99997, -1.5708, -1.45752, -1.39626, -1.0472,
            -0.698132, -0.526391, -0.349066, -0.174533, 0, 0.00017453,
            0.00034907, 0.0279253, 0.0872665, 0.174533, 2.0944 };
    double RFY_y[] = { 0.0234116, 0.0237613, 0.0251141, 0.0252795, 0.0253146,
            0.0249184, 0.0242373, 0.0238447, 0.0234197, 0.0227644, 0.020984,
            0.0209814, 0.0209788, 0.0205225, 0.0191754, 0.0159554,
            -0.0673774 };
    SimTK::Vector RFY_y_s(np_RFY, &RFY_y[0]);
    RFY_y_s = scale * RFY_y_s;
    SimmSpline SS_RF_y(np_RFY, RFY_x, &RFY_y_s[0]);
    RF_P3->set_y_location(SS_RF_y);
    RF_P3->setYCoordinate(knee_angle);
    int np_RFZ = 2;
    double RFZ_x[] = { -2.0944, 0.1745 };
    double RFZ_y[] = { 0.0014, 0.0014 };
    SimTK::Vector RFZ_y_s(np_RFZ, &RFZ_y[0]);
    RFZ_y_s = scale * RFZ_y_s;
    SimmSpline SS_RF_z(np_RFZ, RFZ_x, &RFZ_y_s[0]);
    RF_P3->set_z_location(SS_RF_z);
    RF_P3->setZCoordinate(knee_angle);
    RF->updGeometryPath().updPathPointSet().adoptAndAppend(RF_P3);
    RF->setName("RF");
    model.addComponent(RF);

    model.finalizeConnections();

    return model;
}

// Muscle-tendon lengths and lengthening speeds should be the same as those
// obtained wih original MATLAB code (hard coded values).
void testPolynomialApproximationImpl() {

    Model model(createModel());
    SimTK::State& state = model.initSystem();
    // Set non-null values hip_flexion and knee_angle speeds to have non-null
    // muscle-tendon lengthening speeds
    model.getCoordinateSet().get("hip_flexion").setSpeedValue(state,-1);
    model.getCoordinateSet().get("knee_angle").setSpeedValue(state,1);
    model.realizeVelocity(state);

    double hamstrings_lmt = model.getComponent<PathActuator>(
                "hamstrings").getGeometryPath().getLength(state);
    double hamstrings_vmt = model.getComponent<PathActuator>(
                "hamstrings").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.420495248805280 == hamstrings_lmt);
    CHECK(-0.024935333185797 == Approx(hamstrings_vmt).margin(1e-10));

    double RF_lmt = model.getComponent<PathActuator>(
                "RF").getGeometryPath().getLength(state);
    double RF_vmt = model.getComponent<PathActuator>(
                "RF").getGeometryPath().getLengtheningSpeed(state);
    CHECK(0.411041216316924 == RF_lmt);
    CHECK(-7.027289470986353e-04 == Approx(RF_vmt).margin(1e-10));
}

// Muscle-tendon lengths, lengthening speeds, and moment arms obtained with
// polynomial approximations should closely match those computed from the
// geometry paths.
void testPolynomialApproximation() {

    Model model(createModel());
    model.initSystem();

    // Lengths
    auto& hamstringsPath = model.getComponent<PathActuator>(
            "hamstrings").getGeometryPath();
    double hamstrings_lengthError =
            hamstringsPath.computeApproximationErrorOnGrid(40, "length");
    CHECK(hamstrings_lengthError < 3e-3); // allowed difference: 3 mm
    auto& RFPath = model.getComponent<PathActuator>("RF").getGeometryPath();
    double RF_lengthError =
            RFPath.computeApproximationErrorOnGrid(40, "length");
    CHECK(RF_lengthError < 3e-3); // allowed difference: 3 mm

    // Lengthening speeds
    double hamstrings_lengtheningSpeedError =
            hamstringsPath.computeApproximationErrorOnGrid(40,
                    "lengthening_speed");
    CHECK(hamstrings_lengtheningSpeedError < 3e-3); // allowed difference: 3mm/s
    double RF_lengtheningSpeedError =
            RFPath.computeApproximationErrorOnGrid(40, "lengthening_speed");
    CHECK(RF_lengtheningSpeedError < 3e-3); // allowed difference: 3mm/s

    // Moment arms
    const auto& hip_flexion = model.getCoordinateSet().get("hip_flexion");
    const auto& knee_angle = model.getCoordinateSet().get("knee_angle");
    double hamstrings_momArmErrorHip =
            hamstringsPath.computeApproximationErrorOnGrid(40,
                    "moment_arm", &hip_flexion);
    double hamstrings_momArmErrorKnee =
            hamstringsPath.computeApproximationErrorOnGrid(40,
                    "moment_arm", &knee_angle);
    CHECK(hamstrings_momArmErrorHip < 3e-3); // allowed difference: 3 mm
    CHECK(hamstrings_momArmErrorKnee < 3e-3); // allowed difference: 3 mm
    double RF_momArmErrorHip =
            RFPath.computeApproximationErrorOnGrid(40,"moment_arm",
                    &hip_flexion);
    double RF_momArmErrorKnee =
            RFPath.computeApproximationErrorOnGrid(40,"moment_arm",
                    &knee_angle);
     CHECK(RF_momArmErrorHip < 3e-3); // allowed difference: 3 mm
     CHECK(RF_momArmErrorKnee < 3e-3); // allowed difference: 3 mm
}

TEST_CASE("testPolynomialApproximation") {
    testPolynomialApproximationImpl();
    testPolynomialApproximation();
}
