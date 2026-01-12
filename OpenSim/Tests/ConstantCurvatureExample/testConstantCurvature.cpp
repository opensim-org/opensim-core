/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testConstantCurvature.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
 * Author(s): Keenon Werling                                                  *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/* These tests cover checking the analytical Jacobians and gradients of
 * Jacobians for the ConstantCurvatureJoint. The ground-truth Jacobians were
 * computed using finite differences in Nimble, and then converted into
 * hard-coded C++ by hand.
 */

#include "OpenSim/Actuators/SpringGeneralizedForce.h"
#include "OpenSim/Auxiliary/auxiliaryTestFunctions.h"
#include "OpenSim/Common/Reporter.h"
#include "OpenSim/Simulation/Model/Model.h"
#include "OpenSim/Simulation/Model/Geometry.h"
#include "OpenSim/Simulation/SimbodyEngine/ConstantCurvatureJoint.h"
#include "OpenSim/Simulation/SimulationUtilities.h"
#include <string>
// #define VISUALIZE

#include <catch2/catch_all.hpp>

namespace {

using namespace SimTK;
using namespace OpenSim;

void testJacobians1() {
    using namespace SimTK;

    double d = 0.2;
    Vec3 q(0.5, 0.5, 0.5);
    Vec3 qDot(0.1, 0.2, 0.3);

    ////////////////////////////////////////////////////////
    // Test the ordinary Jacobian
    ////////////////////////////////////////////////////////

    // Jacobian:
    //  0.7701511529 -0.4794255386  0.0000000000
    // -0.4794255386  0.0000000000  1.0000000000
    //  0.4207354924  0.8775825619  0.0000000000
    // -0.0388101162 -0.0864079951  0.0000000000
    //  0.0148489243  0.0148489243  0.0000000000
    //  0.0784030482 -0.0517875648  0.0000000000
    Mat63 expectedJacobian;
    expectedJacobian.setToZero();
    expectedJacobian(0, 0) = 0.7701511529;
    expectedJacobian(0, 1) = -0.4794255386;
    expectedJacobian(0, 2) = 0.0000000000;
    expectedJacobian(1, 0) = -0.4794255386;
    expectedJacobian(1, 1) = 0.0000000000;
    expectedJacobian(1, 2) = 1.0000000000;
    expectedJacobian(2, 0) = 0.4207354924;
    expectedJacobian(2, 1) = 0.8775825619;
    expectedJacobian(2, 2) = 0.0000000000;
    expectedJacobian(3, 0) = -0.0388101162;
    expectedJacobian(3, 1) = -0.0864079951;
    expectedJacobian(3, 2) = 0.0000000000;
    expectedJacobian(4, 0) = 0.0148489243;
    expectedJacobian(4, 1) = 0.0148489243;
    expectedJacobian(4, 2) = 0.0000000000;
    expectedJacobian(5, 0) = 0.0784030482;
    expectedJacobian(5, 1) = -0.0517875648;
    expectedJacobian(5, 2) = 0.0000000000;

    Mat63 J = ConstantCurvatureJoint::getConstantCurveJacobian(q, d);

    Mat63 diffJ = J - expectedJacobian;
    ASSERT(diffJ.norm() < 1e-9, __FILE__, __LINE__,
            "Jacobian didn't match expected value");

    ////////////////////////////////////////////////////////
    // Test the time derivative of the Jacobian
    ////////////////////////////////////////////////////////

    // Jacobian:
    // -0.2103677462 -0.2632747686  0.0000000000
    // -0.1755165124  0.0000000000  0.0000000000
    //  0.1850755765 -0.1438276616  0.0000000000
    // -0.0175753539  0.0167370896  0.0000000000
    //  0.0015923415  0.0051216193  0.0000000000
    // -0.0189190608 -0.0280126345  0.0000000000
    Mat63 expectedJacobianTimeDeriv;
    expectedJacobianTimeDeriv.setToZero();
    expectedJacobianTimeDeriv(0, 0) = -0.2103677462;
    expectedJacobianTimeDeriv(0, 1) = -0.2632747686;
    expectedJacobianTimeDeriv(0, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(1, 0) = -0.1755165124;
    expectedJacobianTimeDeriv(1, 1) = 0.0000000000;
    expectedJacobianTimeDeriv(1, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(2, 0) = 0.1850755765;
    expectedJacobianTimeDeriv(2, 1) = -0.1438276616;
    expectedJacobianTimeDeriv(2, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(3, 0) = -0.0175753539;
    expectedJacobianTimeDeriv(3, 1) = 0.0167370896;
    expectedJacobianTimeDeriv(3, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(4, 0) = 0.0015923415;
    expectedJacobianTimeDeriv(4, 1) = 0.0051216193;
    expectedJacobianTimeDeriv(4, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(5, 0) = -0.0189190608;
    expectedJacobianTimeDeriv(5, 1) = -0.0280126345;
    expectedJacobianTimeDeriv(5, 2) = 0.0000000000;

    Mat63 Jdot = ConstantCurvatureJoint::getConstantCurveJacobianDerivWrtTime(
            q, qDot, d);

    Mat63 diffJdot = Jdot - expectedJacobianTimeDeriv;
    ASSERT(diffJdot.norm() < 1e-9, __FILE__, __LINE__,
            "Jacobian time deriv didn't match expected value");
}

void testJacobians2() {
    using namespace SimTK;

    double d = 1.0;
    Vec3 q(0.50464766, 0.55085949, 0.0018320994);
    Vec3 qDot(0.44693263, 0.76950436, 0.0065713527);

    ////////////////////////////////////////////////////////
    // Test the ordinary Jacobian
    ////////////////////////////////////////////////////////

    // Jacobian:
    //  0.8520735327 -0.0018320984  0.0000000000
    // -0.5234197721  0.0000000000  1.0000000000
    //  0.0015610852  0.9999983217  0.0000000000
    //  0.0184228695 -0.5019869761  0.0000000000
    //  0.0731720241  0.0813768660  0.0000000000
    //  0.4257335679 -0.0234562723  0.0000000000
    Mat63 expectedJacobian;
    expectedJacobian.setToZero();
    expectedJacobian(0, 0) = 0.8520735327;
    expectedJacobian(0, 1) = -0.0018320984;
    expectedJacobian(0, 2) = 0.0000000000;
    expectedJacobian(1, 0) = -0.5234197721;
    expectedJacobian(1, 1) = 0.0000000000;
    expectedJacobian(1, 2) = 1.0000000000;
    expectedJacobian(2, 0) = 0.0015610852;
    expectedJacobian(2, 1) = 0.9999983217;
    expectedJacobian(2, 2) = 0.0000000000;
    expectedJacobian(3, 0) = 0.0184228695;
    expectedJacobian(3, 1) = -0.5019869761;
    expectedJacobian(3, 2) = 0.0000000000;
    expectedJacobian(4, 0) = 0.0731720241;
    expectedJacobian(4, 1) = 0.0813768660;
    expectedJacobian(4, 2) = 0.0000000000;
    expectedJacobian(5, 0) = 0.4257335679;
    expectedJacobian(5, 1) = -0.0234562723;
    expectedJacobian(5, 2) = 0.0000000000;

    Mat63 J = ConstantCurvatureJoint::getConstantCurveJacobian(q, d);

    Mat63 diffJ = J - expectedJacobian;
    ASSERT(diffJ.norm() < 1e-9, __FILE__, __LINE__,
            "Jacobian didn't match expected value");

    ////////////////////////////////////////////////////////
    // Test the time derivative of the Jacobian
    ////////////////////////////////////////////////////////

    // Jacobian:
    // -0.4027833792 -0.0065713417  0.0000000000
    // -0.6556753989  0.0000000000  0.0000000000
    //  0.0048613545 -0.0000120394  0.0000000000
    //  0.0320114385 -0.0017449897  0.0000000000
    //  0.0351757892  0.0935909521  0.0000000000
    // -0.1964663065 -0.0543846437  0.0000000000
    Mat63 expectedJacobianTimeDeriv;
    expectedJacobianTimeDeriv.setToZero();
    expectedJacobianTimeDeriv(0, 0) = -0.4027833792;
    expectedJacobianTimeDeriv(0, 1) = -0.0065713417;
    expectedJacobianTimeDeriv(0, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(1, 0) = -0.6556753989;
    expectedJacobianTimeDeriv(1, 1) = 0.0000000000;
    expectedJacobianTimeDeriv(1, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(2, 0) = 0.0048613545;
    expectedJacobianTimeDeriv(2, 1) = -0.0000120394;
    expectedJacobianTimeDeriv(2, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(3, 0) = 0.0320114385;
    expectedJacobianTimeDeriv(3, 1) = -0.0017449897;
    expectedJacobianTimeDeriv(3, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(4, 0) = 0.0351757892;
    expectedJacobianTimeDeriv(4, 1) = 0.0935909521;
    expectedJacobianTimeDeriv(4, 2) = 0.0000000000;
    expectedJacobianTimeDeriv(5, 0) = -0.1964663065;
    expectedJacobianTimeDeriv(5, 1) = -0.0543846437;
    expectedJacobianTimeDeriv(5, 2) = 0.0000000000;

    Mat63 Jdot = ConstantCurvatureJoint::getConstantCurveJacobianDerivWrtTime(
            q, qDot, d);

    Mat63 diffJdot = Jdot - expectedJacobianTimeDeriv;
    ASSERT(diffJdot.norm() < 1e-9, __FILE__, __LINE__,
            "Jacobian time deriv didn't match expected value");
}

void testJacobians3() {
    using namespace SimTK;

    double d = 1.0;

    Vec3 q(0.79846287622439227, 1.5707963210265892, -0.015968884371590844);
    // Check that this doesn't fire any asserts or crash
    Mat63 J = ConstantCurvatureJoint::getConstantCurveJacobian(q, d);
    (void)J; // keep compiler from complaining
}

}

TEST_CASE("testConstantCurvature") {
    testJacobians1();
    testJacobians2();
    testJacobians3();

    Model model;
    model.setName("spring-stack");
#ifdef VISUALIZE
    model.setUseVisualizer(true);
#endif

    std::vector<OpenSim::Body*> bodies;
    std::vector<OpenSim::Joint*> joints;

    const int numBodies = 3;

    double springStiffness = 28.0;
    double springViscosity = 2.0;

    // Create display geometry.
    Brick bodyGeometry(Vec3(0.1, 0.1, 0.1));
    bodyGeometry.setColor(Gray);

    // Add a console reporter to print the muscle fiber force and elbow angle.
    ConsoleReporter* reporter = new ConsoleReporter();
    reporter->set_report_time_interval(1.0);

    for (int i = 0; i < numBodies; i++) {
        OpenSim::Body* body = new OpenSim::Body("body_" + std::to_string(i), 1,
                Vec3(0), Inertia(0.1, 0.1, 0.1));

        model.addBody(body);
        bodies.push_back(body);

        // Attach an ellipsoid to a frame located at the center of each body.
        PhysicalOffsetFrame* visualCenter =
                new PhysicalOffsetFrame("body_viz_center_" + std::to_string(i),
                        *body, Transform(Vec3(0)));
        body->addComponent(visualCenter);
        visualCenter->attachGeometry(bodyGeometry.clone());

        // Connect the bodies with pin joints. Assume each body is 1 m long.
        ConstantCurvatureJoint* joint;
        if (i > 0) {
            joint = new ConstantCurvatureJoint("ccj" + std::to_string(i),
                    *(bodies[i - 1]), *(bodies[i]), Vec3(0, 0, 0), 1.0);
        } else {
            joint = new ConstantCurvatureJoint("ccj" + std::to_string(i),
                    model.getGround(), *(bodies[i]), Vec3(0, 0, 0), 1.0);
        }
        joint->updCoordinate(ConstantCurvatureJoint::Coord::RotationX)
                .setName("joint_" + std::to_string(i) + "_x");
        joint->updCoordinate(ConstantCurvatureJoint::Coord::RotationZ)
                .setName("joint_" + std::to_string(i) + "_z");
        joint->updCoordinate(ConstantCurvatureJoint::Coord::RotationY)
                .setName("joint_" + std::to_string(i) + "_y");

        SpringGeneralizedForce* springForceX =
                new SpringGeneralizedForce("joint_" + std::to_string(i) + "_x");
        springForceX->setStiffness(springStiffness);
        springForceX->setViscosity(springViscosity);
        springForceX->setRestLength(0);
        model.addForce(springForceX);
        SpringGeneralizedForce* springForceZ =
                new SpringGeneralizedForce("joint_" + std::to_string(i) + "_z");
        springForceZ->setStiffness(springStiffness);
        springForceZ->setViscosity(springViscosity);
        springForceZ->setRestLength(0);
        model.addForce(springForceZ);
        SpringGeneralizedForce* springForceY =
                new SpringGeneralizedForce("joint_" + std::to_string(i) + "_y");
        springForceY->setStiffness(springStiffness);
        springForceY->setViscosity(springViscosity);
        springForceY->setRestLength(0);
        model.addForce(springForceY);

        model.addJoint(joint);
        joints.push_back(joint);
    }

    reporter->addToReport(
            static_cast<ConstantCurvatureJoint*>(joints[0])
                    ->getCoordinate(ConstantCurvatureJoint::Coord::RotationX)
                    .getOutput("value"),
            "X");
    reporter->addToReport(
            static_cast<ConstantCurvatureJoint*>(joints[0])
                    ->getCoordinate(ConstantCurvatureJoint::Coord::RotationZ)
                    .getOutput("value"),
            "Z");
    reporter->addToReport(
            static_cast<ConstantCurvatureJoint*>(joints[0])
                    ->getCoordinate(ConstantCurvatureJoint::Coord::RotationY)
                    .getOutput("value"),
            "Y");
    reporter->addToReport(
            static_cast<ConstantCurvatureJoint*>(joints[0])
                    ->getCoordinate(ConstantCurvatureJoint::Coord::RotationX)
                    .getOutput("speed"),
            "vX");
    reporter->addToReport(
            static_cast<ConstantCurvatureJoint*>(joints[0])
                    ->getCoordinate(ConstantCurvatureJoint::Coord::RotationZ)
                    .getOutput("speed"),
            "vZ");
    reporter->addToReport(
            static_cast<ConstantCurvatureJoint*>(joints[0])
                    ->getCoordinate(ConstantCurvatureJoint::Coord::RotationY)
                    .getOutput("speed"),
            "vY");

    model.addComponent(reporter);

    // Configure the model.
    State& state = model.initSystem();

    static_cast<ConstantCurvatureJoint*>(joints[0])
            ->getCoordinate(ConstantCurvatureJoint::Coord::RotationZ)
            .setValue(state, 0.01);
    // static_cast<ConstantCurvatureJoint*>(joints[0])->getCoordinate(
    //      ConstantCurvatureJoint::Coord::RotationX).setValue(state,
    //      0.001);

    // Configure the visualizer.
#ifdef VISUALIZE
    model.updMatterSubsystem().setShowDefaultGeometry(true);
    Visualizer& viz = model.updVisualizer().updSimbodyVisualizer();
    viz.setBackgroundType(viz.SolidColor);
    viz.setBackgroundColor(White);
#endif

    // Simulate.
    simulate(model, state, 20.0);
}
