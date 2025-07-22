/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testFunctionBasedPath.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Nicholas Bianco                                                 *
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

#include <OpenSim/Simulation/Model/FunctionBasedPath.h>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/PolynomialFunction.h>
#include <OpenSim/Common/MultivariatePolynomialFunction.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Model/PathActuator.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using Catch::Matchers::WithinAbs;

TEST_CASE("testFunctionBasedPath") {
    
    const double q_x = 0.12;
    const double q_y = 0.34;
    const double qdot_x = 0.56;
    const double qdot_y = 0.78;
    const double tension = 0.91;
    
    SECTION("Sliding point mass, length function") {
        // 1-DOF polynomial path function.
        // length = q^3 + 2*q^2 + 3*q + 4
        PolynomialFunction poly(createVector({1.0, 2.0, 3.0, 4.0}));
        
        // Test values.
        // momentArm = -dl/dq = -3*q^2 - 4*q - 3
        // speed = -qdot * momentArm
        const double length = q_x*q_x*q_x + 2*q_x*q_x + 3*q_x + 4;
        const double momentArm = -3*q_x*q_x - 4*q_x - 3;
        const double speed = -qdot_x * momentArm;
        const double genForce = tension * momentArm;
        
        // Create a sliding mass model and add a PathActuator with a 1-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createSlidingPointMass();
        
        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_1dof");
        fbPath.setLengthFunction(poly);
        fbPath.setCoordinatePaths({"/slider/position"});
        
        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();
        
        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);
        
        // Run inverse dynamics to compute the generalized force applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(1, 0.0);
        matter.calcResidualForce(state, 
                createVector({0.0}), 
                SimTK::Vector_<SimTK::SpatialVec>(2, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);
        
        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& coord = model.getCoordinateSet()[0];
        CHECK_THAT(momentArm, 
            WithinAbs(path.computeMomentArm(state, coord), tol));
        CHECK_THAT(speed, 
            WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce, WithinAbs(residuals[0], tol));
    }
    
    SECTION("Planar point mass, length function") {
        // 2-DOF polynomial path function.
        // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction poly(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);
        
        // Test values.
        // momentArm_x = -dl/dq_x = -4 - 5*q_y - 12*q_x
        // momentArm_y = -dl/dq_y = -2 - 5*q_x - 6*q_y
        // speed = -qdot_x * momentArm_x - qdot_y * momentArm_y
        const double length = 1.0 + 2.0 * q_y + 3.0 * q_y * q_y + 4.0 * q_x + 
                              5.0 * q_x * q_y + 6.0 * q_x * q_x;
        const double momentArm_x = -4.0 - 5.0 * q_y - 12.0 * q_x;
        const double momentArm_y = -2.0 - 5.0 * q_x - 6.0 * q_y;
        const double speed = -qdot_x * momentArm_x - qdot_y * momentArm_y;
        const double genForce_x = tension * momentArm_x;
        const double genForce_y = tension * momentArm_y;
        
        // Create a planar point mass model and add a PathActuator with a 2-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createPlanarPointMass();
        model.setGravity(SimTK::Vec3(0.0));
        
        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_2dof");
        fbPath.setLengthFunction(poly);
        fbPath.setCoordinatePaths({"/jointset/tx/tx", "/jointset/ty/ty"});
        
        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();
        
        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[1].setValue(state, q_y);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.getCoordinateSet()[1].setSpeedValue(state, qdot_y);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);
        
        // Run inverse dynamics to compute the generalized forces applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(2, 0.0);
        matter.calcResidualForce(state, 
                SimTK::Vector(2, 0.0), 
                SimTK::Vector_<SimTK::SpatialVec>(3, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);
        
        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& tx = model.getCoordinateSet()[0];
        auto& ty = model.getCoordinateSet()[1];
        CHECK_THAT(momentArm_x, 
                WithinAbs(path.computeMomentArm(state, tx), tol));
        CHECK_THAT(momentArm_y, 
                WithinAbs(path.computeMomentArm(state, ty), tol));
        CHECK_THAT(speed, 
                WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce_x, WithinAbs(residuals[0], tol));
        CHECK_THAT(genForce_y, WithinAbs(residuals[1], tol));
    }
    
    SECTION("Planar point mass, all functions") {
        // 2-DOF polynomial path function.
        // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction lengthFunc(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);

        // Moment arm functions.
        // momentArm_x = -dl/dq_x = -4 - 5*q_y - 12*q_x
        // momentArm_y = -dl/dq_y = -2 - 6*q_y - 5*q_x
        MultivariatePolynomialFunction momentArmFunc_x(
                createVector({-4.0, -5.0, -12.0}), 2, 1);
        MultivariatePolynomialFunction momentArmFunc_y(
                createVector({-2.0, -6.0, -5.0}), 2, 1);

        // Speed function.
        // speed = -qdot_x * momentArm_x - qdot_y * momentArm_y
        //       = qdot_x * (4 + 5*q_y + 12*q_x) + qdot_y * (2 + 5*q_x + 6*q_y)
        //       = 4*qdot_x + 5*qdot_x*q_y + 12*qdot_x*q_x + 2*qdot_y +
        //         5*qdot_y*q_x + 6*qdot_y*q_y
        // 
        // See the documentation for MultivariatePolynomialFunction for an
        // explanation of the coefficients.
        SimTK::Vector speedCoeffs(15, 0.0);
        speedCoeffs[1] = 2.0;
        speedCoeffs[3] = 4.0;
        speedCoeffs[7] = 6.0;
        speedCoeffs[8] = 5.0;
        speedCoeffs[11] = 5.0;
        speedCoeffs[12] = 12.0;
        MultivariatePolynomialFunction speedFunc(speedCoeffs, 4, 2);

        // Test values.
        const double length = 1.0 + 2.0 * q_y + 3.0 * q_y * q_y + 4.0 * q_x + 
                              5.0 * q_x * q_y + 6.0 * q_x * q_x;
        const double momentArm_x = -4.0 - 5.0 * q_y - 12.0 * q_x;
        const double momentArm_y = -2.0 - 5.0 * q_x - 6.0 * q_y;
        const double speed = -qdot_x * momentArm_x - qdot_y * momentArm_y;
        const double genForce_x = tension * momentArm_x;
        const double genForce_y = tension * momentArm_y;

        // Create a planar point mass model and add a PathActuator with a 2-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createPlanarPointMass();
        model.setGravity(SimTK::Vec3(0.0));

        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_2dof");
        fbPath.setLengthFunction(lengthFunc);
        fbPath.appendMomentArmFunction(momentArmFunc_x);
        fbPath.appendMomentArmFunction(momentArmFunc_y);
        fbPath.setLengtheningSpeedFunction(speedFunc);
        fbPath.setCoordinatePaths({"/jointset/tx/tx", "/jointset/ty/ty"});

        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();

        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[1].setValue(state, q_y);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.getCoordinateSet()[1].setSpeedValue(state, qdot_y);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);

        // Run inverse dynamics to compute the generalized forces applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(2, 0.0);
        matter.calcResidualForce(state, 
                SimTK::Vector(2, 0.0), 
                SimTK::Vector_<SimTK::SpatialVec>(3, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);

        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& tx = model.getCoordinateSet()[0];
        auto& ty = model.getCoordinateSet()[1];
        CHECK_THAT(momentArm_x,
                WithinAbs(path.computeMomentArm(state, tx), tol));
        CHECK_THAT(momentArm_y, 
                WithinAbs(path.computeMomentArm(state, ty), tol));
        CHECK_THAT(speed, 
                WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce_x, WithinAbs(residuals[0], tol));
        CHECK_THAT(genForce_y, WithinAbs(residuals[1], tol));
    }

    SECTION("Planar point mass, MultivariatePolynoimalFunction helpers") {
        // 2-DOF polynomial path function.
        // length = 1 + 2*q_y + 3*q_y^2 + 4*q_x + 5*q_x*q_y + 6*q_x^2
        MultivariatePolynomialFunction lengthFunc(
                createVector({1.0, 2.0, 3.0, 4.0, 5.0, 6.0}), 2, 2);

        // Moment arm functions.
        // These functions are the first derivative with respect to the
        // corresponding coordinate of the length function. The coefficients are
        // negated to match the convention in OpenSim.
        bool negateCoefficients = true;
        MultivariatePolynomialFunction momentArmFunc_x = 
            lengthFunc.generateDerivativeFunction(0, negateCoefficients);
        MultivariatePolynomialFunction momentArmFunc_y =
            lengthFunc.generateDerivativeFunction(1, negateCoefficients);

        // Speed function.
        // The lengthening speed function is the time derivative of the length
        // length function, which be computed by 
        MultivariatePolynomialFunction speedFunc = 
                lengthFunc.generatePartialVelocityFunction();

        // Test values.
        const double length = 1.0 + 2.0 * q_y + 3.0 * q_y * q_y + 4.0 * q_x + 
                              5.0 * q_x * q_y + 6.0 * q_x * q_x;
        const double momentArm_x = -4.0 - 5.0 * q_y - 12.0 * q_x;
        const double momentArm_y = -2.0 - 5.0 * q_x - 6.0 * q_y;
        const double speed = -qdot_x * momentArm_x - qdot_y * momentArm_y;
        const double genForce_x = tension * momentArm_x;
        const double genForce_y = tension * momentArm_y;

        // Create a planar point mass model and add a PathActuator with a 2-DOF
        // FunctionBasedPath.
        Model model = ModelFactory::createPlanarPointMass();
        model.setGravity(SimTK::Vec3(0.0));

        FunctionBasedPath fbPath;
        fbPath.setName("polynomial_path_2dof");
        fbPath.setLengthFunction(lengthFunc);
        fbPath.appendMomentArmFunction(momentArmFunc_x);
        fbPath.appendMomentArmFunction(momentArmFunc_y);
        fbPath.setLengtheningSpeedFunction(speedFunc);
        fbPath.setCoordinatePaths({"/jointset/tx/tx", "/jointset/ty/ty"});

        auto* actu = new PathActuator();
        actu->set_path(fbPath);
        actu->setName("actuator");
        actu->setOptimalForce(1);
        model.addComponent(actu);
        model.finalizeConnections();

        // Initialize the system and set the state and controls.
        SimTK::State state = model.initSystem();
        model.getCoordinateSet()[0].setValue(state, q_x);
        model.getCoordinateSet()[1].setValue(state, q_y);
        model.getCoordinateSet()[0].setSpeedValue(state, qdot_x);
        model.getCoordinateSet()[1].setSpeedValue(state, qdot_y);
        model.setControls(state, createVector({tension, 0.0, 0.0}));
        model.realizeAcceleration(state);

        // Run inverse dynamics to compute the generalized forces applied by the
        // PathActuator.
        auto& matter = model.updMatterSubsystem();
        SimTK::Vector residuals(2, 0.0);
        matter.calcResidualForce(state, 
                SimTK::Vector(2, 0.0), 
                SimTK::Vector_<SimTK::SpatialVec>(3, 
                    SimTK::SpatialVec(SimTK::Vec3(0), SimTK::Vec3(0))),
                state.getUDot(), 
                SimTK::Vector(0),
                residuals);

        // Check that the length, moment arms, speed, and generalized forces are
        // correct. Compare quantities that should have been calculated to 
        // machine tolerance given the problem size, which we'll characterize by 
        // the number of mobilities (based on Simbody's testing).
        const auto& path = actu->getPath();
        const double tol = 10 * state.getNU() * SimTK::Test::defTol<double>();
        CHECK_THAT(length, WithinAbs(path.getLength(state), tol));
        auto& tx = model.getCoordinateSet()[0];
        auto& ty = model.getCoordinateSet()[1];
        CHECK_THAT(momentArm_x,
                WithinAbs(path.computeMomentArm(state, tx), tol));
        CHECK_THAT(momentArm_y, 
                WithinAbs(path.computeMomentArm(state, ty), tol));
        CHECK_THAT(speed, 
                WithinAbs(path.getLengtheningSpeed(state), tol));
        CHECK_THAT(genForce_x, WithinAbs(residuals[0], tol));
        CHECK_THAT(genForce_y, WithinAbs(residuals[1], tol));
    }

}
