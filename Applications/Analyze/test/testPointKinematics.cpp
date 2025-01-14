/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testPointKinematics.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 Stanford University and the Authors                *
 * Author(s): Alexander Beattie                                               *
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

// INCLUDE
#include <limits>
#include <iterator> // std::next

#include <OpenSim/Analyses/PointKinematics.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/AnalyzeTool.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

constexpr double tol = std::numeric_limits<double>::epsilon() * 10;

TEST_CASE("PointKinematics Construction with Blank Model", "[PointKinematics]") {
    Model model;
    PointKinematics pk(&model);

    REQUIRE(pk.getBody() == nullptr);
    REQUIRE(pk.getRelativeToBody() == nullptr);
    REQUIRE(pk.getPointName() == "NONAME");
}

TEST_CASE("PointKinematics Construction with No Model", "[PointKinematics]") {
    PointKinematics pk;

    REQUIRE(pk.getBody() == nullptr);
    REQUIRE(pk.getRelativeToBody() == nullptr);
    REQUIRE(pk.getPointName() == "NONAME");
}

TEST_CASE("PointKinematics Set and Get Body", "[PointKinematics]") {
    Model model("SinglePin.osim");
    const auto &bodies = model.getComponentList<Body>();
    const auto bodyIt = bodies.begin();
    const auto body = *bodyIt;

    PointKinematics pk(&model);
    pk.setBody(&body);

    REQUIRE(pk.getBody() == &body);
    REQUIRE(pk.getBody()->getName() == body.getName());
}

TEST_CASE("PointKinematics Set and Get RelativeToBody", "[PointKinematics]") {
    Model model("subject01.osim");
    const auto &bodies = model.getComponentList<Body>();
    // Gets the 2nd body (which is at index 1)
    const auto bodyIt = std::next(bodies.begin(),1);
    const auto body = *bodyIt;

    PointKinematics pk(&model);
    pk.setRelativeToBody(&body);

    REQUIRE(pk.getRelativeToBody() == &body);
    REQUIRE(pk.getRelativeToBody()->getName() == body.getName());
}

TEST_CASE("PointKinematics Set and Get Point", "[PointKinematics]") {
    Model model;
    PointKinematics pk(&model);
    SimTK::Vec3 point(1.0, 2.0, 3.0);
    
    pk.setPoint(point);
    SimTK::Vec3 retrievedPoint;
    pk.getPoint(retrievedPoint);

    REQUIRE_THAT(retrievedPoint[0], Catch::Matchers::WithinAbs(1.0,tol));
    REQUIRE_THAT(retrievedPoint[1], Catch::Matchers::WithinAbs(2.0,tol));
    REQUIRE_THAT(retrievedPoint[2], Catch::Matchers::WithinAbs(3.0,tol));
}

TEST_CASE("PointKinematics Record Kinematics Single Body", "[PointKinematics]") {
    std::string model_name = "subject01.osim";
    std::string coordinates_file_name = "subject01_walk1_ik.mot";

    Model model(model_name);
    model.initSystem();

    AnalyzeTool analyzeTool;
    analyzeTool.setName("test_analysis");
    analyzeTool.setModel(model);
    analyzeTool.setModelFilename(model_name);
    analyzeTool.setCoordinatesFileName(coordinates_file_name);
    analyzeTool.setLowpassCutoffFrequency(-1);
    analyzeTool.setResultsDir("results");

    const auto &bodies = model.getComponentList<Body>();
    const auto bodyIt = bodies.begin();
    const auto body1 = *bodyIt;
    // Gets the 2nd body (which is at index 1)
    const auto bodyIt2 = std::next(bodies.begin(),1);
    const auto body2 = *bodyIt2;

    PointKinematics pk(&model);
    pk.setBody(&body1);
    pk.setRelativeToBody(&body2);
    pk.setPoint(SimTK::Vec3(1.0, 2.0, 3.0));
    analyzeTool.updAnalysisSet().cloneAndAppend(pk);

    std::string output_file_name = "test_analysis_point_kinematics_1.xml";
    analyzeTool.print(output_file_name);
    AnalyzeTool roundTrip(output_file_name);
    roundTrip.run();
}

TEST_CASE("PointKinematics Record Kinematics Whole Body", "[PointKinematics]") {
    std::string model_name = "subject01.osim";
    std::string coordinates_file_name = "subject01_walk1_ik.mot";

    Model model(model_name);
    model.initSystem();

    AnalyzeTool analyzeTool;
    analyzeTool.setName("test_analysis");
    analyzeTool.setModel(model);
    analyzeTool.setModelFilename(model_name);
    analyzeTool.setCoordinatesFileName(coordinates_file_name);
    analyzeTool.setLowpassCutoffFrequency(-1);
    analyzeTool.setResultsDir("results");

    const auto &bodies = model.getComponentList<Body>();
    for (auto& root : bodies) {
        const std::string& root_name = root.getName();
        for (auto& sub_component: bodies) {
            const std::string& sub_component_name = sub_component.getName();
            // Create point kinematics reporter
            PointKinematics pointKin;
            pointKin.setPointName(root_name + "-" + sub_component_name);
            pointKin.setPoint(SimTK::Vec3(1.0, 2.0, 3.0));
            pointKin.setBody(&root);
            pointKin.setRelativeToBody(&sub_component);
            analyzeTool.updAnalysisSet().cloneAndAppend(pointKin);
        }
    }

    std::string output_file_name = "test_analysis_point_kinematics_2.xml";
    analyzeTool.print(output_file_name);
    AnalyzeTool roundTrip(output_file_name);
    roundTrip.run();
}

