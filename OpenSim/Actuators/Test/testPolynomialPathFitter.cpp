/* -------------------------------------------------------------------------- *
*                 OpenSim:  testPolynomialPathFitter.cpp                      *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2023 Stanford University and the Authors                *
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

#include <OpenSim/Actuators/PolynomialPathFitter.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch/catch.hpp>

#include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>
#include <OpenSim/Common/CommonUtilities.h>

using namespace OpenSim;

// HELPER FUNCTIONS
namespace {
    ModelProcessor createHangingMuscleModel() {
        Model model;
        model.setName("hanging_muscle");
        model.set_gravity(SimTK::Vec3(9.81, 0, 0));
        auto* body = new Body("body", 0.5, SimTK::Vec3(0), SimTK::Inertia(0));
        model.addComponent(body);

        auto* joint = new SliderJoint("joint", model.getGround(), *body);
        auto& coord = joint->updCoordinate(SliderJoint::Coord::TranslationX);
        coord.setName("height");
        model.addComponent(joint);

        auto* actu = new DeGrooteFregly2016Muscle();
        actu->setName("muscle");
        actu->addNewPathPoint("origin", model.updGround(), SimTK::Vec3(0));
        actu->addNewPathPoint("insertion", *body, SimTK::Vec3(0));
        model.addForce(actu);
        model.finalizeConnections();
        ModelProcessor processor(model);

        return processor;
    }

    TableProcessor createCoordinatesTable(bool correctLabel, bool addMetaData) {
        SimTK::Vector column = SimTK::Test::randVector(100);
        std::vector<double> times;
        times.reserve(column.size());
        for (int i = 0; i < column.size(); ++i) {
            times.push_back(i * 0.01);
        }

        TimeSeriesTable table(times);
        std::string label = correctLabel ? "/joint/height/value" : "wrong";
        table.appendColumn(label, column);
        if (addMetaData) {
            table.addTableMetaData<std::string>("inDegrees", "no");
        }
        TableProcessor processor(table);

        return processor;
    }

}

TEST_CASE("Invalid configurations") {
    PolynomialPathFitter fitter;
    fitter.setNumParallelThreads(1);

    SECTION("No model") {
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains("No source model."));
    }

    fitter.setModel(createHangingMuscleModel());

    SECTION("No coordinates table") {
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains("No source table."));
    }

    SECTION("Missing metadata") {
        fitter.setCoordinateValues(createCoordinatesTable(true, false));
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains(
                "Table does not have 'inDegrees' metadata."));
    }

    SECTION("Missing coordinate data") {
        fitter.setCoordinateValues(createCoordinatesTable(false, true));
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains(
                "Expected the coordinate values table to contain a column"));
    }

    fitter.setCoordinateValues(createCoordinatesTable(true, true));

    SECTION("Number of threads") {
        fitter.setNumParallelThreads(-1);
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains(
                "Expected 'num_parallel_threads' to be"));
    }

    SECTION("Number of samples per frame") {
        fitter.setNumSamplesPerFrame(-1);
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains(
                "Expected 'num_samples_per_frame' to be a non-zero integer"));
    }

    SECTION("Latin hypercube algorithm") {
        fitter.setLatinHypercubeAlgorithm("wrong");
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains(
                "Property 'latin_hypercube_algorithm' has invalid value"));
    }

    SECTION("Maximum polynomial order") {
        fitter.setMaximumPolynomialOrder(10);
        REQUIRE_THROWS_WITH(fitter.run(), Catch::Contains(
            "Expected 'maximum_polynomial_order' to be at most 9"));
    }
}
