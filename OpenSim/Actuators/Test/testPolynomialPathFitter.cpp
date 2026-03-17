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

#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Actuators/DeGrooteFregly2016Muscle.h>
#include <OpenSim/Actuators/ModelFactory.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/PathSpring.h>
#include <OpenSim/Simulation/Model/Scholz2015GeometryPath.h>
#include <OpenSim/Simulation/SimbodyEngine/SliderJoint.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;
using Catch::Matchers::ContainsSubstring;

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

    TableProcessor createCoordinatesTable(bool correctLabel, bool addMetaData,
            int numRows = 100) {
        SimTK::Vector column = SimTK::Test::randVector(numRows);
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
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring("No source model."));
    }

    fitter.setModel(createHangingMuscleModel());

    SECTION("No coordinates table") {
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring("No source table."));
    }

    SECTION("Missing metadata") {
        fitter.setCoordinateValues(createCoordinatesTable(true, false));
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
                "Table does not have 'inDegrees' metadata."));
    }

    SECTION("Missing coordinate data") {
        fitter.setCoordinateValues(createCoordinatesTable(false, true));
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
                "Expected the coordinate values table to contain a column"));
    }

    fitter.setCoordinateValues(createCoordinatesTable(true, true));

    SECTION("Number of threads") {
        fitter.setNumParallelThreads(-1);
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
                "Expected 'num_parallel_threads' to be"));
    }

    SECTION("Number of samples per frame") {
        fitter.setNumSamplesPerFrame(-1);
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
                "Expected 'num_samples_per_frame' to be a non-zero integer"));
    }

    SECTION("Latin hypercube algorithm") {
        fitter.setLatinHypercubeAlgorithm("wrong");
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
                "Property 'latin_hypercube_algorithm' has invalid value"));
    }

    SECTION("Maximum polynomial order") {
        fitter.setMaximumPolynomialOrder(10);
        REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
            "Expected 'maximum_polynomial_order' to be at most 9"));
    }
}

TEST_CASE("Number of rows less than the number of threads") {
    int numAvailableThreads = std::thread::hardware_concurrency();
    int numRows = (numAvailableThreads == 1) ? 1 : numAvailableThreads - 1;
    CAPTURE(numAvailableThreads);
    CAPTURE(numRows);

    PolynomialPathFitter fitter;
    fitter.setModel(createHangingMuscleModel());
    fitter.setCoordinateValues(createCoordinatesTable(true, true, numRows));
    fitter.setNumParallelThreads(numAvailableThreads);
    REQUIRE_THROWS_WITH(fitter.run(), ContainsSubstring(
        "Expected the number of time points in the coordinate values table"));
}

TEST_CASE("Using one thread does not segfault") {
    int numThreads = 1;
    int numRows = 5;
    CAPTURE(numThreads);
    CAPTURE(numRows);

    // Check that the path fitting process does not segfault.
    ModelProcessor modelProcessor = createHangingMuscleModel();
    TableProcessor values = createCoordinatesTable(true, true, numRows);
    PolynomialPathFitter fitter;
    fitter.setModel(modelProcessor);
    fitter.setCoordinateValues(values);
    fitter.setNumParallelThreads(numThreads);
    REQUIRE_NOTHROW(fitter.run());

    // Check that post-hoc evaluation of the fitted paths does not segfault.
    // The static function PolynomialPathFitter::evaluateFunctionBasedPaths()
    // previously segfaulted if the number of rows in the coordinate values
    // table was less than the number of hardware threads on the machine.
    // See #4280 for more details.
    Model model = modelProcessor.process();
    const std::string functionBasedPathsFile =
            model.getName() + "_FunctionBasedPathSet.xml";
    REQUIRE_NOTHROW(PolynomialPathFitter::evaluateFunctionBasedPaths(
            model, values, functionBasedPathsFile));
}

TEST_CASE("Verify fitted path: double pendulum with wrap cylinder") {

    // The double pendulum model used in this test is based on
    // exampleScholz2015GeometryPath.cpp.
    Model model = ModelFactory::createDoublePendulum();

    // Create a PathSpring with a Scholz2015GeometryPath.
    auto* spring = new PathSpring();
    spring->setName("path_spring");
    spring->setRestingLength(0.25);
    spring->setDissipation(0.75);
    spring->setStiffness(10.0);
    spring->set_path(Scholz2015GeometryPath());
    model.addComponent(spring);

    // Create a cylinder wrapping obstacle.
    auto* obstacle = new ContactCylinder(0.15,
        SimTK::Vec3(-0.2, 0.2, 0.), SimTK::Vec3(0),
        model.getComponent<Body>("/bodyset/b0"));
    model.addComponent(obstacle);

    // Define the path.
    Scholz2015GeometryPath& path = spring->updPath<Scholz2015GeometryPath>();
    path.setName("path");
    path.appendPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
    path.appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.5, 0.1, 0.));
    path.appendObstacle(*obstacle, SimTK::Vec3(0., 0.15, 0.));
    path.appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-0.5, 0.1, 0.));

    // Simulate the model to generate states data.
    SimTK::State state = model.initSystem();
    Manager manager(model);
    manager.setIntegratorMaximumStepSize(0.01);
    manager.initialize(state);
    manager.integrate(1.0);
    TimeSeriesTable states = manager.getStatesTable();

    SECTION("Path length and moment arm errors") {
        PolynomialPathFitter fitter;
        fitter.setModel(model);
        fitter.setCoordinateValues(states);
        fitter.run();

        // Expected number of samples.
        TimeSeriesTable coordinateValues(
                fmt::format("{}_coordinate_values.sto", model.getName()));
        TimeSeriesTable coordinateValuesSampled(
                fmt::format("{}_coordinate_values_sampled.sto",
                            model.getName()));

        // Expected number of samples.
        CHECK(coordinateValues.getNumRows() == states.getNumRows());

        // The total possible number of samples is actually rows * (samples + 1),
        // but the fitter filters out samples that are too far from the nominal
        // trajectory. In this case, the fitter filters out around 1% of samples.
        // We'll use a factor of two to account for roundoff errors on different
        // machines that may produce a slightly different number of samples.
        size_t maxNumSamples = states.getNumRows() *
                              (fitter.getNumSamplesPerFrame() + 1);
        CHECK(coordinateValuesSampled.getNumRows() > 0.98*maxNumSamples);

        // Path length error.
        TimeSeriesTable pathLengths(
                fmt::format("{}_path_lengths.sto", model.getName()));
        TimeSeriesTable pathLengthsFitted(
                fmt::format("{}_path_lengths_fitted.sto", model.getName()));
        double pathLengthMSE =
                (pathLengths.getDependentColumnAtIndex(0) -
                pathLengthsFitted.getDependentColumnAtIndex(0)).normSqr() /
                pathLengths.getNumRows();
        CHECK_THAT(std::sqrt(pathLengthMSE),
                   Catch::Matchers::WithinAbs(0, 1e-3));

        // Moment arm errors.
        TimeSeriesTable momentArms(
                fmt::format("{}_moment_arms.sto", model.getName()));
        TimeSeriesTable momentArmsFitted(
                fmt::format("{}_moment_arms_fitted.sto", model.getName()));
        double momentArmQ0MSE =
                (momentArms.getDependentColumnAtIndex(0) -
                momentArmsFitted.getDependentColumnAtIndex(0)).normSqr() /
                momentArms.getNumRows();
        CHECK_THAT(std::sqrt(momentArmQ0MSE),
                   Catch::Matchers::WithinAbs(0, 1e-3));
        double momentArmQ1MSE =
                (momentArms.getDependentColumnAtIndex(1) -
                momentArmsFitted.getDependentColumnAtIndex(1)).normSqr() /
                momentArms.getNumRows();
        CHECK_THAT(std::sqrt(momentArmQ1MSE),
                   Catch::Matchers::WithinAbs(0, 1e-3));
    }

    SECTION("Coordinate NaN values are filtered out") {
        // Inject a few NaN values to the states table.
        states.updDependentColumnAtIndex(0)[0] = SimTK::NaN;
        states.updDependentColumnAtIndex(3)[5] = SimTK::NaN;

        // Fit the path lengths and moment arms.
        PolynomialPathFitter fitter;
        fitter.setModel(model);
        fitter.setCoordinateValues(states);
        fitter.run();

        // Check that the tables from path fitting have the expected number
        // of rows.
        TimeSeriesTable filteredStates(
            fmt::format("{}_coordinate_values.sto", model.getName()));
        TimeSeriesTable filteredPathLengths(
            fmt::format("{}_path_lengths.sto", model.getName()));
        TimeSeriesTable filteredMomentArms(
            fmt::format("{}_moment_arms.sto", model.getName()));
        CHECK(filteredStates.getNumRows() == states.getNumRows() - 2);
        CHECK(filteredPathLengths.getNumRows() == states.getNumRows() - 2);
        CHECK(filteredMomentArms.getNumRows() == states.getNumRows() - 2);
    }
}

TEST_CASE("Graceful handling of failed path configurations") {

    // The model in this test is designed such that, at certain configurations,
    // the path length and moment arm computations fail. The forward integration
    // generates a validate simulation, but the final state puts one of the path
    // points very close to the ellipsoid wrapping obstacle. During random
    // sampling in the path fitter, coordinate values will be generated that
    // cause the via point to intersect the ellipsoid wrapping obstacle. The
    // path fitter should gracefully handle this failure by removing the
    // coordinate value sample that causes the failure and generate valid
    // path length and moment arm functions for the remaining coordinate value
    // samples.

    Model model = ModelFactory::createDoublePendulum();

    // Create a PathSpring with a Scholz2015GeometryPath.
    auto* spring = new PathSpring();
    spring->setRestingLength(0.25);
    spring->setDissipation(0.75);
    spring->setStiffness(10.0);
    spring->set_path(Scholz2015GeometryPath());
    model.addComponent(spring);

    // Create a ellipsoid wrapping obstacle.
    auto* obstacle = new ContactEllipsoid(SimTK::Vec3(0.15, 0.15, 0.5),
        SimTK::Vec3(-0.2, 0.2, 0.), SimTK::Vec3(0),
        model.getComponent<Body>("/bodyset/b0"));
    model.addComponent(obstacle);

    // Define the path.
    Scholz2015GeometryPath& path = spring->updPath<Scholz2015GeometryPath>();
    path.setName("path");
    path.appendPathPoint(model.getGround(), SimTK::Vec3(0.05, 0.05, 0.));
    path.appendPathPoint(model.getComponent<Body>("/bodyset/b0"),
            SimTK::Vec3(-0.5, 0.1, 0.));
    path.appendObstacle(*obstacle, SimTK::Vec3(0., 0.15, 0.));
    path.appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-0.75, 0.2, 0.));
    path.appendPathPoint(model.getComponent<Body>("/bodyset/b1"),
            SimTK::Vec3(-0.5, 0.1, 0.));

    // Simulate the model to generate states data.
    SimTK::State state = model.initSystem();
    Manager manager(model);
    manager.setIntegratorMaximumStepSize(0.01);
    manager.initialize(state);
    manager.integrate(0.55);
    TimeSeriesTable states = manager.getStatesTable();

    PolynomialPathFitter fitter;
    fitter.setModel(model);
    fitter.setCoordinateValues(states);
    fitter.setGlobalCoordinateSamplingBounds(SimTK::Vec2(-30, 30));
    fitter.setPathLengthTolerance(1e-3);
    fitter.setMomentArmTolerance(1e-3);
    fitter.run();

    // Path length error.
    TimeSeriesTable pathLengths(
            fmt::format("{}_path_lengths.sto", model.getName()));
    TimeSeriesTable pathLengthsFitted(
            fmt::format("{}_path_lengths_fitted.sto", model.getName()));
    double pathLengthMSE =
            (pathLengths.getDependentColumnAtIndex(0) -
            pathLengthsFitted.getDependentColumnAtIndex(0)).normSqr() /
            pathLengths.getNumRows();
    CHECK_THAT(std::sqrt(pathLengthMSE),
                Catch::Matchers::WithinAbs(0, 1e-3));

    // Moment arm errors.
    TimeSeriesTable momentArms(
            fmt::format("{}_moment_arms.sto", model.getName()));
    TimeSeriesTable momentArmsFitted(
            fmt::format("{}_moment_arms_fitted.sto", model.getName()));
    double momentArmQ0MSE =
            (momentArms.getDependentColumnAtIndex(0) -
            momentArmsFitted.getDependentColumnAtIndex(0)).normSqr() /
            momentArms.getNumRows();
    CHECK_THAT(std::sqrt(momentArmQ0MSE),
                Catch::Matchers::WithinAbs(0, 1e-2));
    double momentArmQ1MSE =
            (momentArms.getDependentColumnAtIndex(1) -
            momentArmsFitted.getDependentColumnAtIndex(1)).normSqr() /
            momentArms.getNumRows();
    CHECK_THAT(std::sqrt(momentArmQ1MSE),
                Catch::Matchers::WithinAbs(0, 1e-2));
}
