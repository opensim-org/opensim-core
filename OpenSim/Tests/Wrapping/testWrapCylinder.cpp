/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrapCylinder.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2022 Stanford University and the Authors                *
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

#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <string>
#include <iostream>
#include <cmath>
#include <sstream>
#include <functional>
#include <memory>

using namespace OpenSim;

namespace {
    const double c_TAU = 2. * SimTK::Pi;

    // A path segment determined in terms of the start and end point.
    struct PathSegment final {
        PathSegment() = default;

        PathSegment(
            const SimTK::Vec3& startPoint,
            const SimTK::Vec3& endPoint) :
            start(startPoint),
            end(endPoint)
        {}

        SimTK::Vec3 start{SimTK::NaN};
        SimTK::Vec3 end{SimTK::NaN};
    };

    // Returns PathSegment with start and end swapped.
    PathSegment Reversed(const PathSegment& path) {
        return PathSegment{path.end, path.start};
    }

    std::ostream& operator<<(
        std::ostream& os,
        const PathSegment& path)
    {
        return os <<
            "PathSegment{start: " << path.start << ", end: " << path.end << "}";
    }

    PathSegment operator*(
        const SimTK::Rotation& rot,
        const PathSegment& path)
    {
        return PathSegment{
            rot * path.start,
            rot * path.end,
        };
    }

    // Angular distance from start- to end-angle in either positive or negative direction.
    double AngularDistance(
        double startAngle,
        double endAngle,
        bool positiveRotation)
    {
        double distance = std::fmod(endAngle - startAngle, c_TAU);
        while ((distance < 0.) && positiveRotation) {
            distance += c_TAU;
        }
        while ((distance > 0.) && !positiveRotation) {
            distance -= c_TAU;
        }
        return distance;
    }

    // Returns direction of shortest angular distance for a vector aligned with the
    // start point to become aligned with the end point (true if positive).
    bool DirectionOfShortestAngularDistance(
        SimTK::Vec2 start,
        SimTK::Vec2 end)
    {
        return AngularDistance(
            std::atan2(start[1], start[0]),
            std::atan2(end[1], end[0]),
            true) <= SimTK::Pi;
    }

    bool DirectionOfShortestAngularDistanceAboutZAxis(const PathSegment& path)
    {
        return DirectionOfShortestAngularDistance(
            path.start.getSubVec<2>(0),
            path.end.getSubVec<2>(0));
    }

}

// Section with helpers for evaluating the wrapping result in the upcoming test.
namespace {

    // Struct for holding the result of the wrapping solution.
    struct WrapTestResult final {

        static WrapTestResult NoWrap() {
            WrapTestResult result;
            result.noWrap = true;
            return result;
        }

        // Path segment on cylinder surface.
        PathSegment path;
        // Path segment length.
        double length = SimTK::NaN;
        // Direction of wrapping wrt cylinder axis.
        bool positiveDirection = true;
        // True if there is no wrapping (the other fields don't matter).
        bool noWrap = false;
    };

    std::ostream& operator<<(
        std::ostream& os,
        const WrapTestResult& result)
    {
        if (result.noWrap) {
            return os << "WrapTestResult{noWrap = true}";
        }
        return os <<
            "WrapTestResult{" <<
            "path: " << result.path << ", " <<
            "length: " << result.length << ", " <<
            "direction: " << (result.positiveDirection? "Positive" : "Negative")
            << "}";
    }

    // Struct holding the tolerances when asserting the wrapping result.
    struct WrappingTolerances final {
        WrappingTolerances() = default;

        WrappingTolerances(double eps) :
            position(eps),
            length(eps)
        {}

        double position = 1e-9;
        double length = 1e-9;
    };

    bool IsEqualWithinTolerance(
        double lhs,
        double rhs,
        double tolerance)
    {
        return std::abs(lhs - rhs) <= tolerance;
    }

    bool IsEqualWithinTolerance(
        const SimTK::Vec3& lhs,
        const SimTK::Vec3& rhs,
        double tolerance)
    {
        return IsEqualWithinTolerance(lhs[0], rhs[0], tolerance)
            && IsEqualWithinTolerance(lhs[1], rhs[1], tolerance)
            && IsEqualWithinTolerance(lhs[2], rhs[2], tolerance);
    }

    bool IsEqualWithinTolerance(
        const PathSegment& lhs,
        const PathSegment& rhs,
        double tolerance)
    {
        return IsEqualWithinTolerance(lhs.start, rhs.start, tolerance)
            && IsEqualWithinTolerance(lhs.end, rhs.end, tolerance);
    }

    bool IsEqualWithinTolerance(
        const WrapTestResult& lhs,
        const WrapTestResult& rhs,
        const WrappingTolerances& tolerance)
    {
        if (lhs.noWrap && rhs.noWrap) {
            return true;
        }
        return IsEqualWithinTolerance(lhs.path, rhs.path, tolerance.position)
            && IsEqualWithinTolerance(lhs.length, rhs.length, tolerance.length)
            && lhs.positiveDirection == rhs.positiveDirection
            && lhs.noWrap == rhs.noWrap;
    }

}

// Section on configuring and simulating a specific wrapping scenario.
namespace {

    // Parameterization of the scenario for testing the wrapping.
    //
    // The simulated scene is one with a cylinder at the origin, with a user
    // defined radius, length, active quadrant, and relative orientation.
    // A specific wrapping case can be configured by choosing the start and end
    // points of the path.
    struct WrapInput final {

        SimTK::Rotation cylinderOrientation() const {
            SimTK::Rotation orientation;
            orientation.setRotationToBodyFixedXYZ(eulerRotations);
            return orientation;
        }

        // Endpoints of the total path:
        PathSegment path;

        // Wrapping cylinder parameters:
        double radius = 1.;
        double cylinderLength = 1.;
        std::string quadrant = "all";
        SimTK::Vec3 eulerRotations = {0., 0., 0.};
    };

    std::ostream& operator<<(std::ostream& os, const WrapInput& input) {
        return os <<
            "WrapInput{" <<
            "path: " << input.path << ", " <<
            "radius: " << input.radius << ", " <<
            "cylinderLength: " << input.cylinderLength << ", " <<
            "quadrant: " << input.quadrant << ", " <<
            "eulerRotations: " << input.eulerRotations << "}";
    }

    // Simulates the wrapping scenario as configured by the WrapInput.
    WrapTestResult solve(const WrapInput& input, bool visualize) {
        Model model;
        model.setName("testWrapCylinderModel");

        // Add a spring to create the wrapping path.
        {
            std::unique_ptr<PathSpring> spring(
                new PathSpring("spring", 1., 1., 1.));
            spring->updGeometryPath().appendNewPathPoint(
                "startPoint",
                model.get_ground(),
                input.path.start);
            spring->updGeometryPath().appendNewPathPoint(
                "endPoint",
                model.get_ground(),
                input.path.end);

            model.addComponent(spring.release());
        }

        // Add the cylinder as the wrapping surface for the spring.
        {
            std::unique_ptr<WrapCylinder> cylinder(new WrapCylinder());
            cylinder->setName("cylinder");
            cylinder->set_radius(input.radius);
            cylinder->set_length(input.cylinderLength);
            cylinder->set_quadrant(input.quadrant);
            cylinder->set_xyz_body_rotation(input.eulerRotations);
            cylinder->setFrame(model.getGround());

            model.updComponent<PathSpring>("spring")
                .updGeometryPath().addPathWrap(*cylinder.get());
            model.updGround().addWrapObject(cylinder.release());
        }

        model.finalizeConnections();
        model.setUseVisualizer(visualize);

        const SimTK::State& state = model.initSystem();

        if (visualize) {
            model.getVisualizer().show(state);
        }

        // Trigger computing the wrapping path (realizing the state will not).
        model.getComponent<PathSpring>("spring").getLength(state);
        const WrapResult wrapResult = model.getComponent<PathSpring>("spring")
                .getGeometryPath()
                .getWrapSet()
                .get("pathwrap")
                .getPreviousWrap();

        // Convert the WrapResult to a WrapTestResult for convenient assertions.
        WrapTestResult result;
        result.path = PathSegment{
            wrapResult.r1,
            wrapResult.r2,
        };
        result.length = wrapResult.wrap_path_length;
        // Determine the wrapping sign based on the first path segment.
        result.positiveDirection = DirectionOfShortestAngularDistanceAboutZAxis(
            input.cylinderOrientation().invert() * PathSegment{
                input.path.start,
                wrapResult.r1,
            });
        result.noWrap = wrapResult.wrap_pts.size() == 0;

        return result;
    }

    // Simulates and tests a wrapping case:
    // - Computes the wrapping result given the input.
    // - Tests if computed result is equal to the expected result.
    // - Returns a string containing info on the failed test, or an empty string
    // if succesful.
    std::string TestWrapping(
        const WrapInput& input,
        const WrapTestResult& expected,
        WrappingTolerances tol,
        std::string testCase,
        bool visualize = false)
    {

        WrapTestResult result = solve(input, visualize);

        if (!IsEqualWithinTolerance(result, expected, tol.position)) {
            std::ostringstream oss;
            oss << "\nFAILED: case = " << testCase;
            oss << "\n    input = " << input;
            oss << "\n    result = " << result;
            oss << "\n    expected = " << expected;
            oss << "\n    caused by: "
                << "Expected and simulated result not equal to within tolerance = "
                << tol.position;
            return oss.str();
        }

        return std::string{};
    }

}

int main()
{
    std::vector<std::string> failLog;
    WrappingTolerances tolerance;

    WrapInput input {};
    input.radius = 1.;
    input.cylinderLength = 10.;

    // =========================================================================
    // ======================== No-Wrapping Case ===============================
    // =========================================================================

    // Placing the path inside the cylinder results in a no-wrap.
    std::string name = "Inside cylinder";
    input.path = {{}, {}};

    WrapTestResult expected = WrapTestResult::NoWrap();

    failLog.push_back(TestWrapping(input, expected, tolerance, name));

    // =========================================================================
    // ======================= Perpendicular Case ==============================
    // =========================================================================

    name = "Perpendicular";
    input.path = {{2, -2, 0}, {-2, 2.1, 0}};

    // Solution obtained from copy-pasting the solution (OpenSim 4.5).
    expected.path = {
        {0.911437827766147, 0.411437827766148, 0.},
        {0.441911556159668, 0.897058624913969, 0.},
    };
    expected.length = 0.689036814042993;

    expected.noWrap = false;

    failLog.push_back(TestWrapping(input, expected, tolerance, name));

    // Swapping start and end should not change the path:
    input.path = Reversed(input.path);
    expected.path = Reversed(expected.path);
    expected.positiveDirection = false;

    failLog.push_back(TestWrapping(input, expected, tolerance, name));

    // =========================================================================
    // ====================== Handling of Test Results =========================
    // =========================================================================

    size_t failedCount = 0;
    for (size_t i = 0; i < failLog.size(); ++i) {
        if (!failLog[i].empty()) {
            ++failedCount;
            std::cerr << failLog[i] << std::endl;
        }
    }

    if (failedCount > 0) {
        std::cout << "Wrap Cylinder Test: Failed " << failedCount << " out of "
            << failLog.size() << " tests." << std::endl;
        return 1;
    }

    std::cout << "Wrap Cylinder Test: Passed " << failLog.size() << " tests." <<
        std::endl;
    return 0;
}