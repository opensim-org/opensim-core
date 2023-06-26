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

// Section with geometry related helper functions and classes.
namespace {

    // A path segment determined in terms of the start and end point.
    struct PathSegment final {
        PathSegment(
            SimTK::Vec3 startPoint,
            SimTK::Vec3 endPoint) :
            start(startPoint),
            end(endPoint)
        {}

        SimTK::Vec3 start {};
        SimTK::Vec3 end {};
    };

    PathSegment operator*(
        const SimTK::Rotation& rot,
        const PathSegment& path)
    {
        return PathSegment {
            rot * path.start,
            rot * path.end
        };
    }

    std::ostream& operator<<(
        std::ostream& os,
        const PathSegment& path)
    {
        return os <<
            "PathSegment{start: " << path.start << ", end: " << path.end << "}";
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
        PathSegment path = {{}, {}};
        // Path segment length.
        double length = NAN;
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
            "length: " << result.length << "}";
    }

    // Struct holding the tolerances when asserting the wrapping result.
    struct WrappingTolerances final {
        WrappingTolerances() = default;

        WrappingTolerances(double eps):
            position(eps),
            length(eps)
        {}

        double position = 1e-9;
        double length = 1e-9;
    };

    std::ostream& operator<<(std::ostream& os, const WrappingTolerances& tol) {
        return os <<
            "WrappingTolerances{" <<
            "position: " << tol.position << ", " <<
            "length: " << tol.length << "}";
    }

    bool IsEqual(
        double lhs,
        double rhs,
        double tolerance)
    {
        return std::abs(lhs - rhs) < tolerance;
    }

    bool IsEqual(
        const SimTK::Vec3& lhs,
        const SimTK::Vec3& rhs,
        double tolerance)
    {
        return IsEqual(lhs[0], rhs[0], tolerance)
            && IsEqual(lhs[1], rhs[1], tolerance)
            && IsEqual(lhs[2], rhs[2], tolerance);
    }

    bool IsEqual(
        const PathSegment& lhs,
        const PathSegment& rhs,
        double tolerance)
    {
        return IsEqual(lhs.start, rhs.start, tolerance)
            && IsEqual(lhs.end, rhs.end, tolerance);
    }

    bool IsEqual(
        const WrapTestResult& lhs,
        const WrapTestResult& rhs,
        const WrappingTolerances& tolerance)
    {
        if (lhs.noWrap && rhs.noWrap) {
            return true;
        }
        return IsEqual(lhs.path, rhs.path, tolerance.position)
            && IsEqual(lhs.length, rhs.length, tolerance.length)
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
        PathSegment path = {{}, {}};

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

        std::unique_ptr<WrapCylinder> cylinder(new WrapCylinder());
        cylinder->setName("cylinder");
        cylinder->set_radius(input.radius);
        cylinder->set_length(input.cylinderLength);
        cylinder->set_quadrant(input.quadrant);
        cylinder->set_xyz_body_rotation(input.eulerRotations);
        cylinder->setFrame(model.getGround());

        std::unique_ptr<PathSpring> spring (new PathSpring("spring", 1., 1., 1.));
        spring->updGeometryPath().appendNewPathPoint(
            "startPoint",
            model.get_ground(),
            input.path.start);
        spring->updGeometryPath().appendNewPathPoint(
            "endPoint",
            model.get_ground(),
            input.path.end);
        spring->updGeometryPath().addPathWrap(*cylinder.get());

        model.addComponent(spring.release());
        model.updGround().addWrapObject(cylinder.release());

        model.finalizeConnections();
        model.setUseVisualizer(visualize);

        const SimTK::State& state = model.initSystem();

        if (visualize) {
            model.realizeVelocity(state);
            model.getVisualizer().show(state);
        }

        model.updComponent<PathSpring>("spring").getLength(state);
        WrapResult wrapResult = model.updComponent<PathSpring>("spring")
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
        result.noWrap = wrapResult.wrap_pts.size() == 0;

        return result;
    }

}

namespace {
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

        if (!IsEqual(result, expected, tol.position)) {
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
    std::string failLog;
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

    failLog += TestWrapping(input, expected, tolerance, name);

    // =========================================================================
    // ======================= Perpendicular Case ==============================
    // =========================================================================

    name = "Perpendicular";
    input.path = {{2, 0.9, 0}, {-2, 1.0, 0}};

    expected.path = { {0.0505759009075089, 0.998720220205536, 0}, {0, 1, 0} };
    expected.length = 0.0505974872969326;
    expected.noWrap = false;

    failLog += TestWrapping(input, expected, tolerance, name, true);

    // =========================================================================
    // ====================== Handling of Test Results =========================
    // =========================================================================

    if (!failLog.empty()) {
        std::cerr << failLog << std::endl;
        return 1;
    }

    std::cout << "Wrap Cylinder Test Done." << std::endl;
    return 0;
}