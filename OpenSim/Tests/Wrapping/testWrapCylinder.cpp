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

// Section containing Geometry related helpers.
namespace {

    constexpr double c_TAU = 2. * SimTK_PI;

    // A path segment determined in terms of the start and end point.
    template<typename T>
    struct PathSegment final
    {
        PathSegment(T startPoint, T endPoint) :
            start(std::move(startPoint)),
            end(std::move(endPoint))
        {}

        template<typename X>
        PathSegment(PathSegment<X> other) :
            PathSegment{
                T(std::move(other.start)),
                T(std::move(other.end)),
            }
        {}

        T start;
        T end;
    };

    // Swaps start and end of path.
    template<typename T>
    void SwapStartEnd(PathSegment<T>& path)
    {
        T tmp(std::move(path.start));
        path.start = std::move(path.end);
        path.end = std::move(tmp);
    }

    using PathSegmentVec3 = PathSegment<SimTK::Vec3>;

    std::ostream& operator<<(
        std::ostream& os,
        const PathSegmentVec3& path)
    {
        return os <<
            "PathSegment{start: " << path.start << ", end: " << path.end << "}";
    }

    PathSegmentVec3 operator*(
        const SimTK::Rotation& rot,
        const PathSegmentVec3& path)
    {
        return {
            rot * path.start,
            rot * path.end,
        };
    }

    enum class RotationDirection
    {
        Positive,
        Negative,
    };

    std::ostream& operator<<(
        std::ostream& os,
        const RotationDirection& direction)
    {
        return os << "RotationDirection::" << (
            direction == RotationDirection::Positive ?
            "Positive" :
            "Negative");
    }

    RotationDirection operator!(RotationDirection direction)
    {
        return direction == RotationDirection::Positive ?
            RotationDirection::Negative :
            RotationDirection::Positive;
    }

    // Angular distance from start- to end-angle in either positive or negative
    // direction.
    double AngularDistance(
        double startAngle,
        double endAngle,
        RotationDirection direction)
    {
        double distance = std::fmod(endAngle - startAngle, c_TAU);
        while (distance < 0. && direction == RotationDirection::Positive) {
            distance += c_TAU;
        }
        while (distance > 0. && direction == RotationDirection::Negative) {
            distance -= c_TAU;
        }
        return distance;
    }

    // Returns direction of shortest angular distance for a vector aligned with the
    // start point to become aligned with the end point (true if positive).
    RotationDirection DirectionOfShortestAngularDistance(
        SimTK::Vec2 start,
        SimTK::Vec2 end)
    {
        // Compute angular distance assuming positive rotation.
        double distance = AngularDistance(
            std::atan2(start[1], start[0]),
            std::atan2(end[1], end[0]),
            RotationDirection::Positive);
        // Check if positive direction was the shortest path.
        return distance <= SimTK::Pi ?
            RotationDirection::Positive :
            RotationDirection::Negative;
    }

    RotationDirection DirectionOfShortestAngularDistanceAboutZAxis(
        const PathSegmentVec3& path)
    {
        return DirectionOfShortestAngularDistance(
            path.start.getSubVec<2>(0),
            path.end.getSubVec<2>(0));
    }

    // Representation of a point in space using cylindrical coordinates.
    struct CylindricalCoordinates final {
        // Convert cartesian to cylindrical coordinates.
        CylindricalCoordinates(const SimTK::Vec3& point) :
            radius(SimTK::Vec2(point[0], point[1]).norm()),
            angle(std::atan2(point[1], point[0])),
            axialPosition(point[2]) {}

        double radius;
        double angle;
        double axialPosition;
    };

    // A geodesic (path) definition on a cylindrical surface.
    //
    // The wrapping problem is not solved here, instead the geodesic is constructed
    // given a wrapping result, and allows for the surface tangent direction and
    // path length to be evaluated. These values can then be used for asserting correctness of
    // the given wrapping result in later tests.
    class CylinderGeodesic final
    {
        CylinderGeodesic(
            const PathSegment<CylindricalCoordinates>& wrappedPath,
            RotationDirection wrappingDirection) :
            _startPoint(wrappedPath.start),
            _axialDistance(
                wrappedPath.end.axialPosition
                - wrappedPath.start.axialPosition),
            _angularDistance(
                AngularDistance(
                    wrappedPath.start.angle,
                    wrappedPath.end.angle,
                    wrappingDirection)),
            _cylinderOrientation(SimTK::Rotation()) {}

        CylinderGeodesic(
            const PathSegmentVec3& wrappedPath,
            RotationDirection wrappingDirection) :
            CylinderGeodesic(
                PathSegment<CylindricalCoordinates>(wrappedPath),
                wrappingDirection) {}

    public:
        CylinderGeodesic(
            const PathSegmentVec3& wrappedPath,
            RotationDirection wrappingDirection,
            const SimTK::Rotation& cylinderOrientation) :
            CylinderGeodesic(
                cylinderOrientation.invert() * wrappedPath,
                wrappingDirection)
        {
            _cylinderOrientation = cylinderOrientation;
        }

        double getRadius() const {
            return _startPoint.radius;
        }

    private:
        // Helper for computing the tangent vector to the surface along the
        // geodesic path. The angle argument allows for computing the tangent
        // vector at a certain point along the path.
        SimTK::Vec3 computeSurfaceTangentVector(double angle) const
        {
            SimTK::Rotation rotZAxis;
            rotZAxis.setRotationFromAngleAboutZ(angle);
            return _cylinderOrientation * rotZAxis * (
                getRadius() * _angularDistance * SimTK::Vec3(0., 1., 0.)
                + _axialDistance * SimTK::Vec3(0., 0., 1.));
        }

    public:
        // Compute surface tangent direction at start of the path.
        SimTK::UnitVec3 computeTangentDirectionAtStart() const
        {
            return SimTK::UnitVec3(
                computeSurfaceTangentVector(_startPoint.angle));
        }

        // Compute surface tangent direction at end of the path.
        SimTK::UnitVec3 computeTangentDirectionAtEnd() const
        {
            return SimTK::UnitVec3(computeSurfaceTangentVector(
                _startPoint.angle + _angularDistance));
        }

        // Compute geodesic path length.
        double computeLength() const
        {
            return std::sqrt(
                std::pow(getRadius() * _angularDistance, 2)
                + std::pow(_axialDistance, 2));
        }

    private:
        // Local coordinates of starting point of wrapped path.
        CylindricalCoordinates _startPoint;
        // Distance from start to end of wrapped path.
        double _axialDistance;
        double _angularDistance;
        // Cylinder orientation wrt ground frame.
        SimTK::Rotation _cylinderOrientation;
    };
}

// Section with helpers for evaluating the wrapping result in the upcoming test.
namespace {

    // Struct for holding the result of the wrapping solution.
    struct WrapTestResult final
    {
        static WrapTestResult NoWrap()
        {
            WrapTestResult result;
            result.noWrap = true;
            return result;
        }

        // Path segment on cylinder surface.
        PathSegmentVec3 path{
            SimTK::Vec3{SimTK::NaN},
            SimTK::Vec3{SimTK::NaN},
        };
        // Path segment length.
        double length = SimTK::NaN;
        // Direction of wrapping wrt cylinder axis.
        RotationDirection direction = RotationDirection::Positive;
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
            "direction: " << result.direction << "}";
    }

    // Struct holding the tolerances when asserting the wrapping result.
    struct WrappingTolerances final
    {
        WrappingTolerances() = default;

        WrappingTolerances(double eps) :
            position(eps),
            length(eps),
            tangentDirection(eps) {}

        double position = 1e-9;
        double length = 1e-9;
        double tangentDirection = 1e-9;
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
        const PathSegmentVec3& lhs,
        const PathSegmentVec3& rhs,
        double tolerance)
    {
        return IsEqualWithinTolerance(lhs.start, rhs.start, tolerance)
            && IsEqualWithinTolerance(lhs.end, rhs.end, tolerance);
    }

    bool IsEqualWithinTolerance(
        const WrapTestResult& lhs,
        const WrapTestResult& rhs,
        double tolerance)
    {
        if (lhs.noWrap && rhs.noWrap) {
            return true;
        }
        return IsEqualWithinTolerance(lhs.path, rhs.path, tolerance)
            && IsEqualWithinTolerance(lhs.length, rhs.length, tolerance)
            && lhs.direction == rhs.direction
            && lhs.noWrap == rhs.noWrap;
    }

    double ErrorInfinityNorm(
        const SimTK::UnitVec3& lhs,
        const SimTK::UnitVec3& rhs)
    {
        return SimTK::max((lhs.asVec3() - rhs.asVec3()).abs());
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
    struct WrapInput final
    {
        SimTK::Rotation cylinderOrientation() const
        {
            SimTK::Rotation orientation;
            orientation.setRotationToBodyFixedXYZ(eulerRotations);
            return orientation;
        }

        // Endpoints of the total path:
        PathSegmentVec3 path{
            SimTK::Vec3{SimTK::NaN},
            SimTK::Vec3{SimTK::NaN},
        };
        // Wrapping cylinder parameters:
        double radius = 1.;
        double cylinderLength = 1.;
        std::string quadrant = "all";
        SimTK::Vec3 eulerRotations = {0., 0., 0.};
    };

    std::ostream& operator<<(std::ostream& os, const WrapInput& input)
    {
        return os <<
            "WrapInput{" <<
            "path: " << input.path << ", " <<
            "radius: " << input.radius << ", " <<
            "cylinderLength: " << input.cylinderLength << ", " <<
            "quadrant: " << input.quadrant << ", " <<
            "eulerRotations: " << input.eulerRotations << "}";
    }

    // Simulates the wrapping scenario as configured by the WrapInput.
    WrapTestResult solve(const WrapInput& input, bool visualize)
    {
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
                .getPath<GeometryPath>()
                .getWrapSet()
                .get("pathwrap")
                .getPreviousWrap();

        // Convert the WrapResult to a WrapTestResult for convenient assertions.
        WrapTestResult result;
        result.path = {wrapResult.r1, wrapResult.r2};
        result.length = wrapResult.wrap_path_length;
        // Determine the wrapping sign based on the first path segment.
        result.direction = DirectionOfShortestAngularDistanceAboutZAxis(
            input.cylinderOrientation().invert() * PathSegmentVec3{
                input.path.start,
                wrapResult.r1,
            });
        result.noWrap = wrapResult.wrap_pts.size() == 0;

        return result;
    }

}

// Section on testing.
namespace {

    // Function for testing the geodesic properties of the wrapping result.
    //
    // Returns string with info on failed tests, or empty string in case of
    // success.
    std::string TestGeodesicProperties(
        const WrapInput& input,
        const WrapTestResult& result,
        const WrappingTolerances& tol,
        const std::string& name)
    {
        std::ostringstream oss;
        std::string delim = "\n        ";

        // =====================================================================
        // ============ Test: Wrapped points distance to surface. ==============
        // =====================================================================

        // We can assert if the wrapped path points lie on the surface of the
        // cylinder, by computing the radial position in cylindrical
        // coordinates.

        CylinderGeodesic geodesicPath(
            result.path,
            result.direction,
            input.cylinderOrientation());

        double error = std::abs(geodesicPath.getRadius() - input.radius);
        if (error > tol.position) {
            oss << delim << name << ": Distance to surface error = " << error <<
                " exceeds tolerance = " << tol.position;
        }

        // =====================================================================
        // ===================== Test: Wrapping length. ========================
        // =====================================================================

        // We can test the wrap-result length by recomputing it from the path
        // points via the CylinderGeodesic.

        error = std::abs(geodesicPath.computeLength() - result.length);
        if (error > tol.length) {
            oss << delim << name << ": Length error = " << error <<
                " exceeds tolerance = " << tol.length;
        }

        // =====================================================================
        // ===================== Test: Tangency to surface =====================
        // =====================================================================

        // The path must at all times be tangent to the cylinder surface. We can
        // test that the straight-line segments, before and after wrapping the
        // surface, are also tangent to the surface.

        // Straight line segment from path start point to surface must be
        // tangent to cylinder surface:
        SimTK::UnitVec3 start_straight_segment_direction =
            SimTK::UnitVec3(result.path.start - input.path.start);
        error = ErrorInfinityNorm(
            geodesicPath.computeTangentDirectionAtStart(),
            start_straight_segment_direction);
        if (error > tol.tangentDirection) {
            oss << delim << name << ": Start segment tangent direction error = " << error
                << " exceeds tolerance = " << tol.tangentDirection;
            oss << delim << name << ": Start surface tangent direction = "
                << geodesicPath.computeTangentDirectionAtStart();
            oss << delim << name << ": Start straight line segment direction = "
                << start_straight_segment_direction;
        }

        // Straight line segment from surface to path end point must be tangent
        // to cylinder surface:
        SimTK::UnitVec3 end_straight_segment_direction =
            SimTK::UnitVec3(input.path.end - result.path.end);
        error = ErrorInfinityNorm(
            geodesicPath.computeTangentDirectionAtEnd(),
            end_straight_segment_direction);
        if (error > tol.tangentDirection) {
            oss << delim << name << ": End segment tangent direction error = " << error
                << " exceeds tolerance = " << tol.tangentDirection;
            oss << delim << name << ": End surface tangent direction = "
                << geodesicPath.computeTangentDirectionAtEnd();
            oss << delim << name << ": End straight line segment direction = "
                << end_straight_segment_direction;
        }

        // Return info on failed tests if any, or an empty string.
        return oss.str();
    }

    // Simulates and tests a wrapping case:
    // - Computes the wrapping result given the input.
    // - Tests if computed result is equal to the expected result.
    // - Tests if geodesic properties of computed result are correct.
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
        std::ostringstream failedBuf;

        // Test if wrapping result matches expected result.
        if (!IsEqualWithinTolerance(result, expected, tol.position)) {
            failedBuf << "\n        ";
            failedBuf << "Expected and simulated result not equal to within tolerance = " << tol.position;
        }

        // Test geodesic properties of wrapping result.
        if (!result.noWrap) {
            failedBuf << TestGeodesicProperties(
                input,
                result,
                tol,
                "Wrapping result"
            );
        }

        std::string failedStr = failedBuf.str();
        if (failedStr.empty()) {
            return failedStr;
        }

        // If failed, print info related to this test case.
        std::ostringstream oss;
        oss << "\nFAILED: case = " << testCase;
        oss << "\n    input = " << input;
        oss << "\n    result = " << result;
        oss << "\n    expected = " << expected;
        oss << "\n    caused by:" << failedStr;
        return oss.str();
    }

}

int main()
{
    struct TestCase
    {
        std::string name{};
        WrapInput input{};
        WrapTestResult expected{};
        bool visualize = false;
    } testCase;

    std::vector<TestCase> testCaseList{};

    testCase.input.radius = 1.;
    testCase.input.cylinderLength = 10.;
    testCase.input.quadrant = "all";

    // =========================================================================
    // ======================== No-Wrapping Case ===============================
    // =========================================================================

    // Placing the path inside the cylinder results in a no-wrap.
    testCase.name = "Inside cylinder";
    testCase.input.path = {{0., 0., 0.}, {0., 0., 0.}};

    testCase.expected = WrapTestResult::NoWrap();

    testCaseList.push_back(testCase);

    // =========================================================================
    // ======================= Perpendicular Case 1 ============================
    // =========================================================================

    testCase.name = "Perpendicular 1";
    testCase.input.path = {{2, 0.9, 0}, {-2, 1.0, 0}};

    // Solution obtained from copy-pasting the solution (OpenSim 4.5).
    testCase.expected.path = { {0.0505759009075089, 0.998720220205536, 0}, {0, 1, 0} };
    testCase.expected.length = 0.0505974872969326;
    testCase.expected.noWrap = false;

    testCaseList.push_back(testCase);

    // =========================================================================
    // ======================= Perpendicular Case 2 ============================
    // =========================================================================

    testCase.name = "Perpendicular 2";
    testCase.input.path = {{2, -2, 0}, {-2, 2.1, 0}};

    testCase.expected.path = {
        {0.911437827766147, 0.411437827766148, 0.},
        {0.441911556159668, 0.897058624913969, 0.},
    };
    testCase.expected.length = 0.689036814042993;

    testCaseList.push_back(testCase);

    // =========================================================================
    // =========================== Reversed path ===============================
    // =========================================================================

    // Swapping the start and end should not affect the wrapping path, other
    // than inverting the wrapping direction.

    for (size_t i = 0, len = testCaseList.size(); i < len; ++i) {
        TestCase reversedCase(testCaseList[i]);
        reversedCase.name = testCaseList[i].name + " (reversed)";
        SwapStartEnd(reversedCase.input.path);
        SwapStartEnd(reversedCase.expected.path);
        reversedCase.expected.direction = !(testCaseList[i].expected.direction);
        testCaseList.push_back(reversedCase);
    }

    // =========================================================================
    // =========================== Run all tests ===============================
    // =========================================================================

    std::vector<std::string> failLog;
    WrappingTolerances tolerance;

    for (size_t i = 0; i < testCaseList.size(); ++i) {
        failLog.push_back(TestWrapping(
            testCaseList[i].input,
            testCaseList[i].expected,
            tolerance,
            testCaseList[i].name,
            testCaseList[i].visualize));
    }

    // =========================================================================
    // ====================== Handling of Test Results =========================
    // =========================================================================

    size_t failedCount = 0;
    for (size_t i = 0; i < failLog.size(); ++i) {
        if (!failLog[i].empty()) {
            ++failedCount;
            std::cout << failLog[i] << std::endl;
        }
    }

    if (failedCount > 0) {
        std::cout << "Wrap Cylinder Test: Failed " << failedCount << " out of "
            << failLog.size() << " tests." << std::endl;
        return 1;
    }

    std::cout << "Wrap Cylinder Test: Passed " << failLog.size() << " tests."
        << std::endl;
    return 0;
}