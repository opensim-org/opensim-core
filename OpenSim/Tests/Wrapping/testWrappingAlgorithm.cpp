/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testWrappingAlgorithm.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2022 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include <set>
#include <string>
#include <iostream>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

void testSingleWrapObjectPerpendicular(OpenSim::WrapObject* wObj, Vec3 axialRotation = Vec3(0.0));
void testEllipsoidWrapLength(OpenSim::WrapEllipsoid* wObj);

const double radius = 0.5;
int main()
{
    SimTK::Array_<std::string> failures;
    
    try {
        auto* wo = new WrapCylinder();
        wo->setName("pulley1");
        wo->set_radius(radius);
        wo->set_length(1);
        testSingleWrapObjectPerpendicular(wo);
        // Rotating a cylinder around its axis doesn't change wrapping result but
        // changes the local coordinate system for computation by changing the quadrant
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, SimTK::Pi / 2 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, SimTK::Pi });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, -SimTK::Pi / 2 });
        wo->set_quadrant("+y");
        testSingleWrapObjectPerpendicular(wo);
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapCylinder");
    }
    
    try {
        auto* wo = new WrapSphere();
        wo->setName("pulley1");
        wo->set_radius(radius);
        testSingleWrapObjectPerpendicular(wo);
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, SimTK::Pi / 2 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, SimTK::Pi });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, -SimTK::Pi / 2 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, SimTK::Pi / 2, 0 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, SimTK::Pi, 0 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, -SimTK::Pi / 2, 0 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ SimTK::Pi / 2, 0, 0 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ SimTK::Pi, 0, 0 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ -SimTK::Pi / 2, 0, 0 });
        wo->set_quadrant("+y");
        testSingleWrapObjectPerpendicular(wo);
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapSphere");
    }
    
    try {
        auto* wo = new WrapEllipsoid();
        wo->setName("pulley1");
        // Use wrapEllipsoid methods to wrap on a sphere
        wo->set_dimensions(Vec3(radius));
        testSingleWrapObjectPerpendicular(wo);
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, SimTK::Pi / 2 });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, SimTK::Pi });
        testSingleWrapObjectPerpendicular(wo, Vec3{ 0, 0, -SimTK::Pi / 2 });
        wo->set_quadrant("+y");
        testSingleWrapObjectPerpendicular(wo);

    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapEllipsoid");
    } 
    
    try {
        auto* wo = new WrapEllipsoid();
        wo->setName("pulley1");
        // change rotation angle by 1 deg up to a little under pi/2 which is a singularity
        for (double angle = 0; angle < SimTK::Pi/2 -.1; angle += SimTK::Pi / 180*5) {
            wo->set_dimensions(Vec3(radius / cos(angle), radius, 1));
            testEllipsoidWrapLength(wo);
        }
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("testEllipsoidWrapLength");
    }
    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << std::endl;
        return 1;
    }

    cout << "Done" << std::endl;
    return 0;
}
// Test results of wrapping a single path perpendicular to a wrapObject
// particularly path perpendicular to cylinder axis (Z axis)
// and compare results to analytical/expected answers. Since cross-section is a circle/arc
// results should match a sphere or ellipsoid with matching radius.
// In Ground frame the path is in XY plane along x axis tangent the wrapObject then wraps
// with coordinate change.
void testSingleWrapObjectPerpendicular(WrapObject* wrapObject, Vec3 axisRotations)
{
    auto visualize = false;
    const double r = radius;
    Model model;
    model.setName("test"+wrapObject->getConcreteClassName());

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(-r, 0, 0), Inertia(0.1, 0.1, 0.01));
    model.addComponent(body);
    
    auto joint = new PinJoint("pin", ground, *body);
    auto& qi = joint->updCoordinate();
    qi.setName("q_pin");
    model.addComponent(joint);

    // Add the wrap object to the body, which takes ownership of it
    WrapObject* wObj = wrapObject->clone();
    wObj->set_xyz_body_rotation(axisRotations);
    ground.addWrapObject(wObj);

    // One spring has wrap cylinder with respect to ground origin
    PathSpring* spring1 =
        new PathSpring("spring1", 1.0, 0.1, 0.01);
    //offset in X direction to avoid ambiguous scenario where path passes through center
    spring1->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(r-.1, r, 0)); 
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(-r, r, 0));
    spring1->updGeometryPath().addPathWrap(*wObj);

    model.addComponent(spring1);

    model.finalizeConnections();
    model.setUseVisualizer(visualize);
    //model.print(wObj->getConcreteClassName()+"Analytical.osim");
    //model.updDisplayHints().disableVisualization();
    SimTK::State& s = model.initSystem();
    auto& coord = joint->getCoordinate();
    int nsteps = 1000;
    for (int i = 0; i <= nsteps; ++i) {
        
        coord.setValue(s, i*SimTK::Pi/(2*nsteps));
        model.realizeVelocity(s);

        if (visualize)
            model.getVisualizer().show(s);

        double ma1 = spring1->computeMomentArm(s, coord);

        ASSERT_EQUAL<double>(-r, ma1, .0001); // SimTK::Eps
        double len1 = spring1->getLength(s);
        // Length is 2*r -0.1 by construction plus a portion of a quarter circle with radius r proportional to i
        ASSERT_EQUAL<double>(len1, 2*r-0.1 + 0.25 * 2 * SimTK::Pi * r * i / nsteps, 1e-6); 

    }
}
// Ellipsoid passed in has radii of a, b, c wrapping occurs along z axis
// no closed-form analytical solution to compare but approximate length
// for full ellipse. Wrapping should match approximately 1/4 
// perimeter of ellipse + fixed offset baked in.
void testEllipsoidWrapLength(OpenSim::WrapEllipsoid* wrapObject)
{
    auto visualize = false;
    const double r = radius;
    Model model;

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(-r, 0, 0), Inertia(0.1, 0.1, 0.01));
    model.addComponent(body);

    auto joint = new PinJoint("pin", ground, *body);
    auto& qi = joint->updCoordinate();
    qi.setName("q_pin");
    model.addComponent(joint);

    // Add the wrap object to the body, which takes ownership of it
    WrapObject* wObj = wrapObject->clone();
    ground.addWrapObject(wObj);

    // One spring has wrap cylinder with respect to ground origin
    PathSpring* spring1 =
        new PathSpring("spring1", 1.0, 0.1, 0.01);
    //offset in X direction to avoid ambiguous scenario where path passes through center
    spring1->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(r - .1, r, 0)); 
    // insertion point is -r down from the tip of the long axis of the ellipsoid
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(-wrapObject->get_dimensions()[0], -r, 0));
    spring1->updGeometryPath().addPathWrap(*wObj);

    model.addComponent(spring1);

    model.finalizeConnections();
    model.setUseVisualizer(visualize);

    SimTK::State& s = model.initSystem();
    model.realizeVelocity(s);

    if (visualize)
        model.getVisualizer().show(s);

    double len1 = spring1->getLength(s);
    // ref Ramanujan formula https://www.cuemath.com/measurement/perimeter-of-ellipse/
    double a = wrapObject->get_dimensions()[0];
    double b = wrapObject->get_dimensions()[1];
    double h = ((a - b) * (a - b)) / ((a + b) * (a+b));
    double lengthAnalyticalApprox = SimTK::Pi * (a + b) * (1 + 3 * h / (10 + std::sqrt(4 - 3 * h)));
    //std::cout << "Compare: " << len1 << " and " << 2 * r - 0.1 + lengthAnalyticalApprox / 4 << std::endl;
    // Length is 1/4 ellipse + 2r -.1
    ASSERT_EQUAL<double>(len1, 2 * r - 0.1 + lengthAnalyticalApprox/4, 1e-4);

}

