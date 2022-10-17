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
void testCompareWrapObjects(OpenSim::WrapCylinder* wObj1, OpenSim::WrapObject* wObj2);

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
    
    // Compare wrap cylinder rotated by angle theta with an ellipsoid that has radii matching 
    // the cross section of the rotated cylinder
    
    try {
        auto* woOne = new WrapCylinder();
        woOne->setName("pulley1");
        woOne->set_radius(radius);
        woOne->set_length(2);
        auto* woTwo = new WrapEllipsoid();
        woTwo->setName("pulley2");
        // Change the angle between the cylinder axis and the line connecting end points of the pulley.
        // Values -36 to 36 degrees guarantee that wrapping doesn't occur at the cap of the cylinder which is a rather poorly 
        // handled scenario that leads to C0 length curve and may need to be dropped as non-biological 
        // this scenario also results in a truncated conic-section that can't be computed analytically.
        // Wider range should work but ellipsoid wrapping bugs out and produces a kink.
        // -Ayman 10/22
        auto startAngle = -SimTK::Pi / 5;
        auto endAngle = SimTK::Pi / 5;
        for (double angle = startAngle; angle <= endAngle; angle += SimTK::Pi/180) {
            woOne->set_xyz_body_rotation(Vec3(0,  angle, 0)); // Rotate the cylinder by angle
            woTwo->set_dimensions(Vec3(radius/cos(angle), radius, 1)); // Change radii of ellipsoid to match cross-section
            // std::cout << "compare cylinder vs ellipsoid at angle " << angle * 180/SimTK::Pi << std::endl;
            testCompareWrapObjects(woOne, woTwo);
        }
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("Test Compare failed.");
    } 

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << std::endl;
        return 1;
    }

    cout << "Done" << std::endl;
    return 0;
}
// Test results of wrapping a sigle path perpendicular to a wrapObject
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
    spring1->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(r-.1, r, 0)); //offset in X direction to avoid ambiguous scenario where path passes through center
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(-r, r, 0));
    spring1->updGeometryPath().addPathWrap(*wObj);

    model.addComponent(spring1);

    model.finalizeConnections();
    model.setUseVisualizer(visualize);
    //model.print(wObj->getConcreteClassName()+"Analytical.osim");
    //model.updDisplayHints().disableVisualization();
    SimTK::State& s = model.initSystem();
    auto& coord = joint->updCoordinate();
    const CoordinateSet& cset = model.getCoordinateSet();
    int nsteps = 1000;
    for (int i = 0; i <= nsteps; ++i) {
        
        coord.setValue(s, i*SimTK::Pi/(2*nsteps));
        model.realizeVelocity(s);

        if (visualize)
            model.getVisualizer().show(s);

        double ma1 = spring1->computeMomentArm(s, coord);

        ASSERT_EQUAL<double>(-r, ma1, .0001); // SimTK::Eps
        double len1 = spring1->getLength(s);
        // Length is 0.9 by construction plus a portion of a quarter circle with radius r proportional to i
        ASSERT_EQUAL<double>(len1, .9 + 0.25 * 2 * SimTK::Pi * r * i / nsteps, 1e-6); //this formula is based on radius = .5

    }
}
// Test results of wrapping a sigle path around a wrapCylinder wObj1
// and compare results to analytically equivalent wrapObject wObj2
// For example wrapping around a rotated cylinder against an ellipsoid with radii 
// picked to match the radii of the elliptical cross-section
void testCompareWrapObjects(OpenSim::WrapCylinder* wObj1, OpenSim::WrapObject* wObj2) {
    auto visualize = false;
    const double r = wObj1->get_radius();
    Model model;
    model.setName("test" + wObj1->getConcreteClassName()+wObj2->getConcreteClassName());

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(-r, 0, 0), Inertia(0.1, 0.1, 0.01));
    model.addComponent(body);

    auto joint = new PinJoint("pin", ground, *body);
    auto& qi = joint->updCoordinate();
    qi.setName("q_pin");
    model.addComponent(joint);

    // Add the wrap object to the body, which takes ownership of it
    WrapObject* wObj = wObj1->clone();
    ground.addWrapObject(wObj);

    // One spring has wrap cylinder with respect to ground origin
    PathSpring* spring1 =
        new PathSpring("spring1", 1.0, 0.1, 0.01);
    spring1->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(r - .1, r, 0)); //offset in X direction to avoid ambiguous scenario where path passes through center
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(-r, r, 0));
    spring1->updGeometryPath().addPathWrap(*wObj);

    model.addComponent(spring1);
    // Ceate offset frame in z direction
    PhysicalOffsetFrame* offsetZ = new PhysicalOffsetFrame(
    "z_plus1", ground, SimTK::Transform(Vec3(0, 0, 1)));
    model.addComponent(offsetZ);
    // repeat for wObj2 offset in z direction
    // Add the wrap object to the body, which takes ownership of it
    WrapObject* wObj_2 = wObj2->clone();
    offsetZ->addWrapObject(wObj_2);
    // One spring has wrap cylinder with respect to ground origin
    PathSpring* spring2 =
        new PathSpring("spring2", 1.0, 0.1, 0.01);
    spring2->updGeometryPath().
        appendNewPathPoint("origin", ground, Vec3(r - .1, r, 1)); //offset in X direction to avoid ambiguous scenario where path passes through center
    spring2->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(-r, r, 1));
    spring2->updGeometryPath().addPathWrap(*wObj_2);

    model.addComponent(spring2);

    model.finalizeConnections();
    model.setUseVisualizer(visualize);
    //model.print("wrapAnalytical.osim");
    //model.updDisplayHints().disableVisualization();
    SimTK::State& s = model.initSystem();
    auto& coord = joint->updCoordinate();
    const CoordinateSet& cset = model.getCoordinateSet();

    int nsteps = 1000;
    for (int i = 0; i <= nsteps; ++i) {

        coord.setValue(s, i * SimTK::Pi / (4 * nsteps));
        model.realizeVelocity(s);

        if (visualize)
            model.getVisualizer().show(s);

        double len1 = spring1->getLength(s);
        double len2 = spring2->getLength(s);
        ASSERT_EQUAL<double>(len1, len2, .01);
    }
}

