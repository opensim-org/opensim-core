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

void testSingleWrapObjectPerpendicular(OpenSim::WrapObject* wObj);
void testCompareWrapObjects(OpenSim::WrapObject* wObj1, OpenSim::WrapObject* wObj2);

int main()
{
    SimTK::Array_<std::string> failures;

    try {
        auto* wo = new WrapCylinder();
        wo->setName("pulley1");
        wo->set_radius(.5);
        wo->set_length(1);
        testSingleWrapObjectPerpendicular(wo);
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
        wo->set_radius(.5);
        testSingleWrapObjectPerpendicular(wo);
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
        wo->set_dimensions(Vec3(.5));
        testSingleWrapObjectPerpendicular(wo);
        wo->set_quadrant("+y");
        testSingleWrapObjectPerpendicular(wo);

    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapEllipsoid");
    }

    // Compare rotated wrap cylinder by angle theta with an ellipsoid with radii matching cylinder
    
    try {
        auto* woOne = new WrapCylinder();
        woOne->setName("pulley1");
        woOne->set_radius(.5);
        woOne->set_length(2);
        auto* woTwo = new WrapEllipsoid();
        woTwo->setName("pulley2");
        // -30 - 30 degrees guarantee no edge which is poorly handled and need to be dropped
        for (int i = -2; i <= 2; i++) {
            double angle = i * SimTK::Pi / 12;
            woOne->set_xyz_body_rotation(Vec3(0,  angle, 0));
            woTwo->set_dimensions(Vec3(.5/cos(angle), .5, 1));
            testCompareWrapObjects(woOne, woTwo);
        }
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("Test Compare failed.");
    } 

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}
// Test results of wrapping a sigle path perpendicular to a wrapObject 
// and compare results to analytical/expected answers
void testSingleWrapObjectPerpendicular(WrapObject* wrapObject)
{
    auto visualize = false;
    const double r = 0.5;
    Model model;
    model.setName("test"+wrapObject->getConcreteClassName());

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(-.5, 0, 0), Inertia(0.1, 0.1, 0.01));
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
        ASSERT_EQUAL<double>(len1, .9 + 0.25 * 2 * SimTK::Pi * r * i / nsteps, 1e-6); //SimTK::Eps

    }
}
// Test results of wrapping a sigle path around a wrapObject wObj1
// and compare results to analytically equivalent wrapObject wObj2
// For example a rotated cylinder against an ellipsoid
void testCompareWrapObjects(OpenSim::WrapObject* wObj1, OpenSim::WrapObject* wObj2) {
    auto visualize = false;
    const double r = 0.5;
    Model model;
    model.setName("test" + wObj1->getConcreteClassName()+wObj2->getConcreteClassName());

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(-.5, 0, 0), Inertia(0.1, 0.1, 0.01));
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
    // get angle
    
    double ang = wObj->get_xyz_body_rotation()[1];
    double a = 0.5;
    double b = .5 / cos(ang);
    // perimeter approx fomrula by Ramanujan
    //double p = SimTK::Pi * (3 * (a + b) - sqrt((3 * a + b) * (a + 3 * b)));
    
    int nsteps = 1000;
    double max_diff = 0.;
    for (int i = 0; i <= nsteps; ++i) {

        coord.setValue(s, i * SimTK::Pi / (4 * nsteps));
        model.realizeVelocity(s);

        if (visualize)
            model.getVisualizer().show(s);

        double len1 = spring1->getLength(s);
        double len2 = spring2->getLength(s);
        max_diff = std::max(max_diff, std::abs(len1 - len2));
        //std::cout << "i=" << i << " length=" << len1 << " " << len2 << " diff="  << std::abs(len1-len2) << std::endl;
        ASSERT_EQUAL<double>(len1, len2, .01);
    }
    //std::cout << "max_diff:" << max_diff << std::endl;
}

