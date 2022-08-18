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

void testWrapObject(OpenSim::WrapObject *wObj);

int main()
{
    SimTK::Array_<std::string> failures;

    try {
        auto* wo = new WrapCylinder();
        wo->setName("pulley1");
        wo->set_radius(.5);
        wo->set_length(1);
        testWrapObject(wo);
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapCylinder");
    }
    try {
        auto* wo = new WrapSphere();
        wo->setName("pulley1");
        wo->set_radius(.5);
        testWrapObject(wo);
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapSphere");
    }
    try {
        auto* wo = new WrapEllipsoid();
        wo->setName("pulley1");
        wo->set_dimensions(Vec3(.5));
        testWrapObject(wo);
    }
    catch (const std::exception& e) {
        std::cout << "Exception: " << e.what() << std::endl;
        failures.push_back("TestWrapEllipsoid");
    }


    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done" << endl;
    return 0;
}

void testWrapObject(WrapObject* wrapObject)
{
    const double r = 0.5;
    Model model;
    model.setName("test"+wrapObject->getConcreteClassName());

    auto& ground = model.updGround();
    auto body = new OpenSim::Body("body", 1, Vec3(0), Inertia(0.1, 0.1, 0.01));
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
        appendNewPathPoint("origin", ground, Vec3(r, r, 0));
    spring1->updGeometryPath().
        appendNewPathPoint("insert", *body, Vec3(-r, r, 0));
    spring1->updGeometryPath().addPathWrap(*wObj);

    model.addComponent(spring1);

    model.finalizeConnections();
    model.setUseVisualizer(true);
    model.print(wObj->getConcreteClassName()+"Analytical.osim");
    SimTK::State& s = model.initSystem();
    auto& coord = joint->updCoordinate();
    const CoordinateSet& cset = model.getCoordinateSet();
    int nsteps = 100;
    for (int i = 0; i < nsteps; ++i) {
        
        coord.setValue(s, i*SimTK::Pi/(2*nsteps));
        model.realizeVelocity(s);

        model.getVisualizer().show(s);

        double ma1 = spring1->computeMomentArm(s, coord);

        ASSERT_EQUAL<double>(-r, ma1, SimTK::Eps);

        double len1 = spring1->getLength(s);
        if (i== 0) {
            std::cout << "i=" << i << "ma=" << ma1 << "len=" << len1 << std::endl;
            ASSERT_EQUAL<double>(len1, 1, SimTK::Eps);
        }
        if (i == nsteps - 1) {
            std::cout << "i=" << i << "ma=" << ma1 << "len=" << len1 << std::endl;
            std::cout << 1 + .25 * 2 * SimTK::Pi * r << std::endl;
            ASSERT_EQUAL<double>(len1, 1 + .25 * 2 * SimTK::Pi * r, .01);
        }
    }
}

