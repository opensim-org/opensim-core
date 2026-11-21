/* -------------------------------------------------------------------------- *
 *                      OpenSim:  testWrapObject.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Ajay Seth, Ayman Habib, Adam Kewley                             *
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

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Wrap/WrapObject.h>
#include <OpenSim/Simulation/Wrap/WrapCylinder.h>
#include <OpenSim/Simulation/Wrap/WrapSphere.h>
#include <OpenSim/Simulation/Wrap/WrapEllipsoid.h>

#include <catch2/catch_all.hpp>

using namespace OpenSim;

 TEST_CASE("WrapObject update from XMLNode version 30515") {
    // In XMLDocument version 30515, we converted VisibleObject, color and
    // display_preference properties to Appearance properties.
    XMLDocument doc("testWrapObject_updateFromXMLNode30515.osim");

    // Make sure this test is not weakened by the model in the repository being
    // updated.
    SimTK_TEST(doc.getDocumentVersion() == 20302);
    Model model("testWrapObject_updateFromXMLNode30515.osim");
    model.print("testWrapObject_updateFromXMLNode30515_updated.osim");
    const auto& wrapObjSet = model.getGround().getWrapObjectSet();

    // WrapSphere has:
    //   display_preference = 1
    //   color = default
    //   VisibleObject display_preference = 0
    {
        const auto& sphere = wrapObjSet.get("wrapsphere");
        SimTK_TEST(!sphere.get_Appearance().get_visible());
        SimTK_TEST_EQ(sphere.get_Appearance().get_color(), SimTK::Cyan);
        SimTK_TEST(sphere.get_Appearance().get_representation() == 
                VisualRepresentation::DrawPoints /* == 1 */);
    }

    // WrapCylinder has:
    //   display_preference = 0
    //   color = default
    //   VisibleObject display_preference = 4 
    {
        const auto& cyl = wrapObjSet.get("wrapcylinder");
        // The outer display_preference overrides the inner one.
        SimTK_TEST(!cyl.get_Appearance().get_visible());
        SimTK_TEST_EQ(cyl.get_Appearance().get_color(), SimTK::Cyan);
        SimTK_TEST(cyl.get_Appearance().get_representation() == 
                VisualRepresentation::DrawSurface /* == 3 */);
    }

    // WrapEllipsoid has:
    //   display_preference = 2
    //   color = 1 0.5 0
    //   VisibleObject display_preference = 3
    {
        const auto& ellipsoid = wrapObjSet.get("wrapellipsoid");
        SimTK_TEST(ellipsoid.get_Appearance().get_visible());
        SimTK_TEST_EQ(ellipsoid.get_Appearance().get_color(), 
                SimTK::Vec3(1, 0.5, 0));
        // Outer display_preference overrides inner one.
        SimTK_TEST(ellipsoid.get_Appearance().get_representation() == 
                VisualRepresentation::DrawWireframe /* == 2 */);
    }
}

TEST_CASE("Model scaling with frameless WrapObjects does not segfault") {
    // reproduction for #3465
    //
    // effectively, if code tries to use a `WrapObject` outside
    // of a `PhysicalFrame` then the code in this test will segfault
    // without the fix because `WrapObject` will contain nullptrs
    // that are usually "fixed" by the parent `PhysicalFrame`
    //
    // the "proper" fix for this is to (e.g.) use the socket API but
    // this was avoided in #3465, which just focuses on downgrading
    // the segfault

    try {
        OpenSim::Model m;
        m.addComponent(new OpenSim::WrapCylinder{});
        m.buildSystem();
        SimTK::State& state = m.initializeState();
        m.scale(state, OpenSim::ScaleSet{}, true);
    } catch (const OpenSim::Exception&) {
        // the fix in #3465 only ensures no runtime segfaults may
        // occur - it does not guarantee that `WrapObject`s are
        // usable outside of their usual usage (i.e. as children
        // of `PhysicalFrame`s)
    }
}
