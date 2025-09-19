/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testModelInterface.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Actuators/PointActuator.h>

#include <memory>

using namespace OpenSim;
using namespace std;

void testModelFinalizePropertiesAndConnections();
void testModelTopologyErrors();
void testDoesNotSegfaultWithUnusualConnections();

int main() {
    // The following model(s) contains Actuators that are registered when the
    // osimActuators library is loaded. But unless we call at least one
    // function defined in the osimActuators library, some linkers will omit
    // its dependency from the executable and it will not be loaded at
    // startup.
    { PointActuator t; }

    SimTK_START_TEST("testModelInterface");
        SimTK_SUBTEST(testModelFinalizePropertiesAndConnections);
        SimTK_SUBTEST(testModelTopologyErrors);
        SimTK_SUBTEST(testDoesNotSegfaultWithUnusualConnections);
    SimTK_END_TEST();
}


void testModelFinalizePropertiesAndConnections() 
{
        Model model("arm26.osim");

        model.printSubcomponentInfo();

        // all subcomponents are accounted for since Model constructor invokes
        // finalizeFromProperties().
        ASSERT(model.countNumComponents() > 0);

        // model must be up-to-date with its properties
        ASSERT(model.isObjectUpToDateWithProperties());

        // get writable access to Components contained in the model's Set
        auto& muscles = model.updMuscles();
        // to make edits, for example, muscles[0].upd_min_control() = 0.02;
        ASSERT(!model.isObjectUpToDateWithProperties());

        model.finalizeFromProperties();
        ASSERT(model.isObjectUpToDateWithProperties());

        // get writable access to another Set for the purpose of editing
        auto& bodies = model.updBodySet();
        // for example, bodies[1].upd_mass() = 0.05;
        ASSERT(!model.isObjectUpToDateWithProperties());

        model.finalizeFromProperties();
        ASSERT(model.isObjectUpToDateWithProperties());

        // make an edit through model's ComponentList access
        for (auto& body : model.updComponentList<Body>()) {
            body.upd_mass_center() = SimTK::Vec3(0);
            break;
        }
        ASSERT(!model.isObjectUpToDateWithProperties());

        SimTK::State dummy_state;

        ASSERT_THROW(ComponentHasNoSystem, model.getSystem());
        ASSERT_THROW(ComponentHasNoSystem, model.realizeDynamics(dummy_state));
        ASSERT_THROW(ComponentHasNoSystem, 
                     model.computeStateVariableDerivatives(dummy_state));
        // should not be able to create a Manager either
        ASSERT_THROW(ComponentHasNoSystem,
                     Manager manager(model));

        // get a valid System and corresponding state
        SimTK::State state = model.initSystem();
        Manager manager(model);
        state.setTime(0.0);

        // this should invalidate the System
        model.finalizeFromProperties();

        // verify that finalizeFromProperties() wipes out the underlying System
        ASSERT_THROW(ComponentHasNoSystem, model.getSystem());

        // should not be able to "trick" the manager into integrating a model
        // given a stale but compatible state
        ASSERT_THROW(Exception, manager.initialize(state));

        // once again, get a valid System and corresponding state
        state = model.initSystem();
        
        // Test for the effects of calling finalizeConnections() on the model
        // after initSystem() has been called.
        // In this case, there are no changes to the connections to be finalized.
        model.finalizeConnections();

        // verify that finalizeConnections() does not wipe out the underlying 
        // System when there are no changes to the connections
        auto& sys = model.getSystem();

        auto elbowInHumerus = new PhysicalOffsetFrame("elbow_in_humerus",
            model.getComponent<Body>("./bodyset/r_humerus"),
            SimTK::Transform(SimTK::Vec3(0, -0.33, 0)) );

        model.addComponent(elbowInHumerus);

        // establish the new offset frame as part of the model but not
        // used by any joints, constraints or forces
        state = model.initSystem();

        // update the elbow Joint and connect its socket to the new frame
        Joint& elbow = model.updComponent<Joint>("./jointset/r_elbow");
        elbow.connectSocket_parent_frame(*elbowInHumerus);

        // satisfy the new connections in the model
        model.finalizeConnections();

        // now finalizing the connections will invalidate the System because
        // a Component (the elbow Joint and its connection) was updated
        ASSERT_THROW(ComponentHasNoSystem, model.getSystem());

        // verify the new connection was made
        ASSERT(model.getComponent<Joint>("./jointset/r_elbow")
            .getParentFrame().getName() == "elbow_in_humerus");
}

void testModelTopologyErrors()
{
    Model model("arm26.osim");
    model.initSystem();

    // connect the shoulder joint from torso to ground instead of r_humerus
    // this is an invalid tree since the underlying PhysicalFrame in both
    // cases is ground.
    Joint& shoulder = model.updComponent<Joint>("./jointset/r_shoulder");
    const PhysicalFrame& shoulderOnHumerus = shoulder.getChildFrame();
    shoulder.connectSocket_child_frame(model.getGround());

    // both shoulder joint frames have Ground as the base frame
    ASSERT_THROW( JointFramesHaveSameBaseFrame,  model.initSystem() );

    // restore the previous frame connected to the shoulder
    shoulder.connectSocket_child_frame(shoulderOnHumerus);
    model.initSystem();

    // create and offset for the elbow joint in the humerus
    auto elbowInHumerus = new PhysicalOffsetFrame("elbow_in_humerus",
        model.getComponent<Body>("./bodyset/r_humerus"),
        SimTK::Transform(SimTK::Vec3(0, -0.33, 0)));

    model.addComponent(elbowInHumerus);

    // update the elbow to connect to the elbow in the humerus frame
    Joint& elbow = model.updComponent<Joint>("./jointset/r_elbow");
    const PhysicalFrame& elbowOnUlna = elbow.getChildFrame();
    elbow.connectSocket_child_frame(*elbowInHumerus);

    // both elbow joint frames have the humerus Body as the base frame
    ASSERT_THROW(JointFramesHaveSameBaseFrame, model.initSystem());

    // restore the ulna frame connected to the elbow
    elbow.connectSocket_child_frame(elbowOnUlna);
    model.initSystem();

    // introduce two offsets to simulate user error in defining the elbow
    // in the ulna by accidentally offsetting the wrong frame.
    auto elbowOnUlnaMistake = new PhysicalOffsetFrame("elbow_in_ulna",
        *elbowInHumerus,
        SimTK::Transform(SimTK::Vec3(0.1, -0.22, 0.03)) );
    model.addComponent(elbowOnUlnaMistake);
    elbow.connectSocket_child_frame(*elbowOnUlnaMistake);

    // both elbow joint frames still have the humerus Body as the base frame
    ASSERT_THROW(JointFramesHaveSameBaseFrame, model.initSystem());


    Model degenerate;
    auto frame1 = new PhysicalOffsetFrame("frame1", degenerate.getGround(),
        SimTK::Transform(SimTK::Vec3(0, -0.1, 0)));
    auto frame2 = new PhysicalOffsetFrame("frame2", degenerate.getGround(),
        SimTK::Transform(SimTK::Vec3(0, 0.2, 0)));

    degenerate.addComponent(frame1);
    degenerate.addComponent(frame2);

    auto joint1 = new PinJoint();
    joint1->setName("joint1");
    joint1->connectSocket_parent_frame(*frame1);
    // intentional mistake to connect to the same frame
    joint1->connectSocket_child_frame(*frame1);

    // Expose infinite recursion in the case that Joint explicitly invokes
    // finalizeConnections on its parent and child frames AND the Joint itself
    // is a subcomponent of either the parent or child frame. This test is why
    // the Joint cannot resolve whether two frames have the same base frame.
    frame1->addComponent(joint1);
    
    // Parent and child frames of a Joint cannot be the same frame
    ASSERT_THROW(JointFramesAreTheSame, 
        joint1->finalizeConnections(degenerate));

    // Joint should also throw JointFramesAreTheSame before
    // Model throws JointFramesHaveSameBaseFrame
    ASSERT_THROW(JointFramesAreTheSame,
        joint1->finalizeConnections(degenerate));

    // now test with child as frame2 (offset) that shares the same base
    joint1->connectSocket_child_frame(*frame2);

    ASSERT_THROW(JointFramesHaveSameBaseFrame, degenerate.initSystem());
}

void testDoesNotSegfaultWithUnusualConnections()
{
    // automated reproduction for bug reported in #3299
    //
    // effectively, the multibody graph making procedure contained
    // an edge-case in which an unusual model graph (reproduced by
    // the code below) could cause it to segfault

    try
    {
        OpenSim::Model m;
        OpenSim::PhysicalFrame const& groundFrame = m.getGround();

        std::unique_ptr<OpenSim::Body> pelvis{new OpenSim::Body{"pelvis", 1.0, SimTK::Vec3{}, SimTK::Inertia{}}};
        OpenSim::PhysicalFrame const& pelvisFrame = *pelvis;
        m.addBody(pelvis.release());

        m.addJoint(new OpenSim::PinJoint{"ground_pelvis", groundFrame, pelvisFrame});
        m.addJoint(new OpenSim::PinJoint{"pelvis_ground", pelvisFrame, groundFrame});

        m.finalizeFromProperties();
        m.finalizeConnections();
    }
    catch (OpenSim::Exception const&)
    {
        // this test only ensures that the #3299 repro doesn't
        // _segfault_ - the method is still permitted to throw
        // a runtime exception (for now... ;))
    }
}
