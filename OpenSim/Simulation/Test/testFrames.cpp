/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testFrames.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

//==============================================================================
//
//  Tests Include:
//      1. Body
//      2. PhysicalOffsetFrame
//      
//     Add tests here as Frames are added to OpenSim
//
//==============================================================================
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/OffsetFrame.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testBody();
void testOffsetFrameOnBody();
void testOffsetFrameOnBodySerialize();
void testOffsetFrameOnOffsetFrame();
void testStationOnFrame();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testBody(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testBody");
    }
        
    try { testOffsetFrameOnBody(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testOffsetFrameOnBody");
    }

    try { testOffsetFrameOnBodySerialize(); }
    catch (const std::exception& e){
        cout << e.what() << endl; 
        failures.push_back("testOffsetFrameOnBodySerialize");
    }
    
    try { testOffsetFrameOnOffsetFrame(); }
    catch (const std::exception& e){
        cout << e.what() << endl;
        failures.push_back("testOffsetFrameOnOffsetFrame");
    }

    try { testStationOnFrame(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testStationOnFrame");
    }

    if (!failures.empty()) {
        cout << "Done, with failure(s): " << failures << endl;
        return 1;
    }

    cout << "Done. All cases passed." << endl;

    return 0;
}

//==============================================================================
// Test Cases
//==============================================================================

void testBody()
{
    cout << "Running testBody" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    SimTK::State& st = dPendulum->initSystem();
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = dPendulum->getCoordinateSet().get("q1");
        coord.setValue(st, radAngle);
        const SimTK::Transform& xform = rod1.getGroundTransform(st);
        // The transform should give a translation of .353553, .353553, 0.0 since 0.353553 = .5 /sqr(2)
        SimTK::Vec3 p_known(0.5*std::sin(radAngle), -0.5*std::cos(radAngle), 0.0);
        ASSERT_EQUAL(p_known, xform.p(), SimTK::Vec3(SimTK::Eps),
            __FILE__, __LINE__, "testBody() failed");
        // The rotation part is a pure bodyfixed Z-rotation by radAngle.
        SimTK::Vec3 angles = xform.R().convertRotationToBodyFixedXYZ();
        SimTK::Vec3 angs_known(0, 0, radAngle);
        ASSERT_EQUAL(angs_known, angles, SimTK::Vec3(SimTK::Eps), 
            __FILE__, __LINE__, "testBody() failed");
    }
}

void testOffsetFrameOnBody()
{
    SimTK::Vec3 tolerance(SimTK::Eps);

    cout << "Running testOffsetFrameOnBody" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");


    SimTK::Transform relX;
    //offset position by some random vector
    relX.setP(SimTK::Vec3(1.2, 2.5, 3.3));
    // rotate the frame 60 degs about some random direction
    SimTK::Vec3 angs_known(0.33, 0.22, 0.11);
    relX.updR().setRotationToBodyFixedXYZ(angs_known);
    PhysicalOffsetFrame* atOriginFrame = new PhysicalOffsetFrame(rod1, relX);
    dPendulum->addFrame(atOriginFrame);
    SimTK::State& s = dPendulum->initSystem();
    const SimTK::Transform& rod1InG = rod1.getGroundTransform(s);
    const SimTK::Transform& offsetInG = atOriginFrame->getGroundTransform(s);

    // Compute the offset of these frames in ground
    SimTK::Transform deltaX = ~rod1InG*offsetInG;
    SimTK::Vec3 angles = deltaX.R().convertRotationToBodyFixedXYZ();

    // Offsets should be identical expressed in ground or in the Body
    ASSERT_EQUAL(relX.p(), deltaX.p(), tolerance,
        __FILE__, __LINE__, "testOffsetFrameOnBody() failed");
    ASSERT_EQUAL(angs_known, angles, tolerance,
        __FILE__, __LINE__, "testOffsetFrameOnBody() failed");
    // make sure that this OffsetFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == atOriginFrame->getMobilizedBodyIndex(),
        __FILE__, __LINE__, "testOffsetFrameOnBody() failed");
}

void testOffsetFrameOnOffsetFrame()
{
    SimTK::Vec3 tolerance(SimTK::Eps);

    cout << "Running testOffsetFrameOnOffsetFrame" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    
    SimTK::Transform relX;
    //offset position by some random vector
    relX.setP(SimTK::Vec3(1.2, 2.5, 3.3));
    // rotate the frame 
    relX.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(0.33, 0.22, 0.11));
    PhysicalOffsetFrame* atOriginFrame = new PhysicalOffsetFrame(rod1, relX);
    dPendulum->addFrame(atOriginFrame);

    //connect a second frame to the first OffsetFrame without any offset
    PhysicalOffsetFrame* secondFrame = atOriginFrame->clone();
    secondFrame->setParentFrame(*atOriginFrame);
    relX.setP(SimTK::Vec3(3.3, 2.2, 1.1));
    relX.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(1.5, -0.707, 0.5));
    secondFrame->setOffsetTransform(relX);
    dPendulum->addFrame(secondFrame);

    SimTK::State& s = dPendulum->initSystem();

    const Frame& base = secondFrame->findBaseFrame();
    SimTK::Transform XinBase = secondFrame->findTransformInBaseFrame();

    const SimTK::Transform& rod1InG = rod1.getGroundTransform(s);
    const SimTK::Transform& offsetInG = secondFrame->getGroundTransform(s);

    SimTK::Vec3 angs_known = XinBase.R().convertRotationToBodyFixedXYZ();

    // Compute the offset of these frames in ground
    SimTK::Transform deltaX = ~rod1InG*offsetInG;
    SimTK::Vec3 angles = deltaX.R().convertRotationToBodyFixedXYZ();

    // Offsets should be identical expressed in ground or in the Body
    ASSERT_EQUAL(XinBase.p(), deltaX.p(), tolerance,
        __FILE__, __LINE__, "testOffsetFrameOnOffsetFrame() failed");
    ASSERT_EQUAL(angs_known, angles, tolerance,
        __FILE__, __LINE__, "testOffsetFrameOnOffsetFrame() failed");

    // make sure that this OffsetFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == secondFrame->getMobilizedBodyIndex(),
        __FILE__, __LINE__, "testOffsetFrameOnOffsetFrame() failed");
}

void testOffsetFrameOnBodySerialize()
{
    SimTK::Vec3 tolerance(SimTK::Eps);

    cout << "Running testOffsetFrameOnBodySerialize" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");

    SimTK::Transform relXform;
    relXform.setP(SimTK::Vec3(0.0, .5, 0.0));
    relXform.updR().setRotationFromAngleAboutAxis(SimTK::Pi/4.0, SimTK::ZAxis);

    PhysicalOffsetFrame* atOriginFrame = new PhysicalOffsetFrame(rod1, relXform);
    atOriginFrame->setName("myExtraFrame");
    dPendulum->addFrame(atOriginFrame);

    SimTK::State& s1 = dPendulum->initSystem();
    const SimTK::Transform& xformPre = atOriginFrame->getGroundTransform(s1);
    dPendulum->print("double_pendulum_extraFrame.osim");
    // now read the model from file
    Model* dPendulumWFrame = new Model("double_pendulum_extraFrame.osim");
    SimTK::State& s2 = dPendulumWFrame->initSystem();
    ASSERT(*dPendulum == *dPendulumWFrame);

    const PhysicalFrame* myExtraFrame = dynamic_cast<const PhysicalFrame*> (&dPendulumWFrame->getFrameSet().get("myExtraFrame"));
    ASSERT(*atOriginFrame == *myExtraFrame);

    const SimTK::Transform& xformPost = myExtraFrame->getGroundTransform(s2);
    ASSERT_EQUAL(xformPost.p(), xformPre.p(), tolerance, __FILE__, __LINE__,
        "testOffsetFrameOnBodySerialize() failed");
    ASSERT_EQUAL(xformPost.R().convertRotationToBodyFixedXYZ(), 
        xformPre.R().convertRotationToBodyFixedXYZ(), tolerance,
        __FILE__, __LINE__, "testOffsetFrameOnBodySerialize() failed");
    // make sure that this OffsetFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == myExtraFrame->getMobilizedBodyIndex(), 
        __FILE__, __LINE__, "testOffsetFrameOnBodySerialize() failed");
}

void testStationOnFrame()
{
    SimTK::Vec3 tolerance(SimTK::Eps);

    cout << "Running testStationOnFrame" << endl;

    Model* dPendulum = new Model("double_pendulum.osim");
    // Get "rod1" frame
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    const SimTK::Vec3& com = rod1.get_mass_center();
    // Create station aligned with rod1 com in rod1_frame
    Station* myStation = new Station();
    myStation->set_location(com);
    myStation->updConnector<PhysicalFrame>("reference_frame").set_connected_to_name("rod1");
    dPendulum->addModelComponent(myStation);
    // myStation should coinicde with com location of rod1 in ground
    SimTK::State& st = dPendulum->initSystem();
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = dPendulum->getCoordinateSet().get("q1");
        coord.setValue(st, radAngle);

        SimTK::Vec3 comInGround = myStation->findLocationInFrame(st, dPendulum->getGroundBody());
        SimTK::Vec3 comBySimbody(0.);
        dPendulum->getSimbodyEngine().getPosition(st, rod1, com, comBySimbody);
        ASSERT_EQUAL(comInGround, comBySimbody, tolerance, __FILE__, __LINE__,
            "testStationOnFrame() failed");
    }
}
