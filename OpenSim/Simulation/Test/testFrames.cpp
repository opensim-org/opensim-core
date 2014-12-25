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
//      1. BodyFrame
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

void testBodyFrame();
void testOffsetFrameOnBodyFrame();
void testOffsetFrameOnBodyFrameSerialize();
void testOffsetFrameOnOffsetFrame();
void testStationOnFrame();

int main()
{
    SimTK::Array_<std::string> failures;

    try { testBodyFrame(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testBodyFrame");
    }
        
    try { testOffsetFrameOnBodyFrame(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testFixedFrameOnBodyFrame");
    }

    try { testOffsetFrameOnBodyFrameSerialize(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testFixedFrameOnBodyFrame");
    }
    
    try { testOffsetFrameOnOffsetFrame(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testFixedFrameOnFixedFrame");
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

void testBodyFrame()
{
    cout << "Running testBodyFrame" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    SimTK::State& st = dPendulum->initSystem();
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = dPendulum->getCoordinateSet().get("q1");
        coord.setValue(st, radAngle);
        SimTK::Transform xform = rod1.getGroundTransform(st);
        // By construction the transform should give a translation of .353553, .353553, 0.0 since 0.353553 = .5 /sqr(2)
        double dNorm = (xform.p() - SimTK::Vec3(0.5*std::sin(radAngle), -0.5*std::cos(radAngle), 0.)).norm();
        ASSERT(dNorm < 1e-6, __FILE__, __LINE__, "testBodyFrame() failed");
        // The rotation part is a pure bodyfixed Z-rotation by radAngle.
        SimTK::Vec3 angles = xform.R().convertRotationToBodyFixedXYZ();
        ASSERT(std::abs(angles[0]) < 1e-6, __FILE__, __LINE__, "testBodyFrame() failed");
        ASSERT(std::abs(angles[1]) < 1e-6, __FILE__, __LINE__, "testBodyFrame() failed");
        ASSERT(std::abs(angles[2] - radAngle) < 1e-6, __FILE__, __LINE__, "testBodyFrame() failed");
    }

    return;
}

void testOffsetFrameOnBodyFrame()
{
    cout << "Running testFixedFrameOnBodyFrame" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");

    SimTK::Transform relX;
    //offset position by some random vector
    relX.setP(SimTK::Vec3(1.2, 2.5, 3.3));
    // rotate the frame 60 degs about some random direction
    relX.updR().setRotationFromAngleAboutNonUnitVector(SimTK::Pi/3, SimTK::Vec3(3.0, 2.0, 1.0));
    PhysicalOffsetFrame* atOriginFrame = new PhysicalOffsetFrame(rod1, relX);
    dPendulum->addFrame(atOriginFrame);
    SimTK::State& s = dPendulum->initSystem();
    const SimTK::Transform& rod1InG = rod1.getGroundTransform(s);
    const SimTK::Transform& offsetInG = atOriginFrame->getGroundTransform(s);

    // Expressed in ground the translation offset shoul be preserved
    ASSERT_EQUAL((rod1InG.p() - offsetInG.p()).norm(), relX.p().norm(), SimTK::Eps,  __FILE__, __LINE__, "testFixedFrameOnBodyFrame() failed");
    // make sure that this FixedFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == atOriginFrame->getMobilizedBodyIndex(), __FILE__, __LINE__, "testFixedFrameOnBodyFrame() failed");
    return;
}

void testOffsetFrameOnOffsetFrame()
{
    cout << "Running testFixedFrameOnFrame" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    
    SimTK::Transform relXform;
    relXform.setP(SimTK::Vec3(0.0, .5, 0.0));
    relXform.updR().setRotationFromAngleAboutAxis(SimTK::Pi / 4.0, SimTK::CoordinateAxis(2));
    PhysicalOffsetFrame* atOriginFrame = new PhysicalOffsetFrame(rod1, relXform);
    dPendulum->addFrame(atOriginFrame);

    //connect a second frame to the first FixedFrame without any offset
    PhysicalOffsetFrame* secondFrame = atOriginFrame->clone();
    secondFrame->setParentFrame(*atOriginFrame);
    relXform.setP(SimTK::Vec3(0.0));
    secondFrame->setOffsetTransform(relXform);
    dPendulum->addFrame(secondFrame);

    SimTK::State& st = dPendulum->initSystem();
    const SimTK::Transform rod1FrameXform = rod1.getGroundTransform(st);
    SimTK::Transform xform = secondFrame->getGroundTransform(st);
    // xform should have 0.0 translation
    ASSERT(xform.p().norm() < 1e-6, __FILE__, __LINE__, "testFixedFrameOnFixedFrame() failed");
    // make sure that this FixedFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == secondFrame->getMobilizedBodyIndex(), __FILE__, __LINE__, "testFixedFrameOnFixedFrame() failed");
    return;
}

void testOffsetFrameOnBodyFrameSerialize()
{
    cout << "Running testFixedFrameOnBodyFrameSerialize" << endl;
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");

    SimTK::Transform relXform;
    relXform.setP(SimTK::Vec3(0.0, .5, 0.0));
    relXform.updR().setRotationFromAngleAboutAxis(SimTK::Pi / 4.0, SimTK::ZAxis);

    PhysicalOffsetFrame* atOriginFrame = new PhysicalOffsetFrame(rod1, relXform);
    atOriginFrame->setName("myExtraFrame");
    dPendulum->addFrame(atOriginFrame);

    SimTK::State& s1 = dPendulum->initSystem();
    const SimTK::Transform& xformPre = atOriginFrame->getGroundTransform(s1);
    dPendulum->print("double_pendulum_extraFrame.osim");
    // now read the model from file
    Model* dPendulumWFrame = new Model("double_pendulum_extraFrame.osim");
    SimTK::State& s2 = dPendulumWFrame->initSystem();
    const PhysicalFrame* myExtraFrame = dynamic_cast<const PhysicalFrame*> (&dPendulumWFrame->getFrameSet().get("myExtraFrame"));
    ASSERT(*atOriginFrame == *myExtraFrame);

    const SimTK::Transform& xformPost = myExtraFrame->getGroundTransform(s2);
    ASSERT((xformPost.p() - xformPre.p()).norm() < 1e-6, __FILE__, __LINE__, "testFixedFrameOnBodyFrameSerialized failed");
    // make sure that this FixedFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == myExtraFrame->getMobilizedBodyIndex(), __FILE__, __LINE__, "testFixedFrameOnBodyFrameSerialized() failed");
    return;
}

void testStationOnFrame()
{
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
        ASSERT((comInGround - comBySimbody).norm() < 1e-6, __FILE__, __LINE__, "testStationOnFrame() failed");
    }
    return;
}
