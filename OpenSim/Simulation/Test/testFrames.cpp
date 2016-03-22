/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testFrames.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
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

/*=============================================================================
 
 Tests Include:
 1. Body
 2. PhysicalOffsetFrame on a Body computations
 3. PhysicalOffsetFrame on a Body serialization
 4. PhysicalOffsetFrame on a PhysicalOffsetFrame computations
 5. Station on a PhysicalFrame computations
 
 Add tests here as Frames are added to OpenSim
 
 //=============================================================================*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;
using SimTK::Transform;

void testBody();
void testPhysicalOffsetFrameOnBody();
void testPhysicalOffsetFrameOnBodySerialize();
void testPhysicalOffsetFrameOnPhysicalOffsetFrame();
void testFilterByFrameType();
void testStationOnFrame();
void testStationInGround();

class OrdinaryOffsetFrame : public OffsetFrame < Frame > {
    OpenSim_DECLARE_CONCRETE_OBJECT(OrdinaryOffsetFrame, OffsetFrame<Frame>);
public:
    OrdinaryOffsetFrame() : OffsetFrame() {}
    virtual ~OrdinaryOffsetFrame() {}
    
    OrdinaryOffsetFrame(const Frame& parent, const SimTK::Transform& offset) :
    OffsetFrame(parent, offset) {}
};


int main()
{
    SimTK::Array_<std::string> failures;
    
    try { testBody(); }
    catch (const std::exception& e){
        cout << e.what() <<endl; failures.push_back("testBody");
    }
    
    try { testPhysicalOffsetFrameOnBody(); }
    catch (const std::exception& e){
        cout << e.what() <<endl;
        failures.push_back("testPhysicalOffsetFrameOnBody");
    }
    
    try { testPhysicalOffsetFrameOnBodySerialize(); }
    catch (const std::exception& e){
        cout << e.what() << endl;
        failures.push_back("testPhysicalOffsetFrameOnBodySerialize");
    }
    
    try { testPhysicalOffsetFrameOnPhysicalOffsetFrame(); }
    catch (const std::exception& e){
        cout << e.what() << endl;
        failures.push_back("testPhysicalOffsetFrameOnPhysicalOffsetFrame");
    }
    
    try { testFilterByFrameType(); }
    catch (const std::exception& e){
        cout << e.what() << endl;
        failures.push_back("testFilterByFrameType");
    }
    
    try { testStationOnFrame(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testStationOnFrame");
    }
    
    try { testStationInGround(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testStationInGround");
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
    cout << "\nRunning testBody" << endl;
    Model* pendulum = new Model("double_pendulum.osim");
    
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    SimTK::State& s = pendulum->initSystem();
    
    const SimTK::DefaultSystemSubsystem& dsys =
    pendulum->getSystem().getDefaultSubsystem();
    
    std::clock_t now = std::clock();
    std::clock_t after = now;
    std::clock_t lookup_time = 0;
    
    //move the pendulum through a range of motion of 0 to 90 degrees
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = pendulum->getCoordinateSet().get("q1");
        coord.setValue(s, radAngle);
        
        const SimTK::Transform& xform = rod1.getGroundTransform(s);
        
        now = std::clock();
        for (int i = 0; i < 1000; ++i){
            const SimTK::Transform& xform1 = rod1.getGroundTransform(s);
        }
        after = std::clock();
        lookup_time += (after-now);
        
        // The transform should give a translation of .353553, .353553, 0.0
        SimTK::Vec3 p_known(0.5*sin(radAngle), -0.5*cos(radAngle), 0.0);
        ASSERT_EQUAL(p_known, xform.p(), SimTK::Vec3(SimTK::Eps),
                     __FILE__, __LINE__,
                     "testBody(): incorrect rod1 location in ground.");
        // The rotation part is a pure body-fixed Z-rotation by radAngle.
        SimTK::Vec3 angles = xform.R().convertRotationToBodyFixedXYZ();
        SimTK::Vec3 angs_known(0, 0, radAngle);
        ASSERT_EQUAL(angs_known, angles, SimTK::Vec3(SimTK::Eps),
                     __FILE__, __LINE__,
                     "testBody(): incorrect rod1 orientation in ground.");
    }
    cout << "get transform access time = " << 1e3*lookup_time << "ms" << endl;
}

void testPhysicalOffsetFrameOnBody()
{
    SimTK::Vec3 tolerance(SimTK::Eps);
    
    cout << "\nRunning testOffsetFrameOnBody" << endl;
    Model* pendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    
    // The offset transform on the rod body
    SimTK::Transform X_RO;
    // offset position by some random vector
    X_RO.setP(SimTK::Vec3(1.2, 2.5, 3.3));
    // rotate the frame by some non-planar rotation
    SimTK::Vec3 angs_known(0.33, 0.22, 0.11);
    X_RO.updR().setRotationToBodyFixedXYZ(angs_known);
    
    PhysicalOffsetFrame* offsetFrame = new PhysicalOffsetFrame(rod1, X_RO);
    pendulum->addFrame(offsetFrame);
    
    SimTK::State& s = pendulum->initSystem();
    const SimTK::Transform& X_GR = rod1.getGroundTransform(s);
    const SimTK::Transform& X_GO = offsetFrame->getGroundTransform(s);
    
    // Compute the offset transform based on frames expressed in ground
    SimTK::Transform X_RO_2 = ~X_GR*X_GO;
    SimTK::Vec3 angles = X_RO_2.R().convertRotationToBodyFixedXYZ();
    
    // Offsets should be identical expressed in ground or in the Body
    ASSERT_EQUAL(X_RO.p(), X_RO_2.p(), tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect expression of offset in ground.");
    ASSERT_EQUAL(angs_known, angles, tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect expression of offset in ground.");
    // make sure that this PhysicalOffsetFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == offsetFrame->getMobilizedBodyIndex(),
           __FILE__, __LINE__,
           "testPhysicalOffsetFrameOnBody(): incorrect MobilizedBodyIndex");
    
    Transform X_RO_3 = offsetFrame->findTransformBetween(s, rod1);
    SimTK::Vec3 angles3 = X_RO_3.R().convertRotationToBodyFixedXYZ();
    // Transform should be identical to the original offset
    ASSERT_EQUAL(X_RO.p(), X_RO_3.p(), tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect transform between offset and rod.");
    ASSERT_EQUAL(angs_known, angles3, tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect transform between offset and rod.");
    
    SimTK::Vec3 f_R(10.1, 20.2, 30.3);
    SimTK::Vec3 f_RG = rod1.expressVectorInAnotherFrame(s, f_R,
                                                        pendulum->getGround());
    
    ASSERT_EQUAL(f_R.norm(), f_RG.norm(), tolerance(0),
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect re-expression of vector.");
    
    SimTK::Vec3 f_RO = rod1.expressVectorInAnotherFrame(s, f_R, *offsetFrame);
    ASSERT_EQUAL(f_R.norm(), f_RO.norm(), tolerance(0),
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect re-expression of vector.");
    
    SimTK::Vec3 p_R(0.333, 0.222, 0.111);
    SimTK::Vec3 p_G =
    rod1.findLocationInAnotherFrame(s, p_R, pendulum->getGround());
    SimTK::Vec3 p_G_2 =
    rod1.getMobilizedBody().findStationLocationInGround(s, p_R);
    
    ASSERT_EQUAL(p_G_2, p_G, tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBody(): incorrect point location in ground.");
}

void testPhysicalOffsetFrameOnPhysicalOffsetFrame()
{
    SimTK::Vec3 tolerance(SimTK::Eps);
    
    cout << "\nRunning testPhysicalOffsetFrameOnPhysicalOffsetFrame" << endl;
    Model* pendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    
    SimTK::Transform X_RO;
    //offset position by some random vector
    X_RO.setP(SimTK::Vec3(1.2, 2.5, 3.3));
    // rotate the frame
    X_RO.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(0.33, 0.22, 0.11));
    PhysicalOffsetFrame* offsetFrame = new PhysicalOffsetFrame(rod1, X_RO);
    offsetFrame->setName("first");
    pendulum->addFrame(offsetFrame);
    
    //connect a second frame to the first PhysicalOffsetFrame
    PhysicalOffsetFrame* secondFrame = offsetFrame->clone();
    secondFrame->setName("second");
    secondFrame->setParentFrame(*offsetFrame);
    X_RO.setP(SimTK::Vec3(3.3, 2.2, 1.1));
    X_RO.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(1.5, -0.707, 0.5));
    secondFrame->setOffsetTransform(X_RO);
    pendulum->addFrame(secondFrame);
    
    SimTK::State& s = pendulum->initSystem();
    
    const Frame& base = secondFrame->findBaseFrame();
    SimTK::Transform XinBase = secondFrame->findTransformInBaseFrame();
    
    const SimTK::Transform& X_GR = rod1.getGroundTransform(s);
    const SimTK::Transform& X_GO = secondFrame->getGroundTransform(s);
    
    SimTK::Vec3 angs_known = XinBase.R().convertRotationToBodyFixedXYZ();
    
    // Compute the offset of these frames in ground
    SimTK::Transform X_RO_2 = ~X_GR*X_GO;
    SimTK::Vec3 angles = X_RO_2.R().convertRotationToBodyFixedXYZ();
    
    // Offsets should be identical expressed in ground or in the Body
    ASSERT_EQUAL(XinBase.p(), X_RO_2.p(), tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnPhysicalOffsetFrame(): incorrect expression of offset in ground.");
    ASSERT_EQUAL(angs_known, angles, tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnPhysicalOffsetFrame(): incorrect expression of offset in ground.");
    
    // make sure that this PhysicalOffsetFrame knows that it is rigidly fixed to the
    // same MobilizedBody as Body rod1
    ASSERT(rod1.getMobilizedBodyIndex() == secondFrame->getMobilizedBodyIndex(),
           __FILE__, __LINE__,
           "testPhysicalOffsetFrameOnPhysicalOffsetFrame(): incorrect MobilizedBodyIndex");
    
    // test base Frames are identical
    const Frame& baseRod = rod1.findBaseFrame();
    ASSERT(base == baseRod, __FILE__, __LINE__,
           "testPhysicalOffsetFrameOnPhysicalOffsetFrame(): incorrect base frame for PhysicalOffsetFrame");
    const Frame& base1 = offsetFrame->findBaseFrame();
    ASSERT(base1 == base, __FILE__, __LINE__,
           "testPhysicalOffsetFrameOnPhysicalOffsetFrame(): incorrect base frames for PhysicalOffsetFrame");
}

void testPhysicalOffsetFrameOnBodySerialize()
{
    SimTK::Vec3 tolerance(SimTK::Eps);
    
    cout << "\nRunning testPhysicalOffsetFrameOnBodySerialize" << endl;
    Model* pendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    
    SimTK::Transform X_RO;
    X_RO.setP(SimTK::Vec3(0.0, .5, 0.0));
    X_RO.updR().setRotationFromAngleAboutAxis(SimTK::Pi/4.0, SimTK::ZAxis);
    
    PhysicalOffsetFrame* offsetFrame = new PhysicalOffsetFrame(rod1, X_RO);
    offsetFrame->setName("myExtraFrame");
    pendulum->addFrame(offsetFrame);
    
    SimTK::State& s1 = pendulum->initSystem();
    const SimTK::Transform& X_GO_1 = offsetFrame->getGroundTransform(s1);
    pendulum->print("double_pendulum_extraFrame.osim");
    // now read the model from file
    Model* pendulumWFrame = new Model("double_pendulum_extraFrame.osim");
    SimTK::State& s2 = pendulumWFrame->initSystem();
    ASSERT(*pendulum == *pendulumWFrame);
    
    const PhysicalFrame& myExtraFrame =
    dynamic_cast<const PhysicalFrame&>(pendulumWFrame->getComponent("myExtraFrame"));
    ASSERT(*offsetFrame == myExtraFrame);
    
    const SimTK::Transform& X_GO_2 = myExtraFrame.getGroundTransform(s2);
    ASSERT_EQUAL(X_GO_2.p(), X_GO_1.p(), tolerance, __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBodySerialize(): incorrect expression of offset in ground.");
    ASSERT_EQUAL(X_GO_2.R().convertRotationToBodyFixedXYZ(),
                 X_GO_1.R().convertRotationToBodyFixedXYZ(), tolerance,
                 __FILE__, __LINE__,
                 "testPhysicalOffsetFrameOnBodySerialize(): incorrect expression of offset in ground.");
    // verify that PhysicalOffsetFrame shares the same underlying MobilizedBody as rod1
    ASSERT(rod1.getMobilizedBodyIndex() == myExtraFrame.getMobilizedBodyIndex(),
           __FILE__, __LINE__,
           "testPhysicalOffsetFrameOnBodySerialize(): incorrect MobilizedBodyIndex");
}

void testFilterByFrameType()
{
    // Previous model with a PhysicalOffsetFrame attached to rod1
    Model* pendulumWFrame = new Model("double_pendulum_extraFrame.osim");
    
    // Create an ordinary (non-physical) OffsetFrame attached to rod2
    const OpenSim::Body& rod2 = pendulumWFrame->getBodySet().get("rod2");
    SimTK::Transform X_RO_2;
    X_RO_2.setP(SimTK::Vec3(0.1, 0.22, 0.333));
    X_RO_2.updR().setRotationToBodyFixedXYZ(SimTK::Vec3(1.2, -0.7, 0.48));
    OrdinaryOffsetFrame* anOffset =
    new OrdinaryOffsetFrame(rod2, X_RO_2);
    
    // add OffsetFrame to the model
    pendulumWFrame->addFrame(anOffset);
    
    std::cout << "\nList all Frames in the model." << std::endl;
    int i = 0;
    for (auto& component : pendulumWFrame->getComponentList<Frame>()) {
        std::cout << "frame[" << ++i << "] is " << component.getName()
        << " of type " << typeid(component).name() << std::endl;
    }
    
    ASSERT_EQUAL(9, i, 0, __FILE__, __LINE__,
                 "testFilterByFrameType failed to find the 9 Frames in the model.");
    
    i = 0;
    std::cout << "\nList all PhysicalFrames in the model." << std::endl;
    for (auto& component : pendulumWFrame->getComponentList<PhysicalFrame>()) {
        std::cout << "frame[" << ++i << "] is " << component.getName()
        << " of type " << typeid(component).name() << std::endl;
    }
    ASSERT_EQUAL(8, i, 0, __FILE__, __LINE__,
                 "testFilterByFrameType failed to find 6 PhysicalFrames.");
    
    i = 0;
    std::cout << "\nList all Bodies in the model." << std::endl;
    for (auto& component : pendulumWFrame->getComponentList<Body>()) {
        std::cout << "frame[" << ++i << "] is " << component.getName()
        << " of type " << typeid(component).name() << std::endl;
    }
    
    ASSERT_EQUAL(2, i, 0, __FILE__, __LINE__,
                 "testFilterByFrameType failed to find the 2 Bodies in the model.");
    
    
    i = 0;
    std::cout << "\nList the PhyscicalOffsetFrame in the model." << std::endl;
    for (auto& component :
         pendulumWFrame->getComponentList<PhysicalOffsetFrame>()) {
        std::cout << "frame[" << ++i << "] is " << component.getName()
        << " of type " << typeid(component).name() << std::endl;
    }
    ASSERT_EQUAL(5, i, 0, __FILE__, __LINE__,
                 "testFilterByFrameType failed to find the 3 PhyscicalOffsetFrame in the model.");
}




void testStationOnFrame()
{
    SimTK::Vec3 tolerance(SimTK::Eps);
    
    cout << "Running testStationOnFrame" << endl;
    
    Model* pendulum = new Model("double_pendulum.osim");
    // Get the frames for the two rods of the pendulum
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    const SimTK::Vec3& com1 = rod1.get_mass_center();
    
    // Create a station in each frame
    Station* stationInFrame1 = new Station();
    stationInFrame1->set_location(com1);
    stationInFrame1->updConnector<PhysicalFrame>("reference_frame").setConnecteeName("rod1");
    pendulum->addModelComponent(stationInFrame1);
    
    Station* stationInFrame2 = new Station();
    stationInFrame2->set_location(com1);
    stationInFrame2->updConnector<PhysicalFrame>("reference_frame").setConnecteeName("rod2");
    pendulum->addModelComponent(stationInFrame2);
    
    Station* stationInGround = new Station();
    stationInGround->set_location(com1);
    stationInGround->updConnector<PhysicalFrame>("reference_frame").setConnecteeName("ground");
    pendulum->addModelComponent(stationInGround);
    
    
    SimTK::Vec3 stationPosition;
    
    // Compare the vector between two stations, in two different frames.
    SimTK::State& s = pendulum->initSystem();
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = pendulum->getCoordinateSet().get("q1");
        coord.setValue(s, radAngle);
        
        // get the position of rod1's station in rod2 frame
        stationPosition = stationInFrame1->findLocationInFrame(s, pendulum->getBodySet().get("rod2") );
        // Set the location of the rod2's station.
        stationInFrame2->set_location(stationPosition);
        
        //get the position of rod2's station in ground frame
        stationPosition = stationInFrame2->findLocationInFrame(s, pendulum->getGround() );
        // Set the location of the ground's station
        stationInGround->set_location(stationPosition);
        
        //get the position of the ground's station in rod1
        stationPosition = stationInGround->findLocationInFrame(s, pendulum->getBodySet().get("rod1") );
        
        // now compare the original station location in rod1 (com) with the transformed station.
        SimTK_TEST_EQ(stationPosition, com1);
    }
}

void testStationInGround()
{
    SimTK::Vec3 tolerance(SimTK::Eps);
    
    cout << "Running testStationInGround" << endl;
    
    Model* pendulum = new Model("double_pendulum.osim");
    // Get "rod1" frame
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    const SimTK::Vec3& com = rod1.get_mass_center();
    // Create station aligned with rod1 com in rod1_frame
    Station* myStation = new Station();
    myStation->set_location(com);
    myStation->updConnector<PhysicalFrame>("reference_frame")
    .setConnecteeName("rod1");
    pendulum->addModelComponent(myStation);
    // myStation should coincide with com location of rod1 in ground
    SimTK::State& s = pendulum->initSystem();
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = pendulum->getCoordinateSet().get("q1");
        coord.setValue(s, radAngle);
        
        // Get the station in ground via Simbody
        SimTK::Vec3 comBySimbody = rod1.getMobilizedBody().findStationLocationInGround(s, com);
        
        // test findLocationInGround
        SimTK::Vec3 stationInGround = myStation->findLocationInGround(s);
        
        SimTK_TEST_EQ(stationInGround, comBySimbody);
        
    }
}
