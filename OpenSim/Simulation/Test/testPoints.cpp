/* -------------------------------------------------------------------------- *
 *                          OpenSim:  testPoints.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2016 Stanford University and the Authors                *
 * Author(s): Ajay Seth, James Dunne                                          *
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
    1. Station
    2. Marker
    3. Stations on a Frame computations 
      
     Add tests here as Points are added to OpenSim

//=============================================================================*/
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/PhysicalOffsetFrame.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;
using SimTK::Transform;

void testStationOnBody();
void testStationOnOffsetFrame();

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

    try { testStationOnBody(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testStationOnBody");
    }

    try { testStationOnOffsetFrame(); }
    catch (const std::exception& e) {
        cout << e.what() << endl;
        failures.push_back("testStationOnOffsetFrame");
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

void testStationOnBody()
{
    SimTK::Vec3 tolerance(SimTK::Eps);

    cout << "Running testStationOnFrame" << endl;

    Model* pendulum = new Model("double_pendulum.osim");
    // Get "rod1" frame
    const OpenSim::Body& rod1 = pendulum->getBodySet().get("rod1");
    const SimTK::Vec3& com = rod1.get_mass_center();
    // Create station aligned with rod1 com in rod1_frame
    Station* myStation = new Station(rod1, com);
    pendulum->addModelComponent(myStation);
    // myStation should coincide with com location of rod1 in ground
    SimTK::State& s = pendulum->initSystem();
    for (double ang = 0; ang <= 90.0; ang += 10.){
        double radAngle = SimTK::convertDegreesToRadians(ang);
        const Coordinate& coord = pendulum->getCoordinateSet().get("q1");
        coord.setValue(s, radAngle);
        coord.setSpeedValue(s, radAngle*radAngle);

        pendulum->realizeAcceleration(s);

        SimTK::Vec3 comInGround =  myStation->getLocationInGround(s);
        SimTK::Vec3 comBySimbody = 
            rod1.getMobilizedBody().findStationLocationInGround(s, com);
        ASSERT_EQUAL(comInGround, comBySimbody, tolerance, __FILE__, __LINE__,
            "testStationOnBody(): failed to resolve station location in ground.");

        SimTK::Vec3 comVInGround = myStation->getVelocityInGround(s);
        SimTK::Vec3 comVBySimbody =
            rod1.getMobilizedBody().findStationVelocityInGround(s, com);
        ASSERT_EQUAL(comVInGround, comVBySimbody, tolerance, __FILE__, __LINE__,
            "testStationOnBody(): failed to resolve station velocity in ground.");

        SimTK::Vec3 comAInGround = myStation->getAccelerationInGround(s);
        SimTK::Vec3 comABySimbody =
            rod1.getMobilizedBody().findStationAccelerationInGround(s, com);
        ASSERT_EQUAL(comAInGround, comABySimbody, tolerance, __FILE__, __LINE__,
            "testStationOnBody(): failed to resolve station velocity in ground.");
    }
}

void testStationOnOffsetFrame()
{
    SimTK::Vec3 tolerance(SimTK::Eps);
    SimTK::MultibodySystem system;
    SimTK::SimbodyMatterSubsystem matter(system);
    SimTK::GeneralForceSubsystem forces(system);

    cout << "Running testStationOnOffsetFrame" << endl;

    Model* pendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod2 = pendulum->getBodySet().get("rod2");

    // Define and add a frame to the rod2 body
    SimTK::Transform X_RO;
    X_RO.setP(SimTK::Vec3(1.234, -0.2667, 0));
    X_RO.updR().setRotationFromAngleAboutAxis(SimTK::Pi/3.33 , SimTK::ZAxis);
    PhysicalOffsetFrame* offsetFrame = new PhysicalOffsetFrame(rod2, X_RO);
    offsetFrame->setName("myExtraFrame");
    pendulum->addFrame(offsetFrame);

    // Create station in the extra frame
    Station* myStation = new Station();
    const SimTK::Vec3 point(0.5, 1, -1.5);
    myStation->set_location(point);
    myStation->setReferenceFrame(*offsetFrame);
    pendulum->addModelComponent(myStation);

    // Initialize the the system
    SimTK::State state = pendulum->initSystem();

    // set the model coordinates and coordinate speeds
    pendulum->getCoordinateSet().get(0).setValue(state, 0.29);
    pendulum->getCoordinateSet().get(0).setSpeedValue(state, 0.1);
    pendulum->getCoordinateSet().get(1).setValue(state, -0.38);
    pendulum->getCoordinateSet().get(1).setSpeedValue(state, -0.13);

    // realize to accelerations
    pendulum->realizeAcceleration(state);

    // Get the frame's mobilized body
    const OpenSim::PhysicalFrame&  frame = myStation->getReferenceFrame();
    SimTK::MobilizedBody  mb = frame.getMobilizedBody();

    // Use simbody to get the location, velocity and acceleration in ground.
    SimTK::Vec3 l, v, a;
    // Need to map the point into the base frame to use MobilizedBody's
    // station methods otherwise we exclude the effect of the offset frame
    SimTK::Vec3 pointInBase = frame.findTransformInBaseFrame()*point;
    mb.findStationLocationVelocityAndAccelerationInGround(state, 
        pointInBase, l, v, a);

    // Compare Simbody values to values from Station
    SimTK_TEST_EQ(l, myStation->getLocationInGround(state));
    SimTK_TEST_EQ(v, myStation->getVelocityInGround(state));
    SimTK_TEST_EQ(a, myStation->getAccelerationInGround(state));
}
