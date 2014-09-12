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
//	Tests Include:
//      1. BodyFrame
//		2. FixedFrame
//		
//     Add tests here as Frames are added to OpenSim
//
//==============================================================================
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Simulation/osimSimulation.h>
#include <OpenSim/Analyses/osimAnalyses.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testBodyFrame();
void testFixedFrameOnBodyFrame();
void testFixedFrameOnBodyFrameSerialize();
void testStationOnFrame();

int main()
{
	SimTK::Array_<std::string> failures;

	try { testBodyFrame(); }
    catch (const std::exception& e){
		cout << e.what() <<endl; failures.push_back("testBodyFrame");
	}
		
	try { testFixedFrameOnBodyFrame(); }
    catch (const std::exception& e){
		cout << e.what() <<endl; failures.push_back("testFixedFrameOnBodyFrame");
	}

    try { testFixedFrameOnBodyFrameSerialize(); }
    catch (const std::exception& e){
        cout << e.what() << endl; failures.push_back("testFixedFrameOnBodyFrame");
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
	Model* dPendulum = new Model("double_pendulum.osim");
	const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
	SimTK::State& st = dPendulum->initSystem();
	for (double ang = 0; ang <= 90.0; ang += 10.){
		double radAngle = SimTK::convertDegreesToRadians(ang);
		const Coordinate& coord = dPendulum->getCoordinateSet().get("q1");
		coord.setValue(st, radAngle);
        SimTK::Transform xform = rod1.calcGroundTransform(st);
		// By construction the transform should gove a translation of .353553, .353553, 0.0 since 0.353553 = .5 /sqr(2)
		double dNorm = (xform.p() - SimTK::Vec3(0.5*std::sin(radAngle), -0.5*std::cos(radAngle), 0.)).norm();
		assert(dNorm < 1e-6);
		// The rotation part is a pure bodyfixed Z-rotation by radAngle.
		SimTK::Vec3 angles = xform.R().convertRotationToBodyFixedXYZ();
		assert(std::fabs(angles[0]) < 1e-6);
		assert(std::fabs(angles[1]) < 1e-6);
		assert(std::fabs(angles[2] - radAngle) < 1e-6);
	}
	return;
}

void testFixedFrameOnBodyFrame()
{
	Model* dPendulum = new Model("double_pendulum.osim");
	const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    FixedFrame* atOriginFrame = new FixedFrame(rod1);
	SimTK::Transform relXform;
	relXform.setP(SimTK::Vec3(0.0, .5, 0.0));
	relXform.updR().setRotationFromAngleAboutAxis(SimTK::Pi / 4.0, SimTK::CoordinateAxis(2));
	atOriginFrame->setTransform(relXform);
	dPendulum->addFrame(atOriginFrame);
	SimTK::State& st = dPendulum->initSystem();
    const SimTK::Transform rod1FrameXform = rod1.calcGroundTransform(st);
    SimTK::Transform xform = atOriginFrame->getGroundTransform(st);
	// xform should have 0.0 translation
	assert(xform.p().norm() < 1e-6);
	return;
}

void testFixedFrameOnBodyFrameSerialize()
{
    Model* dPendulum = new Model("double_pendulum.osim");
    const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
    FixedFrame* atOriginFrame = new FixedFrame(rod1);
    atOriginFrame->setName("myExtraFrame");
    SimTK::Transform relXform;
    relXform.setP(SimTK::Vec3(0.0, .5, 0.0));
    relXform.updR().setRotationFromAngleAboutAxis(SimTK::Pi / 4.0, SimTK::CoordinateAxis(2));
    atOriginFrame->setTransform(relXform);
    dPendulum->addFrame(atOriginFrame);
    SimTK::State& st1 = dPendulum->initSystem();
    SimTK::Transform xformPre = atOriginFrame->getGroundTransform(st1);
    dPendulum->print("double_pendulum_extraFrame.osim");
    // now read the model from file
    Model* dPendulumWFrame = new Model("double_pendulum_extraFrame.osim");
    SimTK::State& st2 = dPendulumWFrame->initSystem();
    const Frame& myExtraFrame = dPendulumWFrame->getFrameSet().get("myExtraFrame");
    SimTK::Transform xformPost = myExtraFrame.getGroundTransform(st2);
    assert((xformPost.p()- xformPre.p()).norm() < 1e-6);
    return;
}

void testStationOnFrame()
{
	Model* dPendulum = new Model("double_pendulum.osim");
	// Get "rod1" frame
	const OpenSim::Body& rod1 = dPendulum->getBodySet().get("rod1");
	const SimTK::Vec3& com = rod1.get_mass_center();
	// Create station aligned with rod1 com in rod1_frame
	Station* myStation = new Station();
	myStation->set_location(com);
	myStation->updConnector<RigidFrame>("reference_frame").set_connected_to_name("rod1");
	dPendulum->addModelComponent(myStation);
	// myStation should coinicde with com location of rod1 in ground
	SimTK::State& st = dPendulum->initSystem();
	for (double ang = 0; ang <= 90.0; ang += 10.){
		double radAngle = SimTK::convertDegreesToRadians(ang);
		const Coordinate& coord = dPendulum->getCoordinateSet().get("q1");
		coord.setValue(st, radAngle);
        SimTK::Vec3 comInGround = myStation->reexpressLocationInFrame(st, dPendulum->getGroundBody());
		SimTK::Vec3 comBySimbody(0.);
		dPendulum->getSimbodyEngine().getPosition(st, rod1, com, comBySimbody);
		assert((comInGround - comBySimbody).norm() < 1e-6);
	}
	return;
}

