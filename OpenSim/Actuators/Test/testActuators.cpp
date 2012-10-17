/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testActuators.cpp                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application acts as an example for utilizing the MySpring actuator.
 */


#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/TorqueActuator.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/SimbodyEngine/PinJoint.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldConstraint.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Simulation/Control/ControlSet.h>
#include <OpenSim/Common/Constant.h>
#include <OpenSim/Simulation/Control/ControlLinear.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		// Create a new OpenSim model
		Model osimModel;
		osimModel.setName("osimModel");
		
		// Get the ground body
		OpenSim::Body& ground = osimModel.getGroundBody();
		//ground.addDisplayGeometry("ground.vtp");
	}
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }

//#ifdef WIN32
//	QueryPerformanceCounter(&stop);
//	double duration = (double)(stop.QuadPart-start.QuadPart)/(double)frequency.QuadPart;
//	std::cout << "main() routine time = " << (duration*1.0e3) << " milliseconds" << std::endl;
//#endif
//	osimModel.disownAllComponents();

	cout << "Done" << endl;
	return 0;
}
