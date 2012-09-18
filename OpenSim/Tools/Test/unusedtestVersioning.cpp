/* -------------------------------------------------------------------------- *
 *                     OpenSim:  unusedtestVersioning.cpp                     *
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

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

#include "Simbody.h"

using namespace OpenSim;
using namespace std;

void testUpdateModel(string fileName);

int main()
{
	try {
		LoadOpenSimLibrary("osimActuators");
		LoadOpenSimLibrary("osimTools");
		//OpenSim::Object::setSerializeAllDefaults(true);
		testUpdateModel("snowboard10600.osim");
		cout << "Load and update model snowboard10600 version 10600: PASSED\n" << endl;
		testUpdateModel("gait2354_simbody.osim");
		cout << "Load and update model gait2354_simbody version 10901: PASSED\n" << endl;
		testUpdateModel("arm26.osim");
		cout << "Load and update model Arm26 version 10905: PASSED\n" << endl;
		testUpdateModel("Pendulum.osim");
		cout << "Load and update model Pendulum version 10905: PASSED\n" << endl;
		testUpdateModel("Neck3dof_point_constraint.osim"); 
		cout << "Load and update model Neck3dof_point_constraint version 20001: PASSED\n" << endl;
		testUpdateModel("BothLegs22.osim");
		cout << "Load and update model BothLegs22 version 20200: PASSED\n" << endl;

	}
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
	return 0;
}

void testUpdateModel(string fileName)
{
	Model model(fileName);
	//model.initSystem();
	string newName(fileName);
	newName=IO::replaceSubstring(newName, ".osim", "_latest.osim");
	model.print(newName);
	// Now make 2 XMLDocuments and compare them
	XMLDocument doc1(newName);
	XMLDocument doc2("std_"+newName);
	ASSERT(doc1.isEqualTo(doc2, 1e-4, false));
}