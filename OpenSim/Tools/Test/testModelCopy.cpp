/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testModelCopy.cpp                         *
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

#include <iostream>
#include <stdint.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Exception.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "SimTKsimbody.h"
#include "SimTKmath.h"


using namespace OpenSim;
using namespace std;

void testCopyModel(string fileName);

int main()
{
	try {
		LoadOpenSimLibrary("osimActuators");
		testCopyModel("arm26.osim");
		//testCopyModel("snowboard.osim");
		testCopyModel("Neck3dof_point_constraint.osim");
	}
	catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
	return 0;
}

void testCopyModel(string fileName)
{
	size_t mem1 = getCurrentRSS( );
	cout << "Memory use BEFORE load, copy and init: " << mem1 << "Bytes." << endl;

	Model *model = new Model(fileName);
	SimTK::State defaultState = model->initSystem();
	Model *modelCopy = new Model(*model);
	// At this point properties should all match. assert that
	ASSERT(*model==*modelCopy);
	delete model;
	SimTK::State& defaultStateOfCopy = modelCopy->initSystem();
	// Compare state
	defaultState.getY().dump("defaultState:Y");
	ASSERT ((defaultState.getY()-defaultStateOfCopy.getY()).norm() < 1e-7);
	defaultState.getZ().dump("defaultState:Z");
	ASSERT ((defaultState.getZ()-defaultStateOfCopy.getZ()).norm() < 1e-7);

	//  Now delete original model and make sure copy can stand
	Model *newModel = modelCopy->clone();
	// Compare state again
	delete modelCopy;
	SimTK::State& defaultStateOfCopy2 = newModel->initSystem();
	// Compare state
	ASSERT ((defaultState.getY()-defaultStateOfCopy2.getY()).norm() < 1e-7);
	ASSERT ((defaultState.getZ()-defaultStateOfCopy2.getZ()).norm() < 1e-7);
	delete newModel;

	size_t mem2 = getCurrentRSS( );
	int64_t delta = mem2-mem1;

	cout << "Memory change AFTER copy and init:  " << delta/1024 << "KB." << endl;
}
