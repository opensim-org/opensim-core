// testVersioning.cpp
// Author:  Ayman Habib
/*
* Copyright (c) 2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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