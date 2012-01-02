// testID.cpp
// Author: Ayman Habib, Ajay Seth 
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
* 
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
// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Tools/InverseDynamicsTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
	try {
		InverseDynamicsTool id1("arm26_Setup_InverseDynamics.xml");
		id1.run();
		Storage result1("Results/arm26_InverseDynamics.sto"), standard1("std_arm26_InverseDynamics.sto");
		CHECK_STORAGE_AGAINST_STANDARD( result1, standard1, Array<double>(1e-2, 23), __FILE__, __LINE__, "testArm failed");
		cout << "testArm passed" << endl;

		InverseDynamicsTool id2("subject01_Setup_InverseDynamics.xml");
		id2.run();
		Storage result2("Results/subject01_InverseDynamics.sto"), standard2("std_subject01_InverseDynamics.sto");
		CHECK_STORAGE_AGAINST_STANDARD(result2, standard2, Array<double>(2.0, 23), __FILE__, __LINE__, "testGait failed");
		cout << "testGait passed" << endl;
		/*
		InverseDynamicsTool id3("subject221_Setup_InverseDynamics.xml");
		id3.run();
		Storage result3("Results/subject221_InverseDynamics.sto"), standard3("std_subject221_InverseDynamics.sto");
		CHECK_STORAGE_AGAINST_STANDARD(result3, standard3, Array<double>(2.0, 23), __FILE__, __LINE__, "subject 221 old setup failed");
		cout << "subject 221 old setup passed" << endl;
		*/
	}
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
