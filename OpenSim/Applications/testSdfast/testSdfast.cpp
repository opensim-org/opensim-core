// testSdfast.cpp
// Author: Peter Loan
/* Copyright (c)  2006, Stanford University and Peter Loan.
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

// INCLUDES
#include <string>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/VisibleProperties.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/DynamicsEngines/SdfastEngine/SdfastEngine.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>

using namespace std;
using namespace OpenSim;


//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments
 * @param argv Command line arguments: testSdfast 
 */
int main(int argc, char **argv)
{
	Object::RegisterType(SdfastEngine());
	SdfastEngine::registerTypes();

	Object::RegisterType(SimmKinematicsEngine());
	SimmKinematicsEngine::registerTypes();

	Object::RegisterType(Schutte1993Muscle());
	//Schutte1993Muscle::registerTypes();

	try {
			// Construct the model with the SimmKinematicsEngine
			Model *model1 = new Model("C:/SimTK/OpenSim/Utilities/simmToOpenSim/test/meTest2.osim");
			model1->setup();
			model1->kinTest();

			// Construct the model with the SdfastEngine
			Model *model2 = new Model("ellWrapSdfast.xml");
			model2->setup();
			model2->kinTest();

			// Cleanup
			delete model1;
			delete model2;
	}
	catch(Exception &x) {
		x.print(cout);
	}

}
