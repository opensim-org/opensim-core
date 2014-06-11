/* Copyright (c)  2008, Stanford University and Peter Loan.
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
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <SimTKcommon/internal/Exception.h>
#include <OpenSim/Common/rdMath.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/Utilities/migrateSimmKEModelDLL/migrateSimmKEModelDLL.h>

#ifdef STATIC_OSIM_LIBS
#include <OpenSim/Common/RegisterTypes_osimCommon.h>
#include <OpenSim/Simulation/RegisterTypes_osimSimulation.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/RegisterTypes_osimSimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/RegisterTypes_osimSimmKinematicsEngine.h>
#include <OpenSim/Actuators/RegisterTypes_osimActuators.h>
#else
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#endif

using namespace std;
using namespace OpenSim;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments (should be 2).
 * @param argv Command line arguments:  migrateSimmKEModel model_in model_out
 */
int main(int argc,char **argv)
{
	std::cout << "migrateSimmKEModel, " << OpenSim::GetVersionAndDate() << std::endl;

#ifdef STATIC_OSIM_LIBS
	RegisterTypes_osimCommon();
	RegisterTypes_osimSimulation();
	RegisterTypes_SimbodyEngine();
	RegisterTypes_osimSimmKinematicsEngine();
	RegisterTypes_osimActuators();
#else
	LoadOpenSimLibrary("osimActuators");
	LoadOpenSimLibrary("osimSimbodyEngine");
	LoadOpenSimLibrary("osimSimmKinematicsEngine");
#endif

	// PARSE COMMAND LINE
	string inName = "";
	string outName = "";

	for(int i=1; i<argc; i++) {
		string option = argv[i];

		// PRINT THE USAGE OPTIONS
		if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
			(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
			PrintUsage(argv[0], cout);
			return(0);
		}
	}

	if (argc != 3) {
		PrintUsage(argv[0], cout);
		return(-1);
	}

	inName = argv[1];
	outName = argv[2];

	try {
		Model model(inName);
		model.setup();

		SimmKinematicsEngine* oldEngine = dynamic_cast<SimmKinematicsEngine*>(&model.getDynamicsEngine());
		if (oldEngine != NULL) {
			AbstractDynamicsEngine* newEngine = makeSimbodyEngine(model, *oldEngine);
			model.replaceEngine(newEngine);
		} else {
			cout << inName << " does not use a SimmKinematicsEngine. Migration cancelled." << endl;
			return -1;
		}
		Model* hack = (Model*)model.copy();
		hack->print(outName);
		cout << "Model " << hack->getName() << " converted to a Simbody engine and written to " << outName << endl;
	}
	catch(Exception &x) {
		x.print(cout);
	}
}
//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(const char *aProgName, ostream &aOStream)
{
	string progName=IO::GetFileNameFromURI(aProgName);
	aOStream << "Usage: " << progName << " model_in model_out" << std::endl;
}

