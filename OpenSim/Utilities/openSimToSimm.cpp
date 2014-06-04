/* Copyright (c)  2007, Stanford University and Eran Guendelman.
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Utilities/SimmFileWriterDLL/SimmFileWriter.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/Delp1990Muscle.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace std;
using namespace OpenSim;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments (should be 4 or more).
 * @param argv Command line arguments:  openSimToSimm -x xml_in -j joints_out [ -m muscles_out ]
 */
int main(int argc,char **argv)
{
	std::cout << "openSimToSimm, " << OpenSim::GetVersionAndDate() << std::endl;

   Object::RegisterType(SimbodyEngine());
	SimbodyEngine::registerTypes();
   Object::RegisterType(SimmKinematicsEngine());
	SimmKinematicsEngine::registerTypes();
	Object::RegisterType(Schutte1993Muscle());
	Object::RegisterType(Thelen2003Muscle());
	Object::RegisterType(Delp1990Muscle());

	// PARSE COMMAND LINE
	string inName = "";
	string jntName = "";
	string mslName = "";

	for(int i=1; i<argc; i++) {
		string option = argv[i];

		// PRINT THE USAGE OPTIONS
		if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
			(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
			PrintUsage(argv[0], cout);
			return(0);
		}
		else if(option=="-x" || option=="-X") inName = argv[++i];
		else if(option=="-j" || option=="-J") jntName = argv[++i];
		else if(option=="-m" || option=="-M") mslName = argv[++i];
	}

	if(inName=="" || jntName == "") {
		PrintUsage(argv[0], cout);
		return(-1);
	}

	try {
		Model model(inName);
		model.setup();

		SimmFileWriter sfw(&model);
		if(jntName!="") sfw.writeJointFile(jntName);
		if(mslName!="") sfw.writeMuscleFile(mslName);
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
	aOStream << "Usage: " << progName << " -x xml_in -j joints_out [ -m muscles_out ]" << std::endl;
}
