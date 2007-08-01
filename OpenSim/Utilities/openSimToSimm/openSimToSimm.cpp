/* Copyright (c) 2007, Stanford University and Eran Guendelman.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

// INCLUDES
#include <string>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmFileWriter.h>

using namespace std;
using namespace OpenSim;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments (should be 2 or more).
 * @param argv Command line arguments:  mkModel -IM inFile and at least one of [-SD SdfastFile] [-OM SimulationModelFile] [-D OutputDirectory]
 */
int main(int argc,char **argv)
{
	std::cout << "openSimToSimm, " << OpenSim::GetVersionAndDate() << std::endl;

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
