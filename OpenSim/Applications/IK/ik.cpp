// ik.cpp
// Author: Peter Loan
/* Copyright (c)  2005, Stanford University and Peter Loan.
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace std;
using namespace OpenSim;

static void PrintUsage(const char *aProgName, ostream &aOStream);
//______________________________________________________________________________
/**
 * Test program to read SIMM model elements from an XML file.
 *
 * @param argc Number of command line arguments (should be 1).
 * @param argv Command line arguments:  simmReadXML inFile
 */
int main(int argc,char **argv)
{
	//----------------------
	// Surrounding try block
	//----------------------
	try {
	//----------------------

	// REGISTER TYPES
	InverseKinematicsTool::registerTypes();

	// PARSE COMMAND LINE
	string option = "";
	string setupFileName;
	if (argc < 2)
	{
		PrintUsage(argv[0], cout);
		exit(-1);
	}
	else {		// Don't know maybe user needs help or have special needs
		int i;
		for(i=1;i<=(argc-1);i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(argv[0], cout);
					return(0);

			// IDENTIFY SETUP FILE
			} else if((option=="-Setup")||(option=="-S")) {
				setupFileName = argv[i+1];
				break;

			} else if((option=="-PrintSetup")||(option=="-PS")) {
				InverseKinematicsTool *tool = new InverseKinematicsTool();
				tool->setName("default");
				Object::setSerializeAllDefaults(true);
				tool->print("default_Setup_IK.xml");
				Object::setSerializeAllDefaults(false);
				cout << "Created file default_Setup_IK.xml with default setup" << endl;
				return 0;

			// PRINT PROPERTY INFO
			} else if((option=="-PropertyInfo")||(option=="-PI")) {
				if((i+1)>=argc) {
					Object::PrintPropertyInfo(cout,"");

				} else {
					char *compoundName = argv[i+1];
					if(compoundName[0]=='-') {
						Object::PrintPropertyInfo(cout,"");
					} else {
						Object::PrintPropertyInfo(cout,compoundName);
					}
				}
				return(0);

			// UNRECOGNIZED
			} else {
				cout << "Unrecognized option" << option << "on command line... Ignored" << endl;
				PrintUsage(argv[0], cout);
				return(0);
			}
		}
	}

	// ERROR CHECK
	if(setupFileName=="") {
		cout<<"\n\nik.exe: ERROR- A setup file must be specified.\n";
		PrintUsage(argv[0], cout);
		return(-1);
	}

	// CONSTRUCT
	cout<<"Constructing tool from setup file "<<setupFileName<<".\n\n";
	InverseKinematicsTool ik(setupFileName);
	//ik.print("ik_setup_check.xml");

	// PRINT MODEL INFORMATION
	//Model& model = ik.getModel();
	//model.printBasicInfo(cout);

	// start timing
	std::clock_t startTime = std::clock();

	// RUN
	ik.run();

	std::cout << "IK compute time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";


	//----------------------------
	// Catch any thrown exceptions
	//----------------------------
	} catch(Exception x) {
		x.print(cout);
		return(-1);
	}
	//----------------------------

	return(0);
}

//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(const char *aProgName, ostream &aOStream)
{
	string progName=IO::GetFileNameFromURI(aProgName);
	aOStream<<"\n\n"<<progName<<":\n"<<GetVersionAndDate()<<"\n\n";
	aOStream<<"Option             Argument         Action / Notes\n";
	aOStream<<"------             --------         --------------\n";
	aOStream<<"-Help, -H                           Print the command-line options for "<<progName<<".\n";
	aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize the scaling\n";
	aOStream<<"-Setup, -S         SetupFileName    Specify an xml setup file for solving an inverse kinematics problem.\n";
	aOStream<<"-PropertyInfo, -PI                  Print help information for properties in setup files.\n";
}
	
