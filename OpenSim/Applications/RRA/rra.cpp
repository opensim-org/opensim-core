// rra.cpp
// Author: Ayman Habib based on Frank C. Anderson's CMC
/*
* Copyright (c)  2005, Stanford University, All rights reserved. 
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
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/RRATool.h>
#include <OpenSim/Common/IO.h>

using namespace std;
using namespace OpenSim;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//_____________________________________________________________________________
/**
 * Main routine for running the Computed Muscle Control (CMC) algorithm.
 */
int main(int argc,char **argv)
{
	//----------------------
	// Surrounding try block
	//----------------------
	try {
	//----------------------

	// PARSE COMMAND LINE
	int i;
	string option = "";
	string setupFileName = "";
	if(argc<2) {
		PrintUsage(argv[0], cout);
		return(-1);
	}
	// Load libraries first
	LoadOpenSimLibraries(argc,argv);
	// Parse other command-line options
	for(i=1;i<argc;i++) {
		option = argv[i];

		// PRINT THE USAGE OPTIONS
		if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
		(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
			PrintUsage(argv[0], cout);
			return(0);
 
		// PRINT A DEFAULT SETUP FILE FOR THIS INVESTIGATION
		} else if((option=="-PrintSetup")||(option=="-PS")) {
			RRATool *investigation = new RRATool();
			investigation->setName("default");
			Object::setSerializeAllDefaults(true);
			investigation->print("default_Setup_RRA.xml");
			Object::setSerializeAllDefaults(false);
			cout << "Created file default_Setup_RRA.xml with default setup" << endl;
			return(0);

		// IDENTIFY SETUP FILE
		} else if((option=="-Setup")||(option=="-S")) {
			if((i+1)<argc) setupFileName = argv[i+1];
			break;

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
		}
	}

	// ERROR CHECK
	if(setupFileName=="") {
		cout<<"\n\n"<<argv[0]<<": ERROR- A setup file must be specified.\n";
		PrintUsage(argv[0], cout);
		return(-1);
	}

	// CONSTRUCT
	cout<<"Constructing investigation from setup file "<<setupFileName<<".\n\n";
	RRATool rra(setupFileName);

	// PRINT MODEL INFORMATION
	Model& model = rra.getModel();
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"Loaded library\n";
	cout<<"-----------------------------------------------------------------------\n";
	model.printBasicInfo(cout);
	cout<<"-----------------------------------------------------------------------\n\n";


	// RUN
	rra.run();


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
	aOStream<<"Option              Argument         Action / Notes\n";
	aOStream<<"------              --------         --------------\n";
	aOStream<<"-Help, -H                            Print the command-line options for "<<progName<<".\n";
	aOStream<<"-PrintSetup, -PS                     Print a default setup file for cmc.exe (setup_cmc_default.xml).\n";
	aOStream<<"-Setup, -S          SetupFileName    Specify the name of the XML setup file for the CMC investigation.\n";
	aOStream<<"-PropertyInfo, -PI                   Print help information for properties in setup files.\n";
	aOStream<<"-Library, -L        LibraryName      Specifiy a library to load. Do not include the extension (e.g., .lib or .dll).\n";
	aOStream<<"                                     To load more than one library, repeat the -Library command-line option.\n\n";

}

