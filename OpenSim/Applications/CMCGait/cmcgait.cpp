// cmcgait.cpp
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Copyright (c) 2006 Stanford University and Realistic Dynamics, Inc.
// Contributors: Frank C. Anderson, Ph.D.
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject
// to the following conditions:
// 
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE AUTHORS,
// CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH
// THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/Simulation/Model/LoadModel.h>
#include <OpenSim/CMC/InvestigationCMCGait.h>

using namespace std;
using namespace OpenSim;

static void PrintUsage(ostream &aOStream);


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
		PrintUsage(cout);
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
			PrintUsage(cout);
			return(0);
 
		// PRINT A DEFAULT SETUP FILE FOR THIS INVESTIGATION
		} else if((option=="-PrintSetup")||(option=="-PS")) {
			InvestigationCMCGait *investigation = new InvestigationCMCGait();
			investigation->setName("default");
			Object::setSerializeAllDefaults(true);
			investigation->print("setup_cmc_default.xml");
			Object::setSerializeAllDefaults(false);
			return(0);

		// IDENTIFY SETUP FILE
		} else if((option=="-Setup")||(option=="-S")) {
			if((i+1)<argc) setupFileName = argv[i+1];
			break;
		}
	}

	// ERROR CHECK
	if(setupFileName=="") {
		cout<<"\n\ncmc.exe: ERROR- A setup file must be specified.\n";
		PrintUsage(cout);
		return(-1);
	}

	// CONSTRUCT
	cout<<"Constructing investigation from setup file "<<setupFileName<<".\n\n";
	InvestigationCMCGait cmcgait(setupFileName);

	// PRINT MODEL INFORMATION
	AbstractModel *model = cmcgait.getModel();
	if(model==NULL) {
		cout<<"\nperturb:  ERROR- failed to load model.\n";
		exit(-1);
	}
	cout<<"-----------------------------------------------------------------------\n";
	cout<<"Loaded library\n";
	cout<<"-----------------------------------------------------------------------\n";
	model->printBasicInfo(cout);
	cout<<"-----------------------------------------------------------------------\n\n";


	// RUN
	cmcgait.run();


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
void PrintUsage(ostream &aOStream)
{
	aOStream<<"\n\ncmcgait.exe:\n\n";
	aOStream<<"At least one of the following command-line options must be specified.\n\n";
	aOStream<<"Option              Argument         Action / Notes\n";
	aOStream<<"------              --------         --------------\n";
	aOStream<<"-Help, -H                            Print the command-line options for cmc.exe.\n";
	aOStream<<"-PrintSetup, -PS                     Print a default setup file for cmc.exe (setup_cmc_default.xml).\n";
	aOStream<<"-Setup, -S          SetupFileName    Specifies the name of the XML setup file for the CMC investigation.\n";
	aOStream<<"-Library, -L        LibraryName      Specifiy a library to load. Do not include the extension (e.g., .lib or .dll).\n";
	aOStream<<"                                     To load more than one library, repeat the -Library command-line option.\n\n";

}

