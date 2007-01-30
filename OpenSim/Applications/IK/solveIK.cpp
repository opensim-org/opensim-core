// ik.cpp
// Author: Peter Loan
/* Copyright (c) 2005, Stanford University and Peter Loan.
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
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Simulation/SIMM/MarkerSet.h>
#include <OpenSim/Subject/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarker.h>
#include <OpenSim/Simulation/SIMM/SimmCoordinate.h>
#include "IKTool.h"

using namespace std;
using namespace OpenSim;

static void PrintUsage(ostream &aOStream);
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

	// PARSE COMMAND LINE
	string option = "";
	string setupFileName;
	if (argc < 2)
	{
		PrintUsage(cout);
		exit(-1);
	}
	else {		// Don't know maybe user needs help or have special needs
		int i;
		for(i=1;i<=(argc-1);i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(cout);
					return(0);
					// IDENTIFY SETUP FILE
				} else if((option=="-Setup")||(option=="-S")) {
					setupFileName = argv[i+1];
					break;
				}
				else if((option=="-PrintSetup")||(option=="-PS")) {
					IKTool *tool = new IKTool();
					tool->setName("default");
					Object::setSerializeAllDefaults(true);

					tool->getMarkerSet().append(new SimmMarker());
					SimmCoordinate* aCoordinate = new SimmCoordinate();
					tool->getCoordinateSet().append(aCoordinate);

					tool->print("setup_ik_default.xml");
					Object::setSerializeAllDefaults(false);
					cout << "Created file setup_ik_default.xml with default setup" << endl;
					return 0;
				}
				else {
					cout << "Unrecognized option" << option << "on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}

	// ERROR CHECK
	if(setupFileName=="") {
		cout<<"\n\nik.exe: ERROR- A setup file must be specified.\n";
		PrintUsage(cout);
		return(-1);
	}

	IKTool::registerTypes();

	// CONSTRUCT
	cout<<"Constructing tool from setup file "<<setupFileName<<".\n\n";
	IKTool ik(setupFileName);
	ik.print("ik_setup_check.xml");

	// PRINT MODEL INFORMATION
	AbstractModel *model = ik.getModel();
	if(model==NULL) {
		cout<<"\nik:  ERROR- failed to load model.\n";
		exit(-1);
	}
	model->printBasicInfo(cout);

	// RUN
	ik.run();

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
	aOStream<<"\n\nik.exe:\n\n";
	aOStream<<"Option             Argument         Action / Notes\n";
	aOStream<<"------             --------         --------------\n";
	aOStream<<"-Help, -H                           Print the command-line options for scale.exe.\n";
	aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize the scaling\n";
	aOStream<<"-Setup, -S         SetupFile        Specify an xml setupfile that specifies an OpenSim model,\n";
	aOStream<<"                                    a marker file, and scaling parameters.\n";
}
	
