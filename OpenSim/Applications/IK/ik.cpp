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
#include <OpenSim/version.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmCoordinate.h>
#include <OpenSim/Tools/IKTool.h>

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

	//TODO: put these options on the command line
	LoadOpenSimLibrary("osimSimbodyEngine");
	LoadOpenSimLibrary("osimSimmKinematicsEngine");

	// REGISTER TYPES
	IKTool::registerTypes();

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
				IKTool *tool = new IKTool();
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
	IKTool ik(setupFileName);
	//ik.print("ik_setup_check.xml");

	// PRINT MODEL INFORMATION
	Model *model = ik.getModel();
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
void PrintUsage(const char *aProgName, ostream &aOStream)
{
	string progName=IO::GetFileNameFromURI(aProgName);
	aOStream<<"\n\n"<<progName<<":\n"<<Version_And_Date()<<"\n\n";
	aOStream<<"Option             Argument         Action / Notes\n";
	aOStream<<"------             --------         --------------\n";
	aOStream<<"-Help, -H                           Print the command-line options for "<<progName<<".\n";
	aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize the scaling\n";
	aOStream<<"-Setup, -S         SetupFileName    Specify an xml setup file for solving an inverse kinematics problem.\n";
	aOStream<<"-PropertyInfo, -PI                  Print help information for properties in setup files.\n";
}
	
