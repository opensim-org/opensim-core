// scale.cpp
// Author: Ayman Habib
/* Copyright (c) 2005, Stanford University and Ayman Habib.
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/VisibleProperties.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>

#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Tools/IKSolverImpl.h>
#include <OpenSim/Tools/IKTarget.h>

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
	// SET OUTPUT FORMATTING
	IO::SetDigitsPad(4);

	// REGISTER TYPES
	Object::RegisterType(VisibleObject());
	Object::RegisterType(ScaleTool());
	ScaleTool::registerTypes();

	// PARSE COMMAND LINE
	string inName;
	string option = "";
	if (argc < 2) {
		PrintUsage(argv[0], cout);
		exit(-1);
	} else {
		int i;
		for(i=1;i<=(argc-1);i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(argv[0], cout);
					return(0);

				// Identify the setup file
				} else if((option=="-S")||(option=="-Setup")) {
					if (argv[i+1]==0){
						PrintUsage(argv[0], cout);
						return(0);
					}
					inName = argv[i+1];
					break;

				// Print a default setup file
				} else if((option=="-PrintSetup")||(option=="-PS")) {
					ScaleTool *subject = new ScaleTool();
					subject->setName("default");
					// Add in useful objects that may need to be instantiated
					Object::setSerializeAllDefaults(true);
					subject->print("default_Setup_Scale.xml");
					Object::setSerializeAllDefaults(false);
					cout << "Created file default_Setup_Scale.xml with default setup" << endl;
					return(0);

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

				// Unrecognized
				} else {
					cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
					PrintUsage(argv[0], cout);
					return(0);
				}
		}
	}


	try {
		// Construct model and read parameters file
		ScaleTool* subject = new ScaleTool(inName);
		Model* model = subject->createModel();

		if(!model) throw Exception("scale: ERROR- No model specified.",__FILE__,__LINE__);

		if (!subject->isDefaultModelScaler())
		{
			ModelScaler& scaler = subject->getModelScaler();
			if(!scaler.processModel(model, subject->getPathToSubject(), subject->getMass())) return 1;
		}
		else
		{
			cout << "Scaling parameters not set. Model is not scaled." << endl;
		}

		if (!subject->isDefaultMarkerPlacer())
		{
			MarkerPlacer& placer = subject->getMarkerPlacer();
			if(!placer.processModel(model, subject->getPathToSubject())) return 1;
		}
		else
		{
			cout << "Marker placement parameters not set. No markers have been moved." << endl;
		}

		delete model;
		delete subject;
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
	aOStream<<"\n\n"<<progName<<":\n"<<Version_And_Date()<<"\n\n";
	aOStream<<"Option            Argument          Action / Notes\n";
	aOStream<<"------            --------          --------------\n";
	aOStream<<"-Help, -H                           Print the command-line options for "<<progName<<".\n";
	aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize scaling\n";
	aOStream<<"-Setup, -S        SetupFileName     Specify an xml setup file for scaling a generic model.\n";
	aOStream<<"-PropertyInfo, -PI                  Print help information for properties in setup files.\n";

}
