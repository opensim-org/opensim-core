// mkModel.cpp
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
#include <OpenSim/Tools/rdTools.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/IO.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/VisibleProperties.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/SIMM/SimmModel.h>
#include <OpenSim/Simulation/SIMM/SimmMuscle.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/Simulation/SIMM/SimmKinematicsEngine.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerSet.h>
#include <OpenSim/Simulation/SIMM/SimmSubject.h>
#include <OpenSim/Simulation/SIMM/SimmMarkerData.h>
#include <OpenSim/Simulation/SIMM/SimmMotionData.h>




using namespace OpenSim;
using namespace std;


static void PrintUsage(ostream &aOStream);

//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments (should be 1).
 * @param argv Command line arguments:  simmReadXML inFile
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	string inName;
	string option = "";
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
				}
				else if((option=="-MF")||(option=="-ModelFile")) {
					inName = argv[i+1];
					break; 
				}
				else {
					cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}
	try {
		// Construct model and read parameters file
		SimmModel *model = new SimmModel(inName);
		model->setup();
		SimmKinematicsEngine& engine = model->getSimmKinematicsEngine();
		engine.saveDynamics();
		delete model;
	}
	catch(Exception &x) {
		x.print(cout);
	}

}
	
//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(ostream &aOStream)
{
	aOStream<<"\n\nscale.exe:\n\n";
	aOStream<<"Option              Argument            Action / Notes\n";
	aOStream<<"------              --------            --------------\n";
	aOStream<<"-Help, -H                               Print the command-line options for scale.exe.\n";
	aOStream<<"-ModelFile, -MF     ModelFile           XML model file\n";
}
