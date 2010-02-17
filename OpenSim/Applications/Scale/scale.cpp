// scale.cpp
// Author: Ayman Habib
/* Copyright (c)  2005, Stanford University and Ayman Habib.
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/VisibleProperties.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
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
	//TODO: put these options on the command line
	//LoadOpenSimLibrary("osimSimbodyEngine");

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

        // Realize the topology and initialize state
       
        SimTK::State& s = model->initSystem();

		if (!subject->isDefaultModelScaler() && subject->getModelScaler().getApply())
		{
			ModelScaler& scaler = subject->getModelScaler();
			if(!scaler.processModel(s, model, subject->getPathToSubject(), subject->getSubjectMass())) return 1;
		}
		else
		{
			cout << "Scaling parameters disabled (apply is false) or not set. Model is not scaled." << endl;
		}
		
		SimTK::State& news = model->initSystem();	// old state is messed up by scaling. can't use it
		if (!subject->isDefaultMarkerPlacer() && subject->getMarkerPlacer().getApply())
		{
			MarkerPlacer& placer = subject->getMarkerPlacer();
			if(!placer.processModel(news, model, subject->getPathToSubject())) return 1;
		}
		else
		{
			cout << "Marker placement parameters disabled (apply is false) or not set. No markers have been moved." << endl;
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
	aOStream<<"\n\n"<<progName<<":\n"<<GetVersionAndDate()<<"\n\n";
	aOStream<<"Option            Argument          Action / Notes\n";
	aOStream<<"------            --------          --------------\n";
	aOStream<<"-Help, -H                           Print the command-line options for "<<progName<<".\n";
	aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize scaling\n";
	aOStream<<"-Setup, -S        SetupFileName     Specify an xml setup file for scaling a generic model.\n";
	aOStream<<"-PropertyInfo, -PI                  Print help information for properties in setup files.\n";

}
