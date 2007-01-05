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
#include <OpenSim/Simulation/SIMM/AbstractModel.h>
#include <OpenSim/Actuators/LinearSetPoint.h>
#include <OpenSim/Subject/SdfastFileWriter.h>

using namespace std;
using namespace OpenSim;


static void PrintUsage(ostream &aOStream);

//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments (should be 2 or more).
 * @param argv Command line arguments:  mkModel -IM inFile and at least one of [-SD SdfastFile] [-FF SourceFile] [-MH HeaderFile] [-PF ParametersFile] [-OM SimulationModelFile] [-D OutputDirectory]
 */
int main(int argc,char **argv)
{
	// PARSE COMMAND LINE
	static string unassigned = "Unassigned";
	string inName = unassigned;
	string sdfastName = unassigned;
	string headerName = unassigned;
	string sourceName = unassigned;
	string parametersName = unassigned;
	string simModelName = unassigned;
	string folderName = ".";
	string option = "";

	if (argc < 3)
	{
		PrintUsage(cout);
		exit(-1);
	}
	else {		// Don't know maybe user needs help or have special needs
		int i;
		for(i=1; i<=(argc-1); i++) {
			option = argv[i];

			// PRINT THE USAGE OPTIONS
			if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
				(option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
					PrintUsage(cout);
					return(0);
					// IDENTIFY SETUP FILE
				}
				else if((option=="-IM")||(option=="-InputModel")) {
					inName = argv[++i];
				}
				else if((option=="-SD")||(option=="-SystemDescription")) {
					sdfastName = argv[++i];
				}
				else if((option=="-FF")||(option=="-ForwardFile")) {
					sourceName = argv[++i];
				}
				else if((option=="-MH")||(option=="-ModelHeader")) {
					headerName = argv[++i];
				}
				else if((option=="-PF")||(option=="-ParametersFile")) {
					parametersName = argv[++i];
				}
				else if((option=="-OM")||(option=="-OutputModel")) {
					simModelName = argv[++i];
				}
				else if((option=="-D")||(option=="-Directory")) {
					folderName = argv[++i];
				}
				else {
					cout << "Unrecognized option " << option << " on command line... Ignored" << endl;
					PrintUsage(cout);
					return(0);
				}
		}
	}

	try {
		if (inName != unassigned)
		{
			// Construct the model
			AbstractModel *model = new AbstractModel(inName);
			model->setup();
			//model->peteTest();

			// Create SdfastFileWriter object and write out selected files
			SdfastFileWriter sfw(model, folderName);

			if (sdfastName != unassigned)
				sfw.writeSdfastFile(sdfastName);

			if (sourceName != unassigned) {
				if (headerName != unassigned) sfw.writeModelSourceFile(sourceName, headerName);
				else sfw.writeModelSourceFile(sourceName, "model.h");
			}

			if (headerName != unassigned)
				sfw.writeModelHeaderFile(headerName);

			if (parametersName != unassigned)
				sfw.writeSimulationParametersFile(parametersName);

			if (simModelName != unassigned)
				sfw.writeSimulationModelFile(simModelName);

			delete model;
		}
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
	aOStream<<"Option                    Argument         Action / Notes\n";
	aOStream<<"------                    --------         --------------\n";
	aOStream<<"-Help, -H                                  Print the command-line options for mkModel.exe.\n";
	aOStream<<"-InputModel, -IM          ModelFile        Input SimmKinematicsEngine model file (XML format).\n";
	aOStream<<"-SystemDescription, -SD   FileName         Output SDFast system description file (model.sd in SIMM)\n";
	aOStream<<"-ForwardFile, -FF         FileName         Output forward dynamics C file (sdfor.c in SIMM)\n";
	aOStream<<"-ModelHeader, -MH         FileName         Output model header file (model.h in SIMM)\n";
	aOStream<<"-ParametersFile, -PF      FileName         Output Parameters file (params.txt in SIMM)\n";
	aOStream<<"-OuputModel, -OM          ModelFile        Output SDfastEngine model file (XML format).\n";
	aOStream<<"-Directory, -D            DirectoryName    Directory into which to write output.\n";
}
