/* -------------------------------------------------------------------------- *
 *                            OpenSim:  scale.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */


// INCLUDES
#include <string>
#include <OpenSim/version.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/ForceSet.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Analysis.h>

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

    // DEPRECATION NOTICE
    const std::string deprecationNotice = R"(
    THIS EXECUTABLE IS DEPRECATED AND WILL BE REMOVED IN A FUTURE RELEASE.

    Use opensim-cmd instead, which can do everything that this executable can.

      scale -S SetupFileName -> opensim-cmd run-tool SetupFileName
      scale -PS              -> opensim-cmd print-xml scale
    )";
    log_warn(deprecationNotice);

    // SET OUTPUT FORMATTING
    IO::SetDigitsPad(4);

    // REGISTER TYPES
    Object::registerType(ScaleTool());
    ScaleTool::registerTypes();

    // PARSE COMMAND LINE
    string inName;
    string option = "";
    if (argc < 2) {
        PrintUsage(argv[0], cout);
        exit(-1);
    } else {
        // Load libraries first
        LoadOpenSimLibraries(argc,argv);
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
                    log_info("Created file default_Setup_Scale.xml with "
                             "default setup");
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
                    log_error("Unrecognized option {} on command line... "
                              "Ignored", option);
                    PrintUsage(argv[0], cout);
                    return(0);
                }
        }
    }


    try {
        // Run the tool.
        std::unique_ptr<ScaleTool> subject(new ScaleTool(inName));
        const bool success = subject->run();
        if (success) return EXIT_SUCCESS;
        else return EXIT_FAILURE;
    }
    catch(const Exception& x) {
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
