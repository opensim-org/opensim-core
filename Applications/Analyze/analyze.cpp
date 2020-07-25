/* -------------------------------------------------------------------------- *
 *                           OpenSim:  analyze.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ayman Habib                                  *
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

// INCLUDE
#include <string>
#include <iostream>
#include <OpenSim/version.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

using namespace OpenSim;
using namespace std;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//_____________________________________________________________________________
/**
 * Main routine for stepping through a trajectory and running a set of
 * analyses.
 */
int main(int argc,char **argv)
{
    //----------------------
    // Surrounding try block
    //----------------------
    try {

    // DEPRECATION NOTICE
    const std::string deprecationNotice = R"(
    THIS EXECUTABLE IS DEPRECATED AND WILL BE REMOVED IN A FUTURE RELEASE.

    Use opensim-cmd instead, which can do everything that this executable can.

      analyze -S SetupFileName -> opensim-cmd run-tool SetupFileName
      analyze -PS              -> opensim-cmd print-xml analyze
    )";
    log_warn(deprecationNotice);

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

    for(i=1;i<argc;i++) {
        option = argv[i];

        // PRINT THE USAGE OPTIONS
        if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
        (option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {

            PrintUsage(argv[0], cout);
            return(0);
 
        // PRINT A DEFAULT SETUP FILE FOR THIS INVESTIGATION
        } else if((option=="-PrintSetup")||(option=="-PS")) {
            AnalyzeTool *tool = new AnalyzeTool();
            tool->setName("default");
            Object::setSerializeAllDefaults(true);
            tool->print("default_Setup_Analyze.xml");
            Object::setSerializeAllDefaults(false);
            log_info("Created file default_Setup_Analyze.xml with default "
                     "setup");
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
        log_error("analyze.exe: A setup file must be specified.", argv[0]);
        PrintUsage(argv[0], cout);
        return(-1);
    }

    //ISSUES:
    //Load dlls that register Built in Actuator classes.
    LoadOpenSimLibrary("osimActuators");

    // CONSTRUCT
    log_info("Constructing tool from setup file {}", setupFileName);
    AnalyzeTool analyze(setupFileName);

    // PRINT MODEL INFORMATION
    // Model& model = analyze.getModel();
    log_info("--------------------------------------------------------------");
    log_info("Loaded library");
    log_info("--------------------------------------------------------------");

    // start timing
    std::clock_t startTime = std::clock();

    // RUN
    analyze.run();

    auto timeInMilliseconds = 1.e3*(std::clock() - startTime) / CLOCKS_PER_SEC;
    log_info("Analyze compute time = {} ms", timeInMilliseconds);

    //----------------------------
    // Catch any thrown exceptions
    //----------------------------
    } catch(const std::exception& x) {
        log_error("Exception in analyze: {}", x.what());
        return -1;
    }
    //----------------------------
    return 0;
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
    aOStream<<"-Help, -H                            Print the command-line options for forward.exe.\n";
    aOStream<<"-PrintSetup, -PS                     Print a default setup file for forward.exe (default_forward.xml).\n";
    aOStream<<"-Setup, -S          SetupFileName    Specify the name of the XML setup file to use for this forward tool.\n";
    aOStream<<"-PropertyInfo, -PI                   Print help information for properties in setup files.\n";
}

