/* -------------------------------------------------------------------------- *
 *                             OpenSim:  cmc.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
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
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/CMCTool.h>
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
    
    //LoadOpenSimLibrary("osimSdfastEngine");
    //LoadOpenSimLibrary("osimSimbodyEngine");

    // DEPRECATION NOTICE
    const std::string deprecationNotice = R"(
    THIS EXECUTABLE IS DEPRECATED AND WILL BE REMOVED IN A FUTURE RELEASE.

    Use opensim-cmd instead, which can do everything that this executable can.

      cmc -S SetupFileName -> opensim-cmd run-tool SetupFileName
      cmc -PS              -> opensim-cmd print-xml cmc
    )";
    std::cout << deprecationNotice << std::endl;

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
            CMCTool *investigation = new CMCTool();
            investigation->setName("default");
            Object::setSerializeAllDefaults(true);
            investigation->print("default_Setup_CMC.xml");
            Object::setSerializeAllDefaults(false);
            cout << "Created file default_Setup_CMC.xml with default setup" << endl;
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
    cout<<"Constructing investigation from setup file "<<setupFileName<<".\n"<<endl;
    CMCTool cmcgait(setupFileName);

    // PRINT MODEL INFORMATION
    Model& model = cmcgait.getModel();
    cout<<"-----------------------------------------------------------------------\n";
    cout<<"Loaded library\n";
    cout<<"-----------------------------------------------------------------------\n";
    model.finalizeFromProperties();
    model.printBasicInfo();
    cout<<"-----------------------------------------------------------------------\n\n";


    // RUN
    cmcgait.run();


    //----------------------------
    // Catch any thrown exceptions
    //----------------------------
    } catch(const std::exception& x) {
        cout << "Exception in CMC: " << x.what() << endl;
        return -1;
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
    aOStream<<"-Library, -L        LibraryName      Specify a library to load. Do not include the extension (e.g., .lib or .dll).\n";
    aOStream<<"                                     To load more than one library, repeat the -Library command-line option.\n\n";

}

