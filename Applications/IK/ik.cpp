/* -------------------------------------------------------------------------- *
 *                              OpenSim:  ik.cpp                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/ScaleTool.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

#include <ctime>  // clock(), clock_t, CLOCKS_PER_SEC

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

    // DEPRECATION NOTICE
    const std::string deprecationNotice = R"(
    THIS EXECUTABLE IS DEPRECATED AND WILL BE REMOVED IN A FUTURE RELEASE.

    Use opensim-cmd instead, which can do everything that this executable can.

      ik -S SetupFileName -> opensim-cmd run-tool SetupFileName
      ik -PS              -> opensim-cmd print-xml ik
    )";
    std::cout << deprecationNotice << std::endl;

    // REGISTER TYPES
    InverseKinematicsTool::registerTypes();

    // PARSE COMMAND LINE
    string option = "";
    string setupFileName;
    if (argc < 2)
    {
        PrintUsage(argv[0], cout);
        exit(-1);
    }
    else {      // Don't know maybe user needs help or have special needs
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

            // IDENTIFY SETUP FILE
            } else if((option=="-Setup")||(option=="-S")) {
                setupFileName = argv[i+1];
                break;

            } else if((option=="-PrintSetup")||(option=="-PS")) {
                InverseKinematicsTool *tool = new InverseKinematicsTool();
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
    InverseKinematicsTool ik(setupFileName);
    //ik.print("ik_setup_check.xml");

    // PRINT MODEL INFORMATION
    //Model& model = ik.getModel();
    //model.finalizeFromProperties();
    //model.printBasicInfo();

    // start timing
    std::clock_t startTime = std::clock();

    // RUN
    ik.run();

    std::cout << "IK compute time = " << 1.e3*(std::clock()-startTime)/CLOCKS_PER_SEC << "ms\n";


    //----------------------------
    // Catch any thrown exceptions
    //----------------------------
    } catch(const std::exception& x) {
        cout << "Exception in IK: " << x.what() << endl;
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
    aOStream<<"Option             Argument         Action / Notes\n";
    aOStream<<"------             --------         --------------\n";
    aOStream<<"-Help, -H                           Print the command-line options for "<<progName<<".\n";
    aOStream<<"-PrintSetup, -PS                    Generates a template Setup file to customize the scaling\n";
    aOStream<<"-Setup, -S         SetupFileName    Specify an xml setup file for solving an inverse kinematics problem.\n";
    aOStream<<"-PropertyInfo, -PI                  Print help information for properties in setup files.\n";
}
    
