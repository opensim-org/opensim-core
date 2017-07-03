/* -------------------------------------------------------------------------- *
 *                              openSimToSimm.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Utilities/SimmFileWriterDLL/SimmFileWriter.h>
#include <OpenSim/DynamicsEngines/SimbodyEngine/SimbodyEngine.h>
#include <OpenSim/DynamicsEngines/SimmKinematicsEngine/SimmKinematicsEngine.h>
#include <OpenSim/Actuators/Schutte1993Muscle.h>
#include <OpenSim/Actuators/Thelen2003Muscle.h>
#include <OpenSim/Actuators/Delp1990Muscle.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace std;
using namespace OpenSim;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//______________________________________________________________________________
/**
 * Program to read an xml file for an openSim Model and generate
 * SDFast corresponding code.
 *
 * @param argc Number of command line arguments (should be 4 or more).
 * @param argv Command line arguments:  openSimToSimm -x xml_in -j joints_out [ -m muscles_out ]
 */
int main(int argc,char **argv)
{
    std::cout << "openSimToSimm, " << OpenSim::GetVersionAndDate() << std::endl;

   Object::RegisterType(SimbodyEngine());
    SimbodyEngine::registerTypes();
   Object::RegisterType(SimmKinematicsEngine());
    SimmKinematicsEngine::registerTypes();
    Object::RegisterType(Schutte1993Muscle());
    Object::RegisterType(Thelen2003Muscle());
    Object::RegisterType(Delp1990Muscle());

    // PARSE COMMAND LINE
    string inName = "";
    string jntName = "";
    string mslName = "";

    for(int i=1; i<argc; i++) {
        string option = argv[i];

        // PRINT THE USAGE OPTIONS
        if((option=="-help")||(option=="-h")||(option=="-Help")||(option=="-H")||
            (option=="-usage")||(option=="-u")||(option=="-Usage")||(option=="-U")) {
            PrintUsage(argv[0], cout);
            return(0);
        }
        else if(option=="-x" || option=="-X") inName = argv[++i];
        else if(option=="-j" || option=="-J") jntName = argv[++i];
        else if(option=="-m" || option=="-M") mslName = argv[++i];
    }

    if(inName=="" || jntName == "") {
        PrintUsage(argv[0], cout);
        return(-1);
    }

    try {
        Model model(inName);
        model.setup();

        SimmFileWriter sfw(&model);
        if(jntName!="") sfw.writeJointFile(jntName);
        if(mslName!="") sfw.writeMuscleFile(mslName);
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
    aOStream << "Usage: " << progName << " -x xml_in -j joints_out [ -m muscles_out ]" << std::endl;
}
