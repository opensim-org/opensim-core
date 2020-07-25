/* -------------------------------------------------------------------------- *
 *                        OpenSim:  versionUpdate.cpp                         *
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

// INCLUDE
#include <OpenSim/OpenSim.h>  // clock(), clock_t, CLOCKS_PER_SEC
#include <OpenSim/Common/IO.h>
#include <OpenSim/version.h>
using namespace OpenSim;
using namespace std;

static void PrintUsage(const char *aProgName, ostream &aOStream);

//_____________________________________________________________________________
/**
 * Main routine to read an object and write it to the same object with latest XML format.
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

    DEPRECATED:    versionUpdate inputFileName outputFileName
    REPLACED WITH: opensim-cmd update-file inputFileName outputFileName
    )";
    log_warn(deprecationNotice);

    // PARSE COMMAND LINE
    string option = "";
    string inputFileName = "";
    string outputFileName = "";
    if(argc<2) {
        PrintUsage(argv[0], cout);
        return(-1);
    }
    inputFileName = string(argv[1]);
    outputFileName = (argc == 2)?(outputFileName=inputFileName): string(argv[2]);
    // ERROR CHECK
    if(inputFileName=="") {
        log_error("versionUpdate.exe: An input file must be specified.");
        PrintUsage(argv[0], cout);
        return(-1);
    }
    
    string::size_type extSep = inputFileName.rfind(".");

    if (extSep == string::npos) {
        log_error("versionUpdate.exe: Unknown file type encountered. "
                  "File extension must be specified.");
        PrintUsage(argv[0], cout);
        return 1;// if '_fileName' contains path information...
    }
    std::string extension  = inputFileName.substr(extSep);
    if (extension == ".sto"){
        Storage stg(inputFileName);
        stg.print(outputFileName);
        return (0);
    }
    if (extension != ".xml" && extension != ".osim") {
        log_error("versionUpdate.exe: Unknown file type encountered. "
                  "Only .xml, .osim and .sto files are supported.");
        PrintUsage(argv[0], cout);
        return 1;// if '_fileName' contains path information...
    }

    Object* newObject = Object::makeObjectFromFile(inputFileName);

    newObject->print(outputFileName);

    //----------------------------
    // Catch any thrown exceptions
    //----------------------------
    } catch(const Exception& x) {
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
    aOStream<<"\n\n"<<progName<<":\n"<<GetVersionAndDate()<<" inputFile outputFile\n\n";
    aOStream<<"Option              Argument         Action / Notes\n";
    aOStream<<"------              --------         --------------\n";
    aOStream<<"inputFileName        Specify the name of the OpenSim xml file (.xml, .osim) to perform versionUpdate on.\n";
    aOStream<<"outputFileName       Specify the name of the output file.\n";
}

