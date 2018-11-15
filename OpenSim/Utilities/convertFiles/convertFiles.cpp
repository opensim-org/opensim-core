/* -------------------------------------------------------------------------- *
 *                               convertFiles.cpp                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
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
#include <string.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace std;

int main(int argc,char **argv)
{
#ifndef STATIC_OSIM_LIBS
    LoadOpenSimLibrary("osimTools");
    LoadOpenSimLibrary("osimActuators");
    LoadOpenSimLibrary("osimAnalyses");
    LoadOpenSimLibrary("osimSimulation");
    LoadOpenSimLibrary("osimSimbodyEngine");
#endif

    int offset=0;
    if(argc>1 && string(argv[1])=="-offline") offset++;
    else IO::SetPrintOfflineDocuments(false);
    if(argc<1+offset+2) {
        std::cerr << "Not enough arguments: <INPUT_FILE> <OUTPUT_FILE>" << std::endl;
        exit(1);
    }
    string fileName = argv[offset+1];
    string outputFileName = argv[offset+2];
    Object *obj = Object::makeObjectFromFile(fileName);
    std::cout << fileName << " -> " << outputFileName;
    if(!obj) std::cout << " FAILED" << std::endl;
    else {
        std::cout << std::endl;
        //IO::SetGFormatForDoubleOutput(true);
        obj->copy()->print(outputFileName);
    }
}
