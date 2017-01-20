/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testMemoryLeaks.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <iostream>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <windows.h>
#include <psapi.h>

using namespace OpenSim;
using namespace std;

unsigned int Memory_Usage()
{
    HANDLE process=GetCurrentProcess();
    PROCESS_MEMORY_COUNTERS counters;
    GetProcessMemoryInfo(process,&counters,sizeof(counters));
    return (unsigned int)counters.WorkingSetSize;
}

int main(int argc,char **argv)
{
    LoadOpenSimLibrary("osimActuators");
    LoadOpenSimLibrary("osimSimbodyEngine");
    LoadOpenSimLibrary("osimSimmKinematicsEngine");

    std::string filename = "fullbody_test/FullBody_markerchange.osim";
    if(argc>1) filename=argv[1];

    std::cout << "START " << Memory_Usage() << std::endl;

    Model *originalModel = new Model(filename);
    originalModel->setup();

    std::cout << "BASELINE " << Memory_Usage() << std::endl;

    try {
    for(int i=0; i<3; i++) {
        std::cout << std::endl;
        std::cout << i << " BEFORE " << Memory_Usage() << std::endl;
        Model *model = new Model(*originalModel);
        std::cout << i << " AFTER LOAD " << Memory_Usage() << std::endl;
        model->setup();
        std::cout << i << " AFTER SETUP " << Memory_Usage() << std::endl;
        delete model;
        std::cout << i << " AFTER DELETE " << Memory_Usage() << std::endl;
    }

    delete originalModel;
    std::cout << "AFTER FINAL DELETE " << Memory_Usage() << std::endl;

    } catch (Exception &ex) {
        ex.print(std::cout);
    }
}
