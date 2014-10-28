/* -------------------------------------------------------------------------- *
 *                       OpenSim:  iteratorExample.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

using namespace OpenSim;
using namespace std;

int main(int argc, char **argv)
{
    try {
        LoadOpenSimLibrary("osimActuators");

        std::string filename = "arm26.osim";
        if (argc > 1) filename = argv[1];

        Model *model = new Model(filename);
        model->initSystem();
        ComponentList<Component> componentsList = model->getComponentList();
        std::cout << "list begin: " << componentsList.begin()->getName() << std::endl;
        for (ComponentList<Component>::iterator it = componentsList.begin();
            it != componentsList.end();
            ++it) {
            std::cout << "Iterator is at: " << it->getConcreteClassName() << " " << it->getName() << std::endl;
        }
        
        ComponentList<OpenSim::Body> bodiesList = model->getComponentList<OpenSim::Body>();
        std::cout << "Bodies list begin: " << bodiesList.begin()->getName() << std::endl;
        for (ComponentList<OpenSim::Body>::iterator it = bodiesList.begin();
        it != bodiesList.end();
        ++it) {
        std::cout << "Iterator is at Body: " << it->getName() << std::endl;
        }

        std::cout << "Using range-for loop over Muscles: " << std::endl;
        ComponentList<Muscle> musclesList = model->getComponentList<Muscle>();
        for (const Muscle& muscle : musclesList) {
        std::cout << "Iterator is at muscle: " << muscle.getName() << std::endl;
        }
        
        delete model;

    }
    catch (Exception &ex) {
        ex.print(std::cout);
    }
}
