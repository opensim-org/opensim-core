/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testSetIterators.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2014-2016 Stanford University and the Authors                *
 * Author(s): Dimitar Stanev                                                  *
 * Contributer(s) :                                                           *
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
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

/*
Tests: 
    Array<T> iterators
    Set<T> iterators that are aggregation to ArrayPtrs<T>
*/
int main()
{
    try {
        LoadOpenSimLibrary("osimActuators");

        std::string filename = "arm26.osim";

        Model model(filename);

        // test Array<T> iterator
        Array<double> data;
        double value = 1;
        data.append(value);
        data.append(value);
        for (auto& d : data)
        {
            assert(d == value);
            cout << d << endl;
            d = value + 1;
        }
        for (const auto d : data)
        {
            assert(d == value + 1);
            cout << d << endl;
        }

        // Use pre-increment of iterator and change the name of the objects
        Set<Body>& bodiesSet = model.updBodySet();
        int numBodies = 0;
        string newName = "changed";
        std::cout << "Bodies list begin: " << bodiesSet.begin()->getName() << std::endl;
        for (Set<Body>::iterator it = bodiesSet.begin(); it != bodiesSet.end(); ++it) 
        {
            //check conversation from non-const to const
            Set<Body>::const_iterator constIt = it; 

            std::cout << "Iterator is at Body: " << it->getName() << std::endl;
            it->setName(newName);
            numBodies++;
        }

        // Now we try the post increment variant of the const_iterator
        std::cout << "Bodies list begin, using post increment: " 
            << bodiesSet.begin()->getName() << std::endl;
        int numBodiesPost = 0;
        for (Set<Body>::const_iterator it = bodiesSet.begin(); it != bodiesSet.end(); it++) {
            assert(it->getName() == newName);
            std::cout << "Iterator is at Body: " << it->getName() << std::endl;
            numBodiesPost++;
        }

        // Test range loop
        int numMuscles = 0;
        std::cout << "Using range-for loop over Muscles: " << std::endl;
        const Set<Muscle>& muscleSet = model.getMuscles();
        cout << "Muscles: " << muscleSet.getSize() << endl;
        for (const Muscle& muscle : muscleSet) {
            std::cout << "Iterator is at muscle: " << muscle.getName() << std::endl;
            numMuscles++;
        }
        
        cout << "Num bodies = " << numBodies << std::endl;
        cout << "Num Muscles = " << numMuscles << std::endl;

        int numJoints = 0;
        Set<Joint>& jointSet = model.updJointSet();
        for (auto& joint : jointSet) {
            joint.setName(newName);
            assert(joint.getName() == newName);
            numJoints++;
        }
      
        ASSERT(numBodies == model.getNumBodies());
        ASSERT(numBodiesPost == numBodies);
        ASSERT(numMuscles == model.getMuscles().getSize());
        ASSERT(numJoints == model.getNumJoints());
        
    }
    catch (Exception &ex) {
        ex.print(std::cout);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
