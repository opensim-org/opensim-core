/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testModelCopy.cpp                         *
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

#include <stdint.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

// Test copying, serializing and deserializing models and verify 
// that the number of bodies (nbods) and the number of attached geometry
// (ngeom) on a given PhysicalFrame (by name) are preserved.
void testCopyModel( const string& fileName, const int nbod, 
                    const string& physicalFrameName, const int ngeom);

int main()
{
    LoadOpenSimLibrary("osimActuators");
    try {

        // Test copying a simple property.
        {
            std::cout << "Test copying a simple property." << std::endl;
            Property<double>* a =
                Property<double>::TypeHelper::create("a", true);
            a->setValue(0.123456789);
            Property<double>* b =
                Property<double>::TypeHelper::create("b", true);
            b->setValue(10.0);

            b->assign(*a);

            cout << "b = " << b->toString() << endl;
            ASSERT(*a == *b);
        }

        // Test copying a an Object property.
        {
            std::cout << "Test copying a object property." << std::endl;
            Body A("A", 0.12345, SimTK::Vec3(0.1, 0.2, 0.3),
                SimTK::Inertia(0.33, 0.22, 0.11));
            Body B;

            Property<Body>* a = Property<Body>::TypeHelper::create("a", true);
            a->setValue(A);
            Property<Body>* b = Property<Body>::TypeHelper::create("b", true);
            b->setValue(B);

            b->assign(*a);

            cout << "b = " << b->toString() << endl;
            ASSERT(*a == *b);

            B = A;
            ASSERT(B == A);
        }

        Model arm("arm26.osim");
        Model armAssigned;

        armAssigned = arm;
        ASSERT(armAssigned == arm);

        LoadOpenSimLibrary("osimActuators");
        testCopyModel("arm26.osim", 2, "ground", 6);
        testCopyModel("Neck3dof_point_constraint.osim", 25, "bodyset/spine", 1);
    }
    catch (const Exception& e) {
        cout << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testCopyModel(const string& fileName, const int nbod, 
    const string& physicalFrameName, const int ngeom)
{
    const size_t mem0 = getCurrentRSS();

    // Automatically finalizes properties by default when loading from file
    Model* model = new Model(fileName);

    // Catch a possible decrease in the memory footprint, which will cause
    // size_t (unsigned int) to wrap through zero.
    const size_t mem1 = getCurrentRSS();
    const size_t increaseInMemory = mem1 > mem0 ? mem1-mem0 : 0;

    cout << "Memory use of '" << fileName <<"' model: " << increaseInMemory/1024
         << "KB" << endl;

    Model *test = nullptr;
    for (int i = 0; i < 10; ++i){
        test = new Model(fileName);
        delete test;
    }
    // verify that the print is const and has no side-effects on the model
    model->print("clone_" + fileName);
    
    Model* modelCopy = new Model(*model);
    modelCopy->finalizeFromProperties();
    // At this point properties should all match. assert that
    ASSERT(*model==*modelCopy);
    ASSERT(model->getActuators().getSize() ==
           modelCopy->getActuators().getSize());

    //SimTK::State& defaultStateOfCopy = modelCopy->initSystem();
    // Compare state
    //defaultState.getY().dump("defaultState:Y");
    //ASSERT ((defaultState.getY()-defaultStateOfCopy.getY()).norm() < 1e-7);

    //  Now delete original model and make sure copy can stand
    Model *cloneModel = modelCopy->clone();

    ASSERT(*model == *cloneModel);
    ASSERT(model->getActuators().getSize() ==
           cloneModel->getActuators().getSize());

    // Compare state again
    
    //SimTK::State& defaultStateOfCopy2 = newModel->initSystem();
    // Compare state
    //ASSERT ((defaultState.getY()-defaultStateOfCopy2.getY()).norm() < 1e-7);
    //ASSERT ((defaultState.getZ()-defaultStateOfCopy2.getZ()).norm() < 1e-7);

    std::string latestFile = "lastest_" + fileName;
    modelCopy->print(latestFile);
    modelCopy->finalizeFromProperties();

    Model* modelSerialized = new Model(latestFile);

    ASSERT(*model == *modelSerialized);
    ASSERT(*modelSerialized == *modelCopy);

    int nb = modelSerialized->getNumBodies();

    const PhysicalFrame& physFrame = 
        modelSerialized->getComponent<PhysicalFrame>(physicalFrameName);

    int ng = physFrame.getProperty_attached_geometry().size();

    ASSERT(nb == nbod);
    ASSERT(ng == ngeom);

    delete model;
    delete modelCopy;
    delete cloneModel;
    delete modelSerialized;

    // New memory footprint.
    const size_t mem2 = getCurrentRSS();
    // Increase in memory footprint.
    const int64_t memory_increase = mem2 > mem1 ? mem2-mem1 : 0;

    cout << "Memory increase AFTER copy and init and delete:  " 
         << double(memory_increase)/mem1*100 << "%." << endl;
}
