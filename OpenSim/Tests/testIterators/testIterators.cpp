/* -------------------------------------------------------------------------- *
 *                       OpenSim:  testIterators.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2014-2014 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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

// Example filter that allows for iterating only through Components that have
// state variables, used for demonstration purposes.
class ComponentWithStateVariables : public ComponentFilter {
public:
    ComponentWithStateVariables() {
        //std::cout << "ComponentWithStateVariables constructed" << std::endl;
    };
    bool isMatch(const Component& comp) const override {
        return (comp.getNumStateVariables()>0);
    };
    ~ComponentWithStateVariables() {
        //std::cout << "ComponentWithStateVariables destructed" << std::endl;
    }
    ComponentWithStateVariables* clone() const {
        return new ComponentWithStateVariables(*this);
    }
};
int main()
{
    try {
        LoadOpenSimLibrary("osimActuators");

        std::string filename = "arm26.osim";

        Model model(filename);
        model.initSystem();
        ComponentList<Component> componentsList = model.getComponentList();
        std::cout << "list begin: " << componentsList.begin()->getName() << std::endl;
        int numComponents = 0;
        for (ComponentList<Component>::const_iterator it = componentsList.begin();
            it != componentsList.end();
            ++it) {
                std::cout << "Iterator is at: " << it->getConcreteClassName() << " " << it->getName() << std::endl;
                numComponents++;
        }
        
        ComponentList<OpenSim::Body> bodiesList = model.getComponentList<OpenSim::Body>();
        int numBodies = 0;
        std::cout << "Bodies list begin: " << bodiesList.begin()->getName() << std::endl;
        for (ComponentList<OpenSim::Body>::const_iterator it = bodiesList.begin();
        it != bodiesList.end();
        ++it) {
            std::cout << "Iterator is at Body: " << it->getName() << std::endl;
            numBodies++;
        }
        // Now we try the post increment variant of the iterator
        std::cout << "Bodies list begin, using post increment: " << bodiesList.begin()->getName() << std::endl;
        int numBodiesPost = 0;
        for (ComponentList<OpenSim::Body>::const_iterator itPost = bodiesList.begin();
            itPost != bodiesList.end();
            itPost++) {
            std::cout << "Iterator is at Body: " << itPost->getName() << std::endl;
            numBodiesPost++;
        }

        int numMuscles = 0;
        std::cout << "Using range-for loop over Muscles: " << std::endl;
        ComponentList<Muscle> musclesList = model.getComponentList<Muscle>();
        for (const Muscle& muscle : musclesList) {
            std::cout << "Iterator is at muscle: " << muscle.getName() << std::endl;
            numMuscles++;
        }
        
        int numGeomPaths = 0;
        ComponentList<GeometryPath> geomPathList = model.getComponentList<GeometryPath>();
        for (const GeometryPath& gpath : geomPathList) {
            numGeomPaths++;
        }
        const OpenSim::Joint& shoulderJnt = model.getJointSet().get(0);
        // cycle thru components under shoulderJnt should return the Joint itself and the Coordinate
        int numJntComponents = 0;
        ComponentList<Component> jComponentsList = shoulderJnt.getComponentList();
        std::cout << "Components/subComponents under Shoulder Joint:" << std::endl;
        for (ComponentList<Component>::const_iterator it = jComponentsList.begin();
            it != jComponentsList.end();
            ++it) {
            std::cout << "Iterator is at: " << it->getConcreteClassName() << " " << it->getName() << std::endl;
            numJntComponents++;
        }
        cout << "Num all components = " << numComponents << std::endl;
        cout << "Num bodies = " << numBodies << std::endl;
        cout << "Num Muscles = " << numMuscles << std::endl;
        cout << "Num GeometryPath components = " << numGeomPaths << std::endl;
        // Components = Model+3Body+3Marker+2(Joint+Coordinate)+6(Muscle+GeometryPath)
        // Should test against 1+#Bodies+#Markers+#Joints+#Constraints+#Coordinates+#Forces+#ForcesWithPath+..
        // Would that account for internal (split-bodies etc.?)
        int numComponentsWithStateVariables = 0;
        ComponentList<ModelComponent> compWithStates = model.getComponentList<ModelComponent>();
        ComponentWithStateVariables myFilter;
        compWithStates.setFilter(myFilter); // Filter is cloned and can be deleted or go out of scope safely
        for (const ModelComponent& comp : compWithStates) {
            cout << comp.getConcreteClassName() << ":" << comp.getName() << endl;
            numComponentsWithStateVariables++;
        }
        //Now test a std::iterator method
        ComponentList<ModelComponent> comps = model.getComponentList<ModelComponent>();
        ComponentList<ModelComponent>::const_iterator skipIter = comps.begin();
        int countSkipComponent = 0;
        while (skipIter != comps.end()){
            cout << skipIter->getConcreteClassName() << ":" << skipIter->getName() << endl;
            std::advance(skipIter, 2);
            countSkipComponent++;
        }

        ASSERT(numComponents == 23); 
        ASSERT(numBodies == model.getNumBodies());
        ASSERT(numBodiesPost == numBodies);
        ASSERT(numMuscles == model.getMuscles().getSize());
        ASSERT(numComponentsWithStateVariables == 11);
        ASSERT(numJntComponents == 2);
        ASSERT(countSkipComponent == 12);
    }
    catch (Exception &ex) {
        ex.print(std::cout);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
