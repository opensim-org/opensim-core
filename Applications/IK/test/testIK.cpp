/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testIK.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth, Cassidy Kelly, Peter Loan               *
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
#include <OpenSim/Common/Storage.h>
#include "OpenSim/Common/STOFileAdapter.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/OrientationsReference.h>
#include <OpenSim/Simulation/InverseKinematicsSolver.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

void testInverseKinematicsSolverWithOrientations();

int main()
{
    try {
        testInverseKinematicsSolverWithOrientations();

        InverseKinematicsTool ik1("subject01_Setup_InverseKinematics.xml");
        ik1.run();
        Storage result1(ik1.getOutputMotionFileName()), standard("std_subject01_walk1_ik.mot");
        CHECK_STORAGE_AGAINST_STANDARD(result1, standard, Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 failed");
        cout << "testInverseKinematicsGait2354 passed" << endl;

        InverseKinematicsTool ik2("subject01_Setup_InverseKinematics_NoModel.xml");
        Model mdl("subject01_simbody.osim");
        mdl.initSystem();
        ik2.setModel(mdl);
        ik2.run();
        Storage result2(ik2.getOutputMotionFileName());
        CHECK_STORAGE_AGAINST_STANDARD(result2, standard, Array<double>(0.2, 24), __FILE__, __LINE__, "testInverseKinematicsGait2354 GUI workflow failed");
        cout << "testInverseKinematicsGait2354 GUI workflow passed" << endl;

        InverseKinematicsTool ik3("constraintTest_setup_ik.xml");
        ik3.run();
        cout << "testInverseKinematicsCosntraintTest passed" << endl;
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}

void testInverseKinematicsSolverWithOrientations()
{
    Model model("subject01_simbody.osim");
    // visualize for debugging
    model.setUseVisualizer(true);
    
    SimTK::State& s0 = model.initSystem();

    auto anglesTable = STOFileAdapter::read("std_subject01_walk1_ik.mot");

    size_t nt = anglesTable.getNumRows();
    const auto& coordNames = anglesTable.getColumnLabels();

    // get the coordinates of the model
    const auto& coordinates = model.getComponentList<Coordinate>();

    // get bodies as frames that we want to "sense" rotations
    const auto& bodies = model.getComponentList<Body>();

    std::vector<int> mapDataToModel;

    for (auto& coord : coordinates) {
        int index = -1;
        auto found = std::find(coordNames.begin(), coordNames.end(), coord.getName());
        if(found != coordNames.end())
            index = (int)std::distance(coordNames.begin(), found);
        mapDataToModel.push_back(index);
    }

    TimeSeriesTable_<SimTK::Rotation> orientationsData;
    std::vector<std::string> bodyLabels;
    for (auto& body : bodies) {
        bodyLabels.push_back(body.getName());
    }
    orientationsData.setColumnLabels(bodyLabels);

    cout << "Read in std_subject01_walk1_ik.mot with " << nt << " rows." << endl;
    cout << "Num coordinates in file: " << coordNames.size() << endl;
    cout << "Num of matched coordinates in model: " << mapDataToModel.size() << endl;

    SimTK::RowVector_<SimTK::Rotation> row(bodyLabels.size());

    auto times = anglesTable.getIndependentColumn();
    for (size_t i = 0; i < nt; ++i) {
        const auto& values = anglesTable.getRowAtIndex(i);
        int cnt = 0;
        for (auto& coord : coordinates) {
            if (mapDataToModel[cnt] >= 0) {
                if(coord.getMotionType() == Coordinate::MotionType::Rotational)
                    coord.setValue(s0, 
                        SimTK::convertDegreesToRadians(values(mapDataToModel[cnt++])));
                else
                    coord.setValue(s0, values(mapDataToModel[cnt++]));
            }
        }
        model.realizePosition(s0);
        model.getVisualizer().show(s0);

        size_t nb = 0;
        for (auto& body : bodies) {
            row[nb++] = body.getTransformInGround(s0).R();
        }
        orientationsData.appendRow(times[i], row);
    }
}