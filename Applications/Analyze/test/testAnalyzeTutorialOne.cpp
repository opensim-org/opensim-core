/* -------------------------------------------------------------------------- *
 *                    OpenSim:  testAnalyzeTutorialOne.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main()
{
    try {
        AnalyzeTool analyze1("PlotterTool.xml");
        analyze1.getModel().print("testAnalyzeTutorialOne.osim");
        analyze1.run();
        /* Once this runs to completion we'll make the test more meaningful by comparing output
         * to a validated standard. Let's make sure we don't crash during run first! -Ayman 5/29/12 */
        Storage resultFiberLength("testPlotterTool/BothLegs__FiberLength.sto");
        Storage standardFiberLength("std_BothLegs_fiberLength.sto");
        CHECK_STORAGE_AGAINST_STANDARD(resultFiberLength, standardFiberLength, Array<double>(0.0001, 100), __FILE__, __LINE__, "testAnalyzeTutorialOne failed");
        const Model& mdl = analyze1.getModel();
        //mdl.updMultibodySystem()
        analyze1.setStatesFileName("plotterGeneratedStatesHip45.sto");
        //analyze1.setModel(mdl);
        analyze1.setName("BothLegsHip45");
        analyze1.run();
        Storage resultFiberLengthHip45("testPlotterTool/BothLegsHip45__FiberLength.sto");
        Storage standardFiberLength45("std_BothLegsHip45__FiberLength.sto");
        CHECK_STORAGE_AGAINST_STANDARD(resultFiberLengthHip45, standardFiberLength45, Array<double>(0.0001, 100), __FILE__, __LINE__, "testAnalyzeTutorialOne at Hip45 failed");
        cout << "testAnalyzeTutorialOne passed" << endl;
    }
    catch (const exception& e) {
        cout << "testAnalyzeTutorialOne Failed: " << e.what() << endl;
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
