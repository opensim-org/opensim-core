/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMocoTrack.cpp                                         *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2017 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Nicholas Bianco                                                 *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <Moco/osimMoco.h>
#include <OpenSim/Common/LogManager.h>
#include <OpenSim/OpenSim.h>

#include <iostream>
#include <functional>

using namespace OpenSim;


int main() {

    // get model
    Model model("FAIDC1.osim");
    //model.initSystem();

    // get table
    TableProcessor tp = TableProcessor("FAIDC1_INITIAL_WALK01_ik.mot");


    // get original column headers
    TimeSeriesTable oldiktable = tp.process();
    std::vector<std::string> oldikcols = oldiktable.getColumnLabels();

    // print original column headers
    std::cout << "\nBefore:\n";
    for (auto col : oldikcols) std::cout << col << "\n";


    // update column headers
    tp.append(TabOpUpdColLabelFullPath(model, 2));


    // get updated column headers
    TimeSeriesTable newiktable = tp.process();
    std::vector<std::string> newikcols = newiktable.getColumnLabels();
    
    // print update column headers
    std::cout << "\nAfter:\n";
    for (auto col : newikcols) std::cout << col << "\n";



    return EXIT_SUCCESS;
}
