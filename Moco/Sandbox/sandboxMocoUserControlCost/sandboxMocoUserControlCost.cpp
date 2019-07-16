/* -------------------------------------------------------------------------- *
 * OpenSim Moco: sandboxMocoUserControlCost.cpp                               *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Prasanna Sritharan                                              *
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

#include <OpenSim/OpenSim.h>

using namespace OpenSim;

int main() {

    // create a new MocoStudy
    MocoStudy moco;
    moco.setName("mystudy");

    // DEFINE THE OPTIMAL CONTROL PROBLEM
    // ----------------------------------

    // get the MocoProblem
    MocoProblem problem = moco.updProblem();

    // get the model
    Model model("subject01.osim");
    //problem.setModel(model);




    return EXIT_SUCCESS;
}
