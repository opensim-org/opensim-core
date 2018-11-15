/* -------------------------------------------------------------------------- *
 *                    OpenSim:  TugOfWar1_CreateModel.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Jeffrey A. Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton,    *
 *            Samuel R. Hamner                                                *
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

/* 
 *  Below is an example of an OpenSim application that provides its own 
 *  main() routine.  This application is a forward simulation of tug-of-war between two
 *  muscles pulling on a block.
 */

// Author:  Jeff Reinbolt, Ayman Habib, Ajay Seth, Jack Middleton, Samuel Hamner

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

//______________________________________________________________________________
/**
 * First exercise: create a model that does nothing. 
 */
int main()
{
    try {
        // Create an OpenSim model and set its name
        Model osimModel;
        osimModel.setName("tugOfWar");
    }
    catch (const OpenSim::Exception& ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (const std::exception& ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    std::cout << "OpenSim example completed successfully.\n";
    std::cin.get();
    return 0;
}
