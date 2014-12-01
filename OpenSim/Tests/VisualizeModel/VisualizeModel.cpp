/* -------------------------------------------------------------------------- *
 *                    OpenSim:  VisualizeModel.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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

/* 
 *  Below is an example of an OpenSim application that loads an OpenSim model
 *  and visualize it in the API visualizer.
 */

// Author:  Ayman Habib

//==============================================================================
//==============================================================================
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;
//______________________________________________________________________________
/**
 * First exercise: create a model that does nothing. 
 */
int main(int argc, char **argv)
{
    try {
        // Create an OpenSim model and set its name
        if (argc < 2) {
            string progName = IO::GetFileNameFromURI(argv[0]);
            cout << "Filename needs to be specified or passed in.\n\n";
            return 1;
        }

        std::string modelFile = std::string(argv[1]);
        Model osimModel(modelFile);
        //osimModel.print("updated_" + modelFile);
        osimModel.setUseVisualizer(true);
        SimTK::State& si = osimModel.initSystem();
        osimModel.equilibrateMuscles(si);
        osimModel.getMultibodySystem().realize(si, Stage::Velocity);
        osimModel.getVisualizer().show(si);
        getchar(); // Keep Visualizer from dying..
    }
    catch (OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
        return 1;
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        return 1;
    }

    return 0;
}
