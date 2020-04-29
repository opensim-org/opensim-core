/* -------------------------------------------------------------------------- *
 *                    OpenSim:  VisualizeModel.cpp                            *
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

/* 
 *  Below is an example of an OpenSim application that loads an OpenSim model
 *  and visualize it in the API visualizer.
 */

// Author:  Ayman Habib

//==============================================================================
//==============================================================================
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/VisualizerUtilities.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Common/LoadOpenSimLibrary.h>

using namespace OpenSim;
using namespace SimTK;
using namespace std;

static void PrintUsage(const char* aProgName, ostream& aOStream);
//______________________________________________________________________________
/**
 * First exercise: create a model that does nothing. 
 */
int main(int argc, char **argv)
{
    LoadOpenSimLibrary("osimActuators");
    try {
        // Create an OpenSim model and set its name
        if (argc < 2) {
            string progName = IO::GetFileNameFromURI(argv[0]);
            cout << "Filename needs to be specified or passed in.\n\n";
            return 1;
        }
        string option = "";
        for (int i = 1; i <= (argc - 1); i++) {
            option = argv[i];
            if ((option == "-help") || (option == "-h") ||
                    (option == "-Help") || (option == "-H") ||
                    (option == "-usage") || (option == "-u") ||
                    (option == "-Usage") || (option == "-U")) {
                PrintUsage(argv[0], cout);
                return 0;
            } else if ((option == "-M") || (option == "-Model")) {
                // Check we're not out of range of argc before indexing
                auto modelFile = argv[i + 1];
                std::string geomertySearchPath = "";
                if (argc >= 5) {
                    string nextArg(argv[i + 2]);
                    if (nextArg == "-G") {
                        geomertySearchPath = string(argv[i + 3]);
                    }
                }
                Model m(modelFile);
                VisualizerUtilities::showModel(m, geomertySearchPath);
                return 0;
            }
            /*
            std::string modelFile = std::string(argv[1]);
            Model model(modelFile);
            std::string resultFile = std::string(argv[2]);
            Storage sto(resultFile);
            VisualizerUtilities::showMotion(model, sto);
            */

            std::string orientationsFile = std::string(argv[1]);
            TimeSeriesTableQuaternion quatTable(orientationsFile);
            int layout = 2;
            if (argc == 3) layout = std::stoi(argv[2]);
            VisualizerUtilities::showOrientationData(quatTable, layout);
        }
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

    return 0;
}
//_____________________________________________________________________________
/**
 * Print the usage for this application
 */
void PrintUsage(const char* aProgName, ostream& aOStream) {
    string progName = IO::GetFileNameFromURI(aProgName);
    aOStream << "\n\n" << progName << ":\n\n";
    aOStream << "Option             Argument               Action / Notes\n";
    aOStream << "------             --------               --------------\n";
    aOStream << "-Help, -H                                 "
                "Print the command-line options for " << progName << ".\n";
    aOStream << "-Model, -M model.osim -G searchPath                               "
                "Visualize model from file, with optional geometry search path.\n";
}