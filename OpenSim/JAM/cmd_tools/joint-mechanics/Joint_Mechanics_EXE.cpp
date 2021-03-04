/* -------------------------------------------------------------------------- *
 *                       Joint_Mechanics_EXE.cpp                              *
 * -------------------------------------------------------------------------- *
 * Author(s): Colin Smith                                                     *
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

#include <OpenSim/OpenSim.h>
#include "JointMechanicsTool.h"

using namespace OpenSim;
using SimTK::Vec3;

/** 
*
*arg1: Settings File
*
*
*
*
*
*/
int main(int argc, char *argv[])
{
    
    try {
        Stopwatch watch;

        //Read Inputs
        if (argc != 3) {
            std::cout << "Invalid Number of Arguments. Use form:" << std::endl;
            std::cout << "joint-mechanics plugin_file settings_file" << std::endl;
        }
        std::string plugin_file = argv[1];
        std::string settings_file = argv[2]; 

        LoadOpenSimLibrary(plugin_file, true);

        //Perform Posthoc Joint Mechanics Analysis
        JointMechanicsTool JMP = JointMechanicsTool(settings_file);

        JMP.run();

        std::cout << "\n\nTotal Computation Time: "
            << watch.getElapsedTimeFormatted() << std::endl;
        // **********  END CODE  **********
    }
    catch (OpenSim::Exception ex)
    {
        std::cout << ex.getMessage() << std::endl;
        std::cin.get();
        return 1;
    }
    catch (SimTK::Exception::Base ex)
    {
        std::cout << ex.getMessage() << std::endl;
        std::cin.get();
        return 1;
    }
    catch (std::exception ex)
    {
        std::cout << ex.what() << std::endl;
        std::cin.get();
        return 1;
    }
    catch (...)
    {
        std::cout << "UNRECOGNIZED EXCEPTION" << std::endl;
        std::cin.get();
        return 1;
    }
    return 0;
}
