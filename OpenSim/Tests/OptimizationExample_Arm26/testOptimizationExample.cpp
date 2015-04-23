/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testOptimizationExample.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Cassidy Kelly                                                   *
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

// Author: Cassidy Kelly

//==============================================================================
//==============================================================================

#include <OpenSim/OpenSim.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <istream>

using namespace OpenSim;
using namespace std;

#define ARM26_DESIGN_SPACE_DIM 6
#define REF_MAX_VEL 4.5

// Reference solution used for testing.
// This is the result when Thelen2003 muscles are used.
//const static double refControls[ARM26_DESIGN_SPACE_DIM]
//                    = {0.01, 0.01, 0.01, 0.99, 0.99, 0.948937};
// This is the result when Millard2012Equilibrium muscles are used with rigid
// tendons, with an initial guess of {0.01, 0.01, 0.01, 0.99, 0.99, 0.99}.
const static double refControls[ARM26_DESIGN_SPACE_DIM]
    = {0.060, 0.021, 0.022, 0.99, 0.71, 0.30};


int main()
{
    try {
        Storage result("Arm26_noActivation_states.sto"),
                standard("std_Arm26_noActivation_states.sto");
        CHECK_STORAGE_AGAINST_STANDARD(result, standard, Array<double>(0.01, 16),
            __FILE__, __LINE__, "Arm26 no activation states failed");
        cout << "Arm26 no activation states passed\n";

        // Ensure the optimization result acheived a velocity of at least
        // REF_MAX_VEL, and that the control values are within either 20% of the
        // reference values or 0.05 in absolute value.
        ifstream resFile;
        resFile.open("Arm26_optimization_result");
        ASSERT(resFile.is_open(), __FILE__, __LINE__,
               "Can't open optimization result file" );

        SimTK::Array_<double> resVec;
        for ( ; ; ) {
            double tmp;
            resFile >> tmp;
            if (!resFile.good())
                break;
            resVec.push_back(tmp);
        }

        ASSERT(resVec.size() == ARM26_DESIGN_SPACE_DIM+1, __FILE__, __LINE__,
               "Optimization result size mismatch" );
        
        // Ensure the optimizer found a local minimum we expect.
        for (int i = 0; i < ARM26_DESIGN_SPACE_DIM-1; ++i) {
            ASSERT(fabs(resVec[i] - refControls[i])/refControls[i] < 0.2 ||
                   fabs(resVec[i] - refControls[i]) < 0.05, __FILE__, __LINE__,
                   "Control value does not match reference" );
        }
        ASSERT(resVec[ARM26_DESIGN_SPACE_DIM] > REF_MAX_VEL, __FILE__, __LINE__,
               "Optimized velocity smaller than reference" );

        cout << "Arm26 optimization results passed\n";
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
