/* -------------------------------------------------------------------------- *
 *                   OpenSim:  testOptimizationExample.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#define REF_MAX_VEL 4.8

// Reference solution used for testing.
/*
    TRIlong control value = 0.0231781
    TRIlat control value = 0.0195227
    TRImed control value = 0.0224382
    BIClong control value = 0.987157
    BICshort control value = 0.835002
    BRA control value = 0.608971

    Maximum hand velocity = 4.92489m/s
*/
const static double refControls[ARM26_DESIGN_SPACE_DIM]
    = { 0.0231781, 0.0195227, 0.0224382, 0.987157, 0.835002, 0.608971 };


int main()
{
    try {
        // Load optimization results
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

        // Ensure the optimization result achieved a velocity of at least
        // REF_MAX_VEL
        ASSERT(resVec[ARM26_DESIGN_SPACE_DIM] > REF_MAX_VEL, __FILE__, __LINE__,
            "Optimized velocity smaller than reference");

        
        // Ensure the optimizer found controls within 10% of "optimal" controls
        // Or that the absolute difference in controls with optimal < 0.01
        for (int i = 0; i < ARM26_DESIGN_SPACE_DIM; ++i) {
            double relErr = fabs(resVec[i] - refControls[i]) / refControls[i];
            double absErr = fabs(resVec[i] - refControls[i]);
            cout << i << ": relErr = " << relErr << " | absErr = " << absErr <<endl;
            ASSERT( (relErr < 0.1) || (absErr < 0.01), __FILE__, __LINE__,
                   "Control value does not match reference" );
        }


        cout << "Arm26 optimization results passed\n";
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
