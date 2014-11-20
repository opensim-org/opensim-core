/* -------------------------------------------------------------------------- *
 *                         OpenSim:  testStorage.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include <fstream>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>

using namespace OpenSim;
using namespace std;

int main() {
    try {
        // Create a storge from a std file "std_storage.sto"
        //ofstream checkRunDirFile("rundir.txt");
        //checkRunDirFile << "Run from here:\n\n";
        //checkRunDirFile.close();
        string stdLabels[] = {"time", "v1", "v2"};
        Storage* st = new Storage("test.sto");
        // time[\t]v1[\t]v2
        // 1.[\t]   10.0[Space]20
        // 2.[\t\t] 20.0[\t]40
        ASSERT(st->getSize()==2);
        const Array<std::string> &lbls = st->getColumnLabels();
        ASSERT(lbls.getSize()==3);
        int i=0;
        for(i=0; i<lbls.getSize(); i++){
            ASSERT(lbls[i]==stdLabels[i]);
        }
        double val;
        for(i=0; i<st->getSize(); i++){
            StateVector& row = (*st->getStateVector(i));
            ASSERT(row.getTime()==i+1);
            ASSERT(row.getData()[0]==row.getTime()*10.0);
            row.getDataValue(0, val);
            ASSERT(val==row.getTime()*10.0);
            ASSERT(row.getData()[0]==row.getTime()*10.0);
            ASSERT(row.getData()[1]==row.getTime()*20.0);
        }
        int ncol = st->getSmallestNumberOfStates();
        ASSERT(ncol==2);
        Array<double> col(SimTK::CNT<SimTK::Real>::getNaN(),4);
        st->getDataColumn(1, col);
        ASSERT(col[0]==20.);
        ASSERT(col[1]==40.0);
    
        ASSERT(st->getStateIndex("v2")==1);

        Storage st2("testDiff.sto");
        // Test Comparison
        double diff = st->compareColumn(st2, stdLabels[1], 0.);
        ASSERT(fabs(diff) < 1E-7);
        diff = st->compareColumn(st2, stdLabels[2], 0.);
        ASSERT(fabs(diff) < 1E-7);

        delete st;
    }
    catch (const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
