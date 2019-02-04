/* -------------------------------------------------------------------------- *
 *                        OpenSim:  testMarkerData.cpp                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include <OpenSim/Common/STOFileAdapter.h>

#include <unordered_set>

using namespace OpenSim;
using namespace std;

// Write STO file using STOFileAdapter, read it multiple times using MarkerData.
// Make sure the file does not contain duplicates in the header.
void testSTOFileAdapterWithMarkerData() {
    TimeSeriesTable table{};
    table.setColumnLabels({"0.x", "0.y", "0.z", "1.x", "1.y", "1.z"});
    table.appendRow(0.1, {1, 1, 1, 1, 1, 1});
    table.appendRow(0.2, {2, 2, 2, 2, 2, 2});
    table.appendRow(0.3, {3, 3, 3, 3, 3, 3});

    std::string filename{"table.sto"};
    STOFileAdapter_<double>::write(table, filename);

    MarkerData markerdata1{filename};
    MarkerData markerdata2{filename};
    MarkerData markerdata3{filename};

    std::ifstream filestream{filename};
    std::unordered_set<std::string> headerlines{};
    for(std::string line; std::getline(filestream, line); )
        if(!headerlines.insert(line).second)
            throw Exception{"Test failed: found duplicates in header."};

    std::remove(filename.c_str());
}

int main() {
    // Create a storage from a std file "std_storage.sto"
    try {
        MarkerData md("dataWithNaNsOfDifferentCases.trc");

        int rStartFrame=-1;
        int rEndFrame=-1;
        md.findFrameRange(0.0, 1.0, rStartFrame, rEndFrame);
        ASSERT(rStartFrame==0);
        ASSERT(rEndFrame==4);
        md.findFrameRange(0.004, 0.012, rStartFrame, rEndFrame);
        ASSERT(rStartFrame==1);
        ASSERT(rEndFrame==3);
        // ToBeTested void averageFrames(double aThreshold = -1.0, double aStartTime = -SimTK::Infinity, double aEndTime = SimTK::Infinity);
        ASSERT(md.getFileName()=="dataWithNaNsOfDifferentCases.trc");
        Storage storage;
        md.makeRdStorage(storage);
        ASSERT(md.getUnits().getType()==Units(string("mm")).getType(), __FILE__, __LINE__);
        //std::string mm("mm");
        Units lengthUnit = Units::Millimeters;
        ASSERT(md.getUnits().getType()==lengthUnit.getType(), __FILE__, __LINE__);
        const Array<std::string>& markerNames = md.getMarkerNames();
        ASSERT(markerNames.getSize()==14, __FILE__, __LINE__);
        ASSERT(md.getMarkerIndex("toe")==0, __FILE__, __LINE__);
        ASSERT(md.getMarkerIndex("lASIS")==13, __FILE__, __LINE__);
        ASSERT(md.getMarkerIndex("NotFound")==-1, __FILE__, __LINE__);
        ASSERT(md.getNumFrames()==5, __FILE__, __LINE__);
        ASSERT(md.getStartFrameTime()==0.0, __FILE__, __LINE__);
        ASSERT(md.getLastFrameTime()==0.016, __FILE__, __LINE__);
        ASSERT(md.getDataRate()==250., __FILE__, __LINE__);
        ASSERT(md.getCameraRate()==250., __FILE__, __LINE__);
        //ToBeTested md.convertToUnits(Units(Units::Meters));

        MarkerData md2("dataWithNaNsWithSpaces.trc");
        double expectedData[] = {1006.513977, 1014.924316,-195.748917};
        const MarkerFrame& frame2 = md2.getFrame(1);
        ASSERT(frame2.getFrameTime()==.01, __FILE__, __LINE__);
        const SimTK::Array_<SimTK::Vec3>& markers = frame2.getMarkers();
        const SimTK::Vec3& m1 = markers[0];
        ASSERT(SimTK::isNaN(m1[0]), __FILE__, __LINE__);
        ASSERT(SimTK::isNaN(m1[1]), __FILE__, __LINE__);
        ASSERT(SimTK::isNaN(m1[2]), __FILE__, __LINE__);
        SimTK::Vec3 diff = (markers[1]-SimTK::Vec3(expectedData[0], expectedData[1], expectedData[2]));
        ASSERT(diff.norm() < 1e-7, __FILE__, __LINE__);

        MarkerData md3("dataWithEformat.trc");
        double expectedData3[] = {-1.52E-01,    2.45E-01,   -1.71E+00};
        const MarkerFrame& frame3 = md3.getFrame(0);
        const SimTK::Array_<SimTK::Vec3>& markers3 = frame3.getMarkers();
        /*const SimTK::Vec3& m31 = */markers3[1];    
        /* SimTK::Vec3 diff3 = */(markers3[1]-SimTK::Vec3(expectedData3));
        ASSERT(diff.norm() < 1e-7, __FILE__, __LINE__);

        testSTOFileAdapterWithMarkerData();
    }
    catch(const Exception& e) {
        e.print(cerr);
        return 1;
    }
    cout << "Done" << endl;
    return 0;
}
