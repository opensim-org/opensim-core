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

#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/MarkerData.h>
#include <OpenSim/Common/STOFileAdapter.h>

#include <tests/Testing.h>

#include <catch2/catch_all.hpp>

#include <fstream>
#include <unordered_set>

using namespace OpenSim;
using namespace std;

// Write STO file using STOFileAdapter, read it multiple times using MarkerData.
// Make sure the file does not contain duplicates in the header.
TEST_CASE("MarkerData: Test STO File Adapter")
{
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

TEST_CASE("MarkerData : general tests")
{
    MarkerData md("dataWithNaNsOfDifferentCases.trc");

    int rStartFrame=-1;
    int rEndFrame=-1;
    md.findFrameRange(0.0, 1.0, rStartFrame, rEndFrame);
    OPENSIM_ASSERT_ALWAYS(rStartFrame==0);
    OPENSIM_ASSERT_ALWAYS(rEndFrame==4);
    md.findFrameRange(0.004, 0.012, rStartFrame, rEndFrame);
    OPENSIM_ASSERT_ALWAYS(rStartFrame==1);
    OPENSIM_ASSERT_ALWAYS(rEndFrame==3);
    // ToBeTested void averageFrames(double aThreshold = -1.0, double aStartTime = -SimTK::Infinity, double aEndTime = SimTK::Infinity);
    OPENSIM_ASSERT_ALWAYS(md.getFileName()=="dataWithNaNsOfDifferentCases.trc");
    Storage storage;
    md.makeRdStorage(storage);
    OPENSIM_ASSERT_ALWAYS(
            md.getUnits().getType()==Units(string("mm")).getType());
    //std::string mm("mm");
    Units lengthUnit = Units::Millimeters;
    OPENSIM_ASSERT_ALWAYS(md.getUnits().getType()==lengthUnit.getType());
    const Array<std::string>& markerNames = md.getMarkerNames();
    OPENSIM_ASSERT_ALWAYS(markerNames.getSize()==14);
    OPENSIM_ASSERT_ALWAYS(md.getMarkerIndex("toe")==0);
    OPENSIM_ASSERT_ALWAYS(md.getMarkerIndex("lASIS")==13);
    OPENSIM_ASSERT_ALWAYS(md.getMarkerIndex("NotFound")==-1);
    OPENSIM_ASSERT_ALWAYS(md.getNumFrames()==5);
    OPENSIM_ASSERT_ALWAYS(md.getStartFrameTime()==0.0);
    OPENSIM_ASSERT_ALWAYS(md.getLastFrameTime()==0.016);
    OPENSIM_ASSERT_ALWAYS(md.getDataRate()==250.);
    OPENSIM_ASSERT_ALWAYS(md.getCameraRate()==250.);
    //ToBeTested md.convertToUnits(Units(Units::Meters));

    MarkerData md2("dataWithNaNsWithSpaces.trc");
    double expectedData[] = {1006.513977, 1014.924316,-195.748917};
    const MarkerFrame& frame2 = md2.getFrame(1);
    OPENSIM_ASSERT_ALWAYS(frame2.getFrameTime()==.01);
    const SimTK::Array_<SimTK::Vec3>& markers = frame2.getMarkers();
    const SimTK::Vec3& m1 = markers[0];
    OPENSIM_ASSERT_ALWAYS(SimTK::isNaN(m1[0]));
    OPENSIM_ASSERT_ALWAYS(SimTK::isNaN(m1[1]));
    OPENSIM_ASSERT_ALWAYS(SimTK::isNaN(m1[2]));
    SimTK::Vec3 diff = (markers[1]-SimTK::Vec3(expectedData[0], expectedData[1], expectedData[2]));
    OPENSIM_ASSERT_ALWAYS(diff.norm() < 1e-7);

    MarkerData md3("dataWithEformat.trc");
    double expectedData3[] = {-1.52E-01,    2.45E-01,   -1.71E+00};
    const MarkerFrame& frame3 = md3.getFrame(0);
    const SimTK::Array_<SimTK::Vec3>& markers3 = frame3.getMarkers();
    /*const SimTK::Vec3& m31 = */markers3[1];
    /* SimTK::Vec3 diff3 = */(markers3[1]-SimTK::Vec3(expectedData3));
    OPENSIM_ASSERT_ALWAYS(diff.norm() < 1e-7);
}
