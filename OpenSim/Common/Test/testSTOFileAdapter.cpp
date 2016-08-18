/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testSTOFileAdapter.cpp                *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

#include "OpenSim/Common/Adapters.h"

#include <unordered_set>
#include <fstream>
#include <cstdio>

std::string getNextToken(std::istream& stream, 
                         const std::string& delims) {
    using namespace OpenSim;

    std::string token{};
    char ch{};
    while(stream.get(ch)) {
        if(delims.find(ch) == std::string::npos) {
            token.push_back(ch);
            break;
        }
    }
    while(stream.get(ch)) {
        if(delims.find(ch) != std::string::npos)
            break;
        token.push_back(ch);
    }

    return token;
}

void testFailed(const std::string& filename,
                const std::string& origtoken,
                const std::string& copiedtoken) {
    using namespace OpenSim;

    throw Exception{"Test failed: Original and copied TRC files do not match. "
            "Filename = '" + filename + "'. "
            "Expected token = " + origtoken + ". "
            "Copied token = " + copiedtoken + "."};
}

void compareHeaders(std::ifstream& filenameA,
                    std::ifstream& filenameB) {
    using namespace OpenSim;

    std::unordered_set<std::string> headerA{}, headerB{};

    std::string line{};
    while(std::getline(filenameA, line)) {
        if(line.find("endheader") != std::string::npos)
            break;

        if(line.empty())
            continue;

        // Ignore the key-value pair specifying the datatype. It may not be 
        // present in old files.
        if(line.find("DataType") != std::string::npos)
            continue;

        headerA.insert(line);
    }
    while(std::getline(filenameB, line)) {
        if(line.find("endheader") != std::string::npos)
            break;
        
        if(line.empty())
            continue;

        // Ignore the key-value pair specifying the datatype. It may not be 
        // present in old files.
        if(line.find("DataType") != std::string::npos)
            continue;

        headerB.insert(line);
    }

    if(headerA != headerB)
        throw Exception{"Test failed: Original and copied headers do not "
                        "match."};
}

void compareFiles(const std::string& filenameA, 
                  const std::string& filenameB) {
    // Delimiters include newline.
    const std::string delims{" \t\n"};

    std::ifstream fileA{filenameA};
    std::ifstream fileB{filenameB};

    compareHeaders(fileA, fileB);

    while(fileA && fileB) {
        const auto tokenA = getNextToken(fileA, delims);
        const auto tokenB = getNextToken(fileB, delims);
        if(tokenA != tokenB) {
            double d_tokenA{};
            double d_tokenB{};
            try {
                d_tokenA = std::stod(tokenA);
                d_tokenB = std::stod(tokenB);
            } catch(std::invalid_argument&) {
                testFailed(filenameA, tokenA, tokenB);
            }

            if(d_tokenA != d_tokenB && 
               !(std::isnan(d_tokenA) && 
                 std::isnan(d_tokenB))) {
                testFailed(filenameA, tokenA, tokenB);
            }
        }
    }
}

int main() {
    using namespace OpenSim;

    std::vector<std::string> filenames{};
    filenames.push_back("std_subject01_walk1_ik.mot");
    filenames.push_back("gait10dof18musc_subject01_walk_grf.mot");
    filenames.push_back("runningModel_GRF_data.mot");
    filenames.push_back("subject02_running_arms_ik.mot");
    filenames.push_back("subject01_walk1_grf.mot");
    std::string tmpfile{"testmotfileadapter.mot"};

    for(const auto& filename : filenames) {
        STOFileAdapter_<double> stofileadapter{};
        auto table = stofileadapter.read(filename);
        stofileadapter.write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    for(const auto& filename : filenames) {
        auto table = FileAdapter::readFile(filename).at("table");
        DataAdapter::InputTables tables{};
        tables.emplace(std::string{"table"}, table.get());
        FileAdapter::writeFile(tables, tmpfile);
        compareFiles(filename, tmpfile);
    }

    for(const auto& filename : filenames) {
        TimeSeriesTable table{filename};
        STOFileAdapter_<double>::write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::remove(tmpfile.c_str());

    // Test reading/writing -- STOFileAdapter_<SimTK::Vec3>.
    {
    std::string fileA{"testSTOFileAdapter_A.sto"};
    std::string fileB{"testSTOFileAdapter_B.sto"};
    TimeSeriesTable_<SimTK::Vec3> tableVec3{};
    SimTK::Vec3 elem{0, 1, 2};
    tableVec3.setColumnLabels({"c0", "c1", "c2"});
    for(auto t = 0; t < 10; ++t) {
        elem += 1;
        tableVec3.appendRow(t, {elem, elem, elem});
    }
    STOFileAdapter_<SimTK::Vec3>::write(tableVec3, fileA);
    auto tableVec3_copy = STOFileAdapter_<SimTK::Vec3>::read(fileA);
    auto tableVec3_ptr = FileAdapter::readFile(fileA).at("table");
    DataAdapter::InputTables inputTables{};
    inputTables.emplace(std::string{"table"}, tableVec3_ptr.get());
    FileAdapter::writeFile(inputTables, fileB);
    compareFiles(fileA, fileB);
    std::remove(fileA.c_str());
    std::remove(fileB.c_str());
    }

    // Test reading/writing -- STOFileAdapter<SimTK::Vec6>.
    {
    std::string fileA{"testSTOFileAdapter_A.sto"};
    std::string fileB{"testSTOFileAdapter_B.sto"};
    TimeSeriesTable_<SimTK::Vec6> tableVec6{};
    SimTK::Vec6 elem{0, 1, 2, 3, 4, 5};
    tableVec6.setColumnLabels({"c0", "c1", "c2"});
    for(auto t = 0; t < 10; ++t) {
        elem += 1;
        tableVec6.appendRow(t, {elem, elem, elem});
    }
    STOFileAdapter_<SimTK::Vec6>::write(tableVec6, fileA);
    auto tableVec6_copy = STOFileAdapter_<SimTK::Vec6>::read(fileA);
    auto tableVec6_ptr = FileAdapter::readFile(fileA).at("table");
    DataAdapter::InputTables inputTables{};
    inputTables.emplace(std::string{"table"}, tableVec6_ptr.get());
    FileAdapter::writeFile(inputTables, fileB);
    compareFiles(fileA, fileB);
    std::remove(fileA.c_str());
    std::remove(fileB.c_str());
    }

    // Test reading/writing -- STOFileAdapter<SimTK::SpatialVec>.
    {
    std::string fileA{"testSTOFileAdapter_A.sto"};
    std::string fileB{"testSTOFileAdapter_B.sto"};
    TimeSeriesTable_<SimTK::SpatialVec> tableSpatialVec{};
    SimTK::SpatialVec elem{{0, 1, 2}, {3, 4, 5}};
    tableSpatialVec.setColumnLabels({"c0", "c1", "c2"});
    for(auto t = 0; t < 10; ++t) {
        elem += 1;
        tableSpatialVec.appendRow(t, {elem, elem, elem});
    }
    STOFileAdapter_<SimTK::SpatialVec>::write(tableSpatialVec, fileA);
    auto tableSpatialVec_copy = STOFileAdapter_<SimTK::SpatialVec>::read(fileA);
    auto tableSpatialVec_ptr = FileAdapter::readFile(fileA).at("table");
    DataAdapter::InputTables inputTables{};
    inputTables.emplace(std::string{"table"}, tableSpatialVec_ptr.get());
    FileAdapter::writeFile(inputTables, fileB);
    compareFiles(fileA, fileB);
    std::remove(fileA.c_str());
    std::remove(fileB.c_str());
    }

    return 0;
}
