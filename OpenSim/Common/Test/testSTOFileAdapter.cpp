/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testSTOFileAdapter.cpp                *
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

#include <OpenSim/Auxiliary/auxiliaryTestFunctions.h>
#include "OpenSim/Common/Adapters.h"
#include "OpenSim/Common/Storage.h"

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

    throw Exception{"Test failed: Original and copied STO files do not match. "
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

        // Ignore the key-value pair specifying the version number. Old files
        // will have older version number.
        if(line.find("version") != std::string::npos)
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

        // Ignore the key-value pair specifying the version number. Old files
        // will have older version number.
        if(line.find("version") != std::string::npos)
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

template<typename T>
T createObject() {
    static double init{0};
    T elem{};
    for(auto i = 0; i < elem.size(); ++i)
        elem[i] = init++;
  
  return elem;
}

template<>
SimTK::UnitVec3 createObject<SimTK::UnitVec3>() {
   return {0, 0, 0};
}

template<>
SimTK::Quaternion createObject<SimTK::Quaternion>() {
    return {0, 0, 0, 0};
}

template<typename T>
void testReadingWriting() {
    using namespace OpenSim;
  
    std::string fileA{"testSTOFileAdapter_A.sto"};
    std::string fileB{"testSTOFileAdapter_B.sto"};
    TimeSeriesTable_<T> table{};
    table.setColumnLabels({"c0", "c1", "c2"});
    for(auto t = 0; t < 10; ++t) {
        auto elem = createObject<T>();
        table.appendRow(t, {elem, elem, elem});
    }
    STOFileAdapter_<T>::write(table, fileA);
    auto table_copy = STOFileAdapter_<T>::read(fileA);
    auto table_ptr = FileAdapter::readFile(fileA).at("table");
    DataAdapter::InputTables inputTables{};
    inputTables.emplace(std::string{"table"}, table_ptr.get());
    FileAdapter::writeFile(inputTables, fileB);
    compareFiles(fileA, fileB);
    std::remove(fileA.c_str());
    std::remove(fileB.c_str());
}

int main() {
    using namespace OpenSim;

    std::cout << "Testing reading/writing STOFileAdapter_<double>"
              << std::endl;
    std::vector<std::string> filenames{};
    filenames.push_back("std_subject01_walk1_ik.mot");
    filenames.push_back("gait10dof18musc_subject01_walk_grf.mot");
    filenames.push_back("runningModel_GRF_data.mot");
    filenames.push_back("subject02_running_arms_ik.mot");
    filenames.push_back("subject01_walk1_grf.mot");
    std::string tmpfile{"testmotfileadapter.mot"};

    std::cout << "Testing STOFileAdapter::read() and STOFileAdapter::write()"
              << std::endl;
    for(const auto& filename : filenames) {
        std::cout << " " << filename << std::endl;
        STOFileAdapter_<double> stofileadapter{};
        auto table = stofileadapter.read(filename);
        stofileadapter.write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::cout << "Testing FileAdapter::readFile() and FileAdapter::writeFile()"
              << std::endl;
    for(const auto& filename : filenames) {
        std::cout << "  " << filename << std::endl;
        auto table = FileAdapter::readFile(filename).at("table");
        DataAdapter::InputTables tables{};
        tables.emplace(std::string{"table"}, table.get());
        FileAdapter::writeFile(tables, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::cout << "Testing TimeSeriesTable and STOFileAdapter::write()"
              << std::endl;
    for(const auto& filename : filenames) {
        std::cout << "  " << filename << std::endl;
        TimeSeriesTable table{filename};
        STOFileAdapter_<double>::write(table, tmpfile);
        compareFiles(filename, tmpfile);

        // Make sure Storage can read the new STO/MOT files.
        Storage storage{tmpfile};
        ASSERT(table.getNumRows() == storage.getSize());
        ASSERT(table.getNumColumns() == storage.getColumnLabels().getSize());
        for(auto i = 0; i < storage.getSize(); ++i) {
            const auto& statevec = storage.getStateVector(i);
            ASSERT(statevec->getTime() == table.getIndependentColumn().at(i));
            const auto& storageRow = statevec->getData();
            const auto& tableRow = table.getRowAtIndex(i);
            for(auto j = 0; j < table.getNumColumns(); ++j)
                ASSERT(storageRow.get(j) == tableRow[j]);
        }
    }

    std::remove(tmpfile.c_str());

    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::Vec2>"
              << std::endl;
    testReadingWriting<SimTK::Vec2>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::Vec3>"
              << std::endl;
    testReadingWriting<SimTK::Vec3>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::Vec4>"
              << std::endl;
    testReadingWriting<SimTK::Vec4>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::Vec5>"
              << std::endl;
    testReadingWriting<SimTK::Vec5>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::Vec6>"
              << std::endl;
    testReadingWriting<SimTK::Vec6>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::UnitVec3>"
              << std::endl;
    testReadingWriting<SimTK::UnitVec3>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::Quaternion>"
              << std::endl;
    testReadingWriting<SimTK::Quaternion>();
    std::cout << "Testing reading/writing STOFileAdapter_<SimTK::SpatialVec>"
              << std::endl;
    testReadingWriting<SimTK::SpatialVec>();
    std::cout << "\nAll tests passed!" << std::endl;


    return 0;
}
