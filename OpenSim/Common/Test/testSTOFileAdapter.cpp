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

#include "OpenSim/Common/Adapters.h"
#include "OpenSim/Common/CommonUtilities.h"
#include <cstdio>
#include <fstream>
#include <unordered_set>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>

using namespace OpenSim;

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

void compareHeaders(std::ifstream& fileA,
                    std::ifstream& fileB) {
    using namespace OpenSim;

    std::unordered_set<std::string> headerA{}, headerB{};

    std::string line{};
    while(std::getline(fileA, line)) {
        IO::TrimWhitespace(line);

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

        if(line.find("OpenSimVersion") != std::string::npos)
            continue;

        headerA.insert(line);
    }
    while(std::getline(fileB, line)) {
        IO::TrimWhitespace(line);

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

        if(line.find("OpenSimVersion") != std::string::npos)
            continue;

        headerB.insert(line);
    }

    if(headerA != headerB)
        throw Exception{
                "Test failed: Original and copied headers do not match."};
}

void compareFiles(const std::string& filenameA, 
                  const std::string& filenameB) {
    // Delimiters include carriage return and newline.
    const std::string delims{" \t\r\n"};

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
    {
        TimeSeriesTable_<T> table{};
        table.setColumnLabels({"c0", "c1", "c2"});
        for (auto t = 0; t < 10; ++t) {
            auto elem = createObject<T>();
            table.appendRow(t, {elem, elem, elem});
        }
        STOFileAdapter_<T>::write(table, fileA);
        TimeSeriesTable_<T> table_copy(fileA);
        auto table_ptr = FileAdapter::createAdapterFromExtension(fileA)->read(fileA).at("table");
        DataAdapter::InputTables inputTables{};
        inputTables.emplace(std::string{"table"}, table_ptr.get());
        FileAdapter::writeFile(inputTables, fileB);
        compareFiles(fileA, fileB);
        std::remove(fileA.c_str());
        std::remove(fileB.c_str());
    }

    {
        // Empty table.
        TimeSeriesTable_<T> table{};
        STOFileAdapter_<T>::write(table, fileA);
        TimeSeriesTable_<T> table_copy(fileA);
        auto table_ptr = FileAdapter::createAdapterFromExtension(fileA)->read(fileA).at("table");
        DataAdapter::InputTables inputTables{};
        inputTables.emplace(std::string{"table"}, table_ptr.get());
        FileAdapter::writeFile(inputTables, fileB);
        compareFiles(fileA, fileB);
        std::remove(fileA.c_str());
        std::remove(fileB.c_str());
    }

    {
        // No columns.
        TimeSeriesTable_<T> table{std::vector<double>{0, 0.1, 0.2}};
        STOFileAdapter_<T>::write(table, fileA);
        TimeSeriesTable_<T> table_copy(fileA);
        auto table_ptr = FileAdapter::createAdapterFromExtension(fileA)->read(fileA).at("table");
        DataAdapter::InputTables inputTables{};
        inputTables.emplace(std::string{"table"}, table_ptr.get());
        FileAdapter::writeFile(inputTables, fileB);
        compareFiles(fileA, fileB);
        std::remove(fileA.c_str());
        std::remove(fileB.c_str());
    }
}

TEST_CASE("STOFileAdapter") {

    std::cout << "Testing reading/writing STOFileAdapter_<double>" << std::endl;
    std::vector<std::string> filenames{};
    filenames.push_back("std_subject01_walk1_ik.mot");
    filenames.push_back("gait10dof18musc_subject01_walk_grf.mot");
    // Ensure we can read files with \r\n line endings (even on UNIX).
    filenames.push_back("gait10dof18musc_ik_CRLF_line_ending.mot");
    filenames.push_back("runningModel_GRF_data.mot");
    filenames.push_back("subject02_running_arms_ik.mot");
    filenames.push_back("subject01_walk1_grf.mot");
    std::string tmpfile{"testmotfileadapter.mot"};

    std::cout << "Testing STOFileAdapter::read() and STOFileAdapter::write()"
              << std::endl;
    for (const auto& filename : filenames) {
        std::cout << "  " << filename << std::endl;
        STOFileAdapter_<double> stofileadapter{};
        TimeSeriesTable table(filename);
        stofileadapter.write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::cout << "Testing FileAdapter::read() and FileAdapter::writeFile()"
              << std::endl;
    for (const auto& filename : filenames) {
        std::cout << "  " << filename << std::endl;
        auto table = FileAdapter::createAdapterFromExtension(filename)
                             ->read(filename)
                             .at("table");
        DataAdapter::InputTables tables{};
        tables.emplace(std::string{"table"}, table.get());
        FileAdapter::writeFile(tables, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::cout << "Testing TimeSeriesTable and STOFileAdapter::write()"
              << std::endl;
    for (const auto& filename : filenames) {
        std::cout << "  " << filename << std::endl;
        TimeSeriesTable table{filename};
        STOFileAdapter_<double>::write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::remove(tmpfile.c_str());

    // test detection of invalid column labels
    TimeSeriesTable table{};
    SimTK_TEST_MUST_THROW_EXC(
            table.setColumnLabels({"c1", "c2", "", "c4"}), InvalidColumnLabel);
    SimTK_TEST_MUST_THROW_EXC(table.setColumnLabels({"c1", "  ", "c3", "\t"}),
            InvalidColumnLabel);
    table.setColumnLabels({"c1", "c2", "c3", "c4"});
    SimTK_TEST_MUST_THROW_EXC(
            table.setColumnLabel(3, " \n hi"), InvalidColumnLabel);
    SimTK_TEST_MUST_THROW_EXC(
            table.setColumnLabel(2, "hel\rlo"), InvalidColumnLabel);
    SimTK_TEST_MUST_THROW_EXC(
            table.setColumnLabel(1, "ABC\tDEF"), InvalidColumnLabel);
    SimTK_TEST_MUST_THROW_EXC(
            table.setColumnLabel(0, "   ABC DEF   "), InvalidColumnLabel);
    // space within the label should be OK
    table.setColumnLabel(1, "ABC DEF");

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

    std::cout << "Testing exception for reading an empty file" << std::endl;
    std::string emptyFileName("testSTOFileAdapter_empty.sto");
    FileRemover fileRemover(emptyFileName);
    std::ofstream emptyFile(emptyFileName);
    SimTK_TEST_MUST_THROW_EXC(
            FileAdapter::createAdapterFromExtension(emptyFileName)
                    ->read(emptyFileName),
            FileIsEmpty);
}

TEST_CASE("Reading STO version 1.0 using FileAdapter::read()") {
    // There was a bug where the FileAdapter::read() could not handle
    // version-1.0 STO files because the "DataType" metadata was required to
    // determine the template argument for STOFileAdapter (Issue #1725).
    // This test ensures that bug is fixed (test.sto is version 1.0).
    auto outputTables = FileAdapter::createAdapterFromExtension("test.sto")->read("test.sto");
    SimTK_TEST(outputTables["table"]->getNumRows() == 2);
    SimTK_TEST(outputTables["table"]->getNumColumns() == 2);
}

TEST_CASE("Trimming whitespace in metadata values") {
    const std::string filename = "testing_metadata_whitespace.sto";
    {
        TimeSeriesTable table;
        table.addTableMetaData("inDegrees", std::string("yes\t\t\t"));
        STOFileAdapter::write(table, filename);
    }
    TimeSeriesTable table(filename);
    CHECK(table.getTableMetaDataAsString("inDegrees") == "yes");
}




