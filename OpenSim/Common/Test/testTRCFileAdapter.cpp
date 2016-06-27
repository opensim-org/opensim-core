/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testTRCFileAdapter.cpp                *
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

void compareFiles(const std::string& filenameA, 
                  const std::string& filenameB) {
    // Delimiters include newline.
    const std::string delims{" \t\n"};

    std::ifstream fileA{filenameA};
    std::ifstream fileB{filenameB};
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
    filenames.push_back("TRCFileWithNANs.trc");
    filenames.push_back("testEformatParsing.trc");
    filenames.push_back("testNaNsParsing.trc");
    filenames.push_back("subject01_synthetic_marker_data.trc");
    filenames.push_back("constraintTest.trc");
    filenames.push_back("subject01_static.trc");
    std::string tmpfile{"testtrcfileadapter.trc"};

    for(const auto& filename : filenames) {
        TRCFileAdapter trcfileadapter{};
        auto table = trcfileadapter.read(filename);
        trcfileadapter.write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    for(const auto& filename : filenames) {
        auto table = FileAdapter::readFile(filename).at("markers");
        DataAdapter::InputTables tables{};
        tables.emplace(std::string{"markers"}, table.get());
        FileAdapter::writeFile(tables, tmpfile);
        compareFiles(filename, tmpfile);
    }

    for(const auto& filename : filenames) {
        TimeSeriesTableVec3 table{filename};
        TRCFileAdapter::write(table, tmpfile);
        compareFiles(filename, tmpfile);
    }

    std::remove(tmpfile.c_str());

    return 0;
}
