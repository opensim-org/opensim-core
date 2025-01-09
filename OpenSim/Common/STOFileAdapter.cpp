/* -------------------------------------------------------------------------- *
 *                         OpenSim:  STOFileAdapter.cpp                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2018 Stanford University and the Authors                *
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

#include "STOFileAdapter.h"

namespace OpenSim {

std::shared_ptr<DataAdapter> 
createSTOFileAdapterForReading(const std::string& fileName) {
    std::ifstream file{fileName};

    std::regex keyvalue{R"((.*)=(.*))"};
    std::string line{};
    while(std::getline(file, line)) {
        if(line.find("endheader") != std::string::npos)
            break;

        std::smatch matchRes{};
        if(std::regex_match(line, matchRes, keyvalue)) {
            auto key = matchRes[1].str();
            auto value = matchRes[2].str();
            if(!key.empty() && 
               !value.empty() &&
               key.find("DataType") != std::string::npos) {
                using namespace SimTK;

                if(value == "double")
                    return std::make_shared<STOFileAdapter_<double>>();
                else if(value == "Vec2")
                    return std::make_shared<STOFileAdapter_<Vec2>>();
                else if(value == "Vec3")
                    return std::make_shared<STOFileAdapter_<Vec3>>();
                else if(value == "Vec4")
                    return std::make_shared<STOFileAdapter_<Vec4>>();
                else if(value == "Vec5")
                    return std::make_shared<STOFileAdapter_<Vec5>>();
                else if(value == "Vec6")
                    return std::make_shared<STOFileAdapter_<Vec6>>();
                else if(value == "Vec7")
                    return std::make_shared<STOFileAdapter_<Vec7>>();
                else if(value == "Vec8")
                    return std::make_shared<STOFileAdapter_<Vec8>>();
                else if(value == "Vec9")
                    return std::make_shared<STOFileAdapter_<Vec9>>();
                else if(value == "Vec10")
                    return std::make_shared<STOFileAdapter_<Vec<10>>>();
                else if(value == "Vec11")
                    return std::make_shared<STOFileAdapter_<Vec<11>>>();
                else if(value == "Vec12")
                    return std::make_shared<STOFileAdapter_<Vec<12>>>();
                else if(value == "UnitVec3")
                    return std::make_shared<STOFileAdapter_<UnitVec3>>();
                else if(value == "Quaternion")
                    return std::make_shared<STOFileAdapter_<Quaternion>>();
                else if(value == "SpatialVec")
                    return std::make_shared<STOFileAdapter_<SpatialVec>>();
                else {
                    OPENSIM_THROW(STODataTypeNotSupported,
                                  value);
                }
            }
        }
    }
    // The file does not seem to have a DataType field, and is therefore likely
    // a version 1.0 STO file. These files only supported double as the column
    // data type; try to read the file with type double.
    return std::make_shared<STOFileAdapter_<double>>();
}

template <typename T>
std::shared_ptr<STOFileAdapter_<T>>
makeAdapter(const AbstractDataTable* absTable) {
    if (auto table = dynamic_cast<const TimeSeriesTable_<T>*>(absTable)) {
        return std::make_shared<STOFileAdapter_<T>>();
    }
    return {};
}

std::shared_ptr<DataAdapter>
createSTOFileAdapterForWriting(const DataAdapter::InputTables& absTables) {
    using namespace SimTK;

    auto* absTable = absTables.at("table");

    // Try derived class before base class.
    
    if (auto adapter = makeAdapter<UnitVec3>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Quaternion>(absTable)) return adapter;
    if (auto adapter = makeAdapter<SpatialVec>(absTable)) return adapter;
    if (auto adapter = makeAdapter<double>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec2>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec3>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec4>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec5>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec6>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec7>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec8>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec9>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec<10>>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec<11>>(absTable)) return adapter;
    if (auto adapter = makeAdapter<Vec<12>>(absTable)) return adapter;

    OPENSIM_THROW(STODataTypeNotSupported,
                  "<unknown>");
}

} // namespace OpenSim
