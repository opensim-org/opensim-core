/* -------------------------------------------------------------------------- *
 *                            OpenSim:  testC3DFileAdapter.cpp                *
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

int main() {
    using namespace OpenSim;

    std::vector<std::string> filenames{};
    filenames.push_back("aStaticTrial_2.c3d");
    filenames.push_back("aStaticTrial.c3d");
    filenames.push_back("doubleLegLanding.c3d");
    filenames.push_back("jogging.c3d");
    filenames.push_back("sideCutting.c3d");
    filenames.push_back("singleLeglanding_2.c3d");
    filenames.push_back("singleLegLanding.c3d");
    filenames.push_back("treadMillRunning.c3d");
    filenames.push_back("walking2.c3d");
    filenames.push_back("walking5.c3d");

    for(const auto& filename : filenames) {
        C3DFileAdapter c3d_adapter{};
        auto tables = c3d_adapter.read(filename);

        auto& marker_table = tables.at("markers");
        auto&  force_table = tables.at("markers");

        if(marker_table->getNumRows() != 0) {
            marker_table->updTableMetaData().setValueForKey("Units", std::string{"mm"});
            TRCFileAdapter trc_adapter{};
            trc_adapter.write(*marker_table, filename + ".markers.trc");
        }

        if(force_table->getNumRows() != 0) {
            force_table->updTableMetaData().setValueForKey("Units", std::string{"mm"});
            TRCFileAdapter trc_adapter{};
            trc_adapter.write(*force_table, filename + ".forces.trc");
        }
    }

    for(const auto& filename : filenames) {
        auto tables = FileAdapter::readFile(filename);

        using MT = TimeSeriesTableVec3;
        using FT = TimeSeriesTableVec3;

        auto marker_table = dynamic_cast<MT*>(tables.at("markers").get());
        auto  force_table = dynamic_cast<FT*>(tables.at("forces").get());

    if(marker_table->getNumRows() != 0) {
    marker_table->updTableMetaData().setValueForKey("Units", std::string{"mm"});
    TRCFileAdapter trc_adapter{};
    trc_adapter.write(*marker_table, filename + ".markers.trc");
    }

    if(force_table->getNumRows() != 0) {
    force_table->updTableMetaData().setValueForKey("Units", std::string{"mm"});
    TRCFileAdapter trc_adapter{};
    trc_adapter.write(*force_table, filename + ".forces.trc");
    }
    }

    return 0;
}
