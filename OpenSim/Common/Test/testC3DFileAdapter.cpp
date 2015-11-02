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

#include "OpenSim/Common/C3DFileAdapter.h"

#include "OpenSim/Common/TRCFileAdapter.h"

#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <chrono>

int main() {
    const size_t num_iters{1000000000};
    const size_t num_letters{10};
    const size_t num_elems{1000};
    std::string letters{"abcdefghijklmnopqrstuvwxyz"};
    std::vector<double> vec{};
    std::map<std::string, size_t> umap{};
    std::vector<std::string> keys{};
    for(size_t i = 0; i < num_elems; ++i) {
        vec.push_back(i);
        std::string key{};
        for(size_t l = 0; l < num_letters; ++l)
            key += letters.at(rand() % num_letters);
        umap.insert({key, i});
        keys.push_back(key);
    }

    std::vector<size_t> index{};
    for(size_t i = 0; i < num_iters; ++i) {
        // index.push_back(rand() % num_elems);
        index.push_back(i % num_elems);
    }

    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        for(size_t i = 0; i < num_iters; ++i) {
            auto result = index[i];
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> secs{end - start};
        std::cout << "indices -- " << secs.count() 
                  << " secs" << std::endl;
    }

    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        for(size_t i = 0; i < num_iters; ++i) {
            auto result = vec[index[i]];
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> secs{end - start};
        std::cout << "std::vector -- " << secs.count() 
                  << " secs" << std::endl;
    }

    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        for(size_t i = 0; i < num_iters; ++i) {
            auto result = keys[index[i]];
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> secs{end - start};
        std::cout << "keys -- " << secs.count() 
                  << " secs" << std::endl;
    }

    {
        std::chrono::time_point<std::chrono::system_clock> start, end;
        start = std::chrono::system_clock::now();
        for(size_t i = 0; i < num_iters; ++i) {
            auto result = umap[keys[index[i]]];
        }
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> secs{end - start};
        std::cout << "std::unordered_map -- " << secs.count() 
                  << " secs" << std::endl;
    }



    using namespace OpenSim;

    C3DFileAdapter c3d_adapter{};
    auto tables = c3d_adapter.read("/home/shrik/Downloads/"
                                   // "walking5.c3d");
                                   // "walking2.c3d");
                                   // "treadMillRunning.c3d");
                                   // "singleLegLanding.c3d");
                                   "singleLeglanding_2.c3d");

    auto&    marker_table = std::get<0>(tables);
    auto&     force_table = std::get<1>(tables);
    auto& usr_force_table = std::get<2>(tables);

    // std::cout << marker_table->getNumRows() << " "
    //           << marker_table->getNumColumns() << std::endl;
    // std::cout << force_table->getNumRows() << " "
    //           << force_table->getNumColumns() << std::endl;
    // std::cout << usr_force_table->getNumRows() << " "
    //           << usr_force_table->getNumColumns() << std::endl;


    // TRCFileAdapter trc_adapter{};
    // trc_adapter.write(*marker_table, "/home/shrik/Downloads/markers.trc");

    // force_table->updTableMetaData().setValueForKey("Units", std::string{"N"});

    // trc_adapter.write(*force_table, "/home/shrik/Downloads/forces.trc");
    
    return 0;
}
