/* -------------------------------------------------------------------------- *
 *                            OpenSim:  example.cpp                           *
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

#include "OpenSim/Common/DataAdapter.h"

#include <iostream>
#include <fstream>


class A {
public:
    virtual void prepareForReading() = 0;
};

class B : public A {
};

class C : public B {
public:
    void prepareForReading() {
        std::cout << "C::prepareForReading()" << std::endl;
    }
};


int main(void) {
    using namespace OpenSim;

    DataAdapter::registerDataAdapter("trc", TRCAdapter{});

    TRCAdapter::Table table{};

    auto file_adapter = FileAdapter::createAdapter("trc");

    file_adapter->setFilename("/home/shrik/opensim-core/OpenSim/Wrapping/Java/"
                              "Matlab/testData/Subject01/MarkerData/"
                              "walk_free_01.trc");

    file_adapter->prepareForReading(table);

    file_adapter->read();

    std::cout << table.getNumRows() << " rows" << std::endl;
    std::cout << table.getNumColumns() << " columns" << std::endl; 

    return 0;
}
