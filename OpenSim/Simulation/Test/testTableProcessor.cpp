/* -------------------------------------------------------------------------- *
 * OpenSim: testTableProcessor.cpp                                            *
 * -------------------------------------------------------------------------- *
 * Copyright (c) 2019 Stanford University and the Authors                     *
 *                                                                            *
 * Author(s): Christopher Dembia                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>
#include <OpenSim/Common/CommonUtilities.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Simulation/TableProcessor.h>

using namespace OpenSim;

TEST_CASE("TableProcessor") {
    Object::registerType(TableProcessor());

    class MyTableOperator : public TableOperator {
        OpenSim_DECLARE_CONCRETE_OBJECT(MyTableOperator, TableOperator);

    public:
        void operate(TimeSeriesTable& table, const Model*) const override {
            table.appendRow(10.0, ~createVectorLinspace(
                                          (int)table.getNumColumns(), 0, 1));
        }
    };
    Object::registerType(MyTableOperator());
    TimeSeriesTable table(std::vector<double>{1, 2, 3},
            SimTK::Test::randMatrix(3, 2), std::vector<std::string>{"a", "b"});

    SECTION("Exceptions") {
        // Exception if no table was provided.
        CHECK_THROWS(TableProcessor().process());
        // No exception if an empty table is provided.
        TableProcessor(TimeSeriesTable{}).process();
        {
            TableProcessor proc(TimeSeriesTable{});
            proc.set_filepath("file.sto");
            CHECK_THROWS_WITH(proc.process(),
                    Catch::Contains("Expected either an in-memory table or a "
                                    "filepath"));
        }
    }

    SECTION("Operators take effect") {
        TableProcessor proc = TableProcessor(table) | MyTableOperator();
        CHECK(proc.process().getNumRows() == 4);
    }

    SECTION("Serialization") {
        STOFileAdapter::write(table, "testTableProcessor_table.sto");
        {
            TableProcessor proc =
                    TableProcessor("testTableProcessor_table.sto") |
                    MyTableOperator();
            proc.print("testTableProcessor_TableProcessor.xml");
        }
        {
            std::unique_ptr<Object> obj(Object::makeObjectFromFile(
                    "testTableProcessor_TableProcessor.xml"));
            auto* proc = dynamic_cast<TableProcessor*>(obj.get());
            TimeSeriesTable out = proc->process();
            CHECK(out.getNumRows() == 4);
        }
    }
}
