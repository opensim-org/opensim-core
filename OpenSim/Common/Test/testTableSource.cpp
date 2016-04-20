/* -------------------------------------------------------------------------- *
 *                     OpenSim:  testTableSource.cpp                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2016 Stanford University and the Authors                     *
 * Author(s):                                                                 *
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
#include "OpenSim/Common/TableSource.h"
#include "OpenSim/Common/Reporter.h"

#include <iostream>

class Sub : public OpenSim::Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(Sub, Component);
public:
    OpenSim_DECLARE_OUTPUT_FOR_STATE_VARIABLE(subState);
    Sub() = default;
    virtual ~Sub() = default;
private:
    void extendAddToSystem(SimTK::MultibodySystem &system) const override {
        Super::extendAddToSystem(system);
        addStateVariable("subState", SimTK::Stage::Dynamics);
    }

    void computeStateVariableDerivatives(const SimTK::State& s) const override {
        double deriv = exp(-2.0*s.getTime());
        setStateVariableDerivativeValue(s, "subState", deriv);
    }
}; // class Sub

class TheWorld : public OpenSim::Component {
    OpenSim_DECLARE_CONCRETE_OBJECT(TheWorld, Component);
public:
    TheWorld() : Component() {
        // Constructing own properties, connectors, inputs or connectors? Must 
        // invoke!
        constructInfrastructure();
    }

    TheWorld(const std::string& fileName, bool updFromXMLNode = false)
        : Component(fileName, updFromXMLNode) {
        // have to construct this Component's properties so that deserialization
        // from XML has a place to go.
        constructInfrastructure();
        // Propagate XML file values to properties 
        updateFromXMLDocument();
        // add components listed as properties as sub components.
        finalizeFromProperties();
    }

    void add(Component* comp) {
        addComponent(comp);
        // Edit Sub 
        /*Sub& subc = */updMemberSubcomponent<Sub>(intSubix);
    }

    // Top level connection method for this all encompassing component, TheWorld
    void connect() {
        Super::connect(*this);
    }
    void buildUpSystem(SimTK::MultibodySystem& system) { 
        connect();
        addToSystem(system);
    }

    const SimTK::SimbodyMatterSubsystem& getMatterSubsystem() const { 
        return *matter; 
    }

    SimTK::SimbodyMatterSubsystem& updMatterSubsystem() const { 
        return *matter; 
    }

    const SimTK::GeneralForceSubsystem& getForceSubsystem() const { 
        return *forces; 
    }

    SimTK::GeneralForceSubsystem& updForceSubsystem() const { 
        return *forces; 
    }

protected:
    // Component interface implementation
    void extendAddToSystem(SimTK::MultibodySystem& system) const override {
        if (system.hasMatterSubsystem()){
            matter = system.updMatterSubsystem();
        }
        else{
            // const Sub& subc = getMemberSubcomponent<Sub>(intSubix);

            SimTK::SimbodyMatterSubsystem* old_matter = matter.release();
            delete old_matter;
            matter = new SimTK::SimbodyMatterSubsystem(system);

            SimTK::GeneralForceSubsystem* old_forces = forces.release();
            delete old_forces;
            forces = new SimTK::GeneralForceSubsystem(system);

            SimTK::Force::UniformGravity gravity(*forces, *matter, 
                                                 SimTK::Vec3(0, -9.816, 0));
            fix = gravity.getForceIndex();

            system.updMatterSubsystem().setShowDefaultGeometry(true);
        }
    }

private:
    // Keep track of pointers to the underlying computational subsystems. 
    mutable SimTK::ReferencePtr<SimTK::SimbodyMatterSubsystem> matter;
    mutable SimTK::ReferencePtr<SimTK::GeneralForceSubsystem> forces;

    // keep track of the force added by the component
    mutable SimTK::ForceIndex fix;

    MemberSubcomponentIndex intSubix{constructSubcomponent<Sub>("internalSub")};
}; // class TheWorld


template<typename RowVec>
void assertEqual(const RowVec& a, const RowVec& b) {
    assert(a.nrow() == b.nrow());
    assert(a.ncol() == b.ncol());
    for(int i = 0; i < a.ncol(); ++i)
        ASSERT_EQUAL(a[i], b[i], 1e-10);
}

int main() {
    using namespace OpenSim;
    using namespace SimTK;

    TimeSeriesTable table{};
    table.setColumnLabels({"0", "1", "2", "3"});
    SimTK::RowVector_<double> row{4, double{0}};
    for(unsigned i = 0; i < 4; ++i)
        table.appendRow(0.00 + 0.25 * i, row + i);

    std::cout << "Contents of the table :" << std::endl;
    std::cout << table << std::endl;

    auto tableSource = new TableSource{table};

    auto tableReporter = new TableReporter_<double, double>{};

    // Define the Simbody system
    MultibodySystem system;

    TheWorld theWorld;
    theWorld.setName("World");

    theWorld.add(tableSource);
    theWorld.add(tableReporter);

    tableReporter->updInput("inputs").connect(tableSource->getOutput("column"));

    theWorld.finalizeFromProperties();

    theWorld.connect();
    theWorld.buildUpSystem(system);

    const auto& report = tableReporter->getReport();

    State s = system.realizeTopology();

    s.setTime(0);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(0)  , report.getRowAtIndex(0));

    s.setTime(0.1);
    tableReporter->report(s);
    row = RowVector_<double>{4, 0.4};
    assertEqual(row.getAsRowVectorView(), report.getRowAtIndex(1));

    s.setTime(0.25);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(1)  , report.getRowAtIndex(2));

    s.setTime(0.4);
    tableReporter->report(s);
    row = RowVector_<double>{4, 1.6};
    assertEqual(row.getAsRowVectorView(), report.getRowAtIndex(3));

    s.setTime(0.5);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(2)  , report.getRowAtIndex(4));

    s.setTime(0.6);
    tableReporter->report(s);
    row = RowVector_<double>{4, 2.4};
    assertEqual(row.getAsRowVectorView(), report.getRowAtIndex(5));

    s.setTime(0.75);
    tableReporter->report(s);
    assertEqual(table.getRowAtIndex(3)  , report.getRowAtIndex(6));

    std::cout << "Report: " << std::endl;
    std::cout << report << std::endl;

    return 0;
}
