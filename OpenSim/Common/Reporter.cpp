/* -------------------------------------------------------------------------- *
 *                          OpenSim:  Reporter.cpp                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

#include "Reporter.h"
#include <OpenSim/Common/TimeSeriesTable.h>

using namespace SimTK;

namespace OpenSim {

class OutputPeriodicReporter : public SimTK::PeriodicEventReporter {
public:
    OutputPeriodicReporter(const AbstractReporter& owner,
        const MultibodySystem& system,
        const Real reportInterval) : PeriodicEventReporter(reportInterval), 
        _owner(owner), _system(system) {
    }

    void handleEvent(const State& state) const override {
        // This should be triggered every (interval) time units.
        SimTK_ASSERT(state.getTime() == getNextEventTime(state, true),
            "Reporter did not report at specified time interval.");
        // Guarantee that the system is at the reporting stage
        if (state.getSystemStage() < SimTK::Stage::Report) {
            _system.realize(state, SimTK::Stage::Report);
        }
        // delegate back to the OpenSim::Reporter to do the reporting
        _owner.report(state);
    }

private:
    const AbstractReporter& _owner;
    const MultibodySystem& _system;
};

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
// Default constructor.
AbstractReporter::AbstractReporter()
{
    setNull();
    constructProperties();
}

AbstractReporter::AbstractReporter(SimTK::Xml::Element& node) : Super(node)
{
    setNull();
    constructProperties();
}



//=============================================================================
// CONSTRUCTION METHODS
//=============================================================================

//_____________________________________________________________________________
// Set the data members of this Reporter to their null values.
void AbstractReporter::setNull()
{
    setAuthors("Ajay Seth");
    setName("reporter");
}

//_____________________________________________________________________________
// Define properties.
void AbstractReporter::constructProperties()
{
    constructProperty_report_time_interval(0.0);
}

// Create an underlying SimTK::Reporter to represent the OpenSim::Reporter in the 
// computational system.  Create a SimTK::Reporter::Custom by default.
void AbstractReporter::extendAddToSystem(SimTK::MultibodySystem& system) const
{
    Super::extendAddToSystem(system);

    double reportInterval = get_report_time_interval();

    if (reportInterval >= SimTK::Eps) {
        auto* simTKreporter =
            new OpenSim::OutputPeriodicReporter(*this, system, reportInterval);
        system.addEventReporter(simTKreporter);
    }
}

void AbstractReporter::extendRealizeReport(const SimTK::State& state) const
{
    if (get_report_time_interval() == 0) {
        report(state);
    }
}

void AbstractReporter::report(const SimTK::State& s) const
{
    implementReport(s);
}


} // end of namespace OpenSim
