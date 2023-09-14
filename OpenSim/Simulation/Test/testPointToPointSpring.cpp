/* -------------------------------------------------------------------------- *
*                       OpenSim:  testPointToPointSpring.cpp                 *
* -------------------------------------------------------------------------- *
* The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
* See http://opensim.stanford.edu and the NOTICE file for more information.  *
* OpenSim is developed at Stanford University and supported by the US        *
* National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
* through the Warrior Web program.                                           *
*                                                                            *
* Copyright (c) 2005-2023 Stanford University and the Authors                *
* Author(s): Adam Kewley                                                     *
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

#include <OpenSim/Simulation/Model/PointToPointSpring.h>

#define CATCH_CONFIG_MAIN
#include <OpenSim/Auxiliary/catch.hpp>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

#include <memory>

// repro related to #3485
//
// this ensures that the "sane case" finalizes fine
TEST_CASE("PointToPointSpring finalizeConnectionsWhenBothSidesConnectedToDifferentBodiesIsFine")
{
    OpenSim::Model m;

    OpenSim::Body* b1 = new OpenSim::Body{"b1", 1.0, {}, {}};
    m.addBody(b1);

    OpenSim::Body* b2 = new OpenSim::Body{"b2", 1.0, {}, {}};
    m.addBody(b2);

    CHECK_NOTHROW(m.finalizeConnections());

    auto p2p = std::unique_ptr<OpenSim::PointToPointSpring>{new OpenSim::PointToPointSpring{}};
    p2p->connectSocket_body1(*b1);
    p2p->connectSocket_body2(*b2);
    m.addForce(p2p.release());

    CHECK_NOTHROW(m.finalizeConnections());
}

TEST_CASE("PointToPointSpring finalizeConnectionsWhenBothSidesConnectedToSameBodyThrows")
{
    OpenSim::Model m;

    OpenSim::Body* b = new OpenSim::Body{"b", 1.0, {}, {}};
    m.addBody(b);

    CHECK_NOTHROW(m.finalizeConnections());

    auto p2p = std::unique_ptr<OpenSim::PointToPointSpring>{new OpenSim::PointToPointSpring{}};
    p2p->connectSocket_body1(*b);
    p2p->connectSocket_body2(*b);
    m.addForce(p2p.release());

    CHECK_THROWS_AS(m.finalizeConnections(), OpenSim::Exception);
}