/* -------------------------------------------------------------------------- *
 *                     OpenSim:  ConditionalPathPoint.cpp                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include "ConditionalPathPoint.h"
#include <OpenSim/Simulation/SimbodyEngine/Coordinate.h>

//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

//=============================================================================
// CONSTRUCTOR(S) AND DESTRUCTOR
//=============================================================================
//_____________________________________________________________________________
/**
 * Default constructor.
 */
ConditionalPathPoint::ConditionalPathPoint() : PathPoint()
{
    constructProperties();
}

//_____________________________________________________________________________
/**
 * Destructor.
 */
ConditionalPathPoint::~ConditionalPathPoint()
{}

//_____________________________________________________________________________
/*
 * Override default implementation by object to intercept and fix the XML node
 * underneath the model to match current version
 */
void ConditionalPathPoint::updateFromXMLNode(SimTK::Xml::Element& node, 
                                             int versionNumber)
{
    if (versionNumber <= 20001) {
        // Version has to be 1.6 or later, otherwise assert
        XMLDocument::renameChildNode(node, "coordinates", "coordinate");
    }
    if (versionNumber < 30505) {
        // replace old properties with latest use of Sockets
        SimTK::Xml::element_iterator coord = node.element_begin("coordinate");
        std::string coord_name("");
        if (coord != node.element_end())
            coord->getValueAs<std::string>(coord_name);
        XMLDocument::addConnector(node, "Connector_Coordinate_", "coordinate", coord_name);
    }

    // Call base class now assuming _node has been corrected for current version
    Super::updateFromXMLNode(node, versionNumber);
}

//_____________________________________________________________________________
/*
 * Connect properties to local pointers.
 */
void ConditionalPathPoint::constructProperties()
{
    Array<double> defaultRange(0.0, 2); //two values of the range
    constructProperty_range(defaultRange);
}


//_____________________________________________________________________________
/*
 * Set the coordinate that this point is linked to.
 */
void ConditionalPathPoint::setCoordinate(const Coordinate& coordinate)
{
    connectSocket_coordinate(coordinate);
}

bool ConditionalPathPoint::hasCoordinate() const
{
    return getSocket<Coordinate>("coordinate").isConnected();
}

const Coordinate& ConditionalPathPoint::getCoordinate() const
{
    return getConnectee<Coordinate>("coordinate");
}


//_____________________________________________________________________________
/*
 * Set the range min.
 */
void ConditionalPathPoint::setRangeMin(double minVal)
{
    set_range(0, minVal);
}

//_____________________________________________________________________________
/*
 * Set the range max.
 */
void ConditionalPathPoint::setRangeMax(double maxVal)
{
    set_range(1, maxVal);
}

//_____________________________________________________________________________
/*
 * Determine if this point is active by checking the value of the
 * coordinate that it is linked to.
 */
bool ConditionalPathPoint::isActive(const SimTK::State& s) const
{
    if (getSocket<Coordinate>("coordinate").isConnected()) {
        double value = getConnectee<Coordinate>("coordinate").getValue(s);
        if (value >= get_range(0) - 1e-5 &&
             value <= get_range(1) + 1e-5)
            return true;
    }
    return false;
}
