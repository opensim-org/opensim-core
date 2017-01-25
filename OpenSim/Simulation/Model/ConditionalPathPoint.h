#ifndef OPENSIM_CONDITIONAL_PATH_POINT_H_
#define OPENSIM_CONDITIONAL_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  ConditionalPathPoint.h                      *
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


// INCLUDE
#include <OpenSim/Simulation/Model/PathPoint.h>

namespace OpenSim {

class Coordinate;
//=============================================================================
//=============================================================================
/**
 * A class implementing a conditional path point, which is a point that
 * is active only for a specified range of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API ConditionalPathPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(ConditionalPathPoint, PathPoint);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_LIST_PROPERTY_SIZE(range, double, 2,
        "The minimum and maximum values that the coordinate can range between, "
        "for which the PathPoint is active. Angular coordinates in radians.");

//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(coordinate, Coordinate,
        "The coordinate whose value determines when "
        "the path point is active according to the specified range.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ConditionalPathPoint();
    virtual ~ConditionalPathPoint();
    void setRangeMin(double minVal);
    void setRangeMax(double maxVal);
    void setCoordinate(const Coordinate& coordinate);

    bool hasCoordinate() const;
    const Coordinate& getCoordinate() const;

    // Override PathPoint methods.
    bool isActive(const SimTK::State& s) const override;

private:
    void constructProperties();
    void updateFromXMLNode(SimTK::Xml::Element& node, int versionNumber) override;
//=============================================================================
};  // END of class ConditionalPathPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_CONDITIONAL_PATH_POINT_H_


