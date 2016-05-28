#ifndef OPENSIM_MOVING_PATH_POINT_H_
#define OPENSIM_MOVING_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  MovingPathPoint.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Function.h>
#include <OpenSim/Simulation/Model/PathPoint.h>

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

class Coordinate;
class Model;
class GeometryPath;
class SimbodyEngine;

//=============================================================================
//=============================================================================
/**
 * A class implementing a moving muscle point, which is a muscle point that
 * moves in a body's reference frame as a function of a coordinate.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMULATION_API MovingPathPoint : public PathPoint {
OpenSim_DECLARE_CONCRETE_OBJECT(MovingPathPoint, PathPoint);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_OPTIONAL_PROPERTY(x_location, Function,
        "Function defining the x component of the point's location.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(y_location, Function,
        "Function defining the y component of the point's location.");

    OpenSim_DECLARE_OPTIONAL_PROPERTY(z_location, Function,
        "Function defining the z component of the point's location.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    MovingPathPoint();
    virtual ~MovingPathPoint();

    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;

    const Coordinate& getXCoordinate() const;
    const Coordinate& getYCoordinate() const;
    const Coordinate& getZCoordinate() const;

    void setXCoordinate(const Coordinate& coordinate);
    void setYCoordinate(const Coordinate& coordinate);
    void setZCoordinate(const Coordinate& coordinate);

    // Override methods from PathPoint.
    bool isActive(const SimTK::State& s) const override { return true; }

    // Get the local location of the MovingPathPoint in its Frame
    SimTK::Vec3 getLocation(const SimTK::State& s) const;
    // Get the local velocity of the MovingPathPoint in its Frame
    SimTK::Vec3 getVelocity(const SimTK::State& s) const;

    SimTK::Vec3 getdPointdQ(const SimTK::State& s) const override; 

   void scale(const SimTK::Vec3& aScaleFactors) override;

private:
    void constructProperties() override;
    void constructConnectors() override;
    void extendFinalizeFromProperties() override;

    SimTK::Vec3 calcLocationInGround(const SimTK::State& state) const override;
    SimTK::Vec3 calcVelocityInGround(const SimTK::State& state) const override;
    SimTK::Vec3 calcAccelerationInGround(const SimTK::State& state) const override;

private:
    SimTK::ReferencePtr<const Coordinate> _xCoordinate;
    SimTK::ReferencePtr<const Coordinate> _yCoordinate;
    SimTK::ReferencePtr<const Coordinate> _zCoordinate;

//=============================================================================
};  // END of class MovingPathPoint
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MOVING_PATH_POINT_H_


