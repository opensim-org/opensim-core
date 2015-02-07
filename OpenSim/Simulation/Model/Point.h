#ifndef OPENSIM_POINT_H_
#define OPENSIM_POINT_H_
/* -------------------------------------------------------------------------- *
 *                              OpenSim:  Point.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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

// INCLUDE
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/ModelComponent.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A Point is an OpenSim representation for a location in space. It consists of
 * Points are intended to locate phyical structures (such as origins of joints
 * and points of muscle attachments) as well as embodying the results of spatial
 * calculations. For example, if your system involves contact, you might define
 * a Point that describes the lcoation of the center-of-pressure as one contact
 * element rolls over another.
 *
 * A Point provides its location in the Ground frame as a function of the 
 * Model's (SimTK::MultibodySystem's) state.
 *
 * @author Ajay Seth
 */
class OSIMSIMULATION_API Point : public ModelComponent {
OpenSim_DECLARE_ABSTRACT_OBJECT(Point, ModelComponent);

public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    Point();

    virtual ~Point() {};

    /** @name Spatial Operations for Point
    Convenient spatial operatiosn with Points.*/
    /**@{**/

    /**
    Get the location of this Point relative to the Ground frame.

    @param state       The state applied to the model when determining the
                       transform.
    @return location   The location of the point expressed in the Ground frame
    */
    const SimTK::Vec3& getGroundLocation(const SimTK::State& state) const;

    // End of Point's Spatial Operations
    ///@}

protected:
    /** @name Point Extension methods.
    Concrete Point types must override these methods. */
    /**@{**/

    /** Calculate the location of this Point with respect to ground as
        a function of the state. */
    virtual SimTK::Vec3
        calcGroundLocation(const SimTK::State& state) const = 0;
    /**@}**/

    /** @name Component Extension methods.
        Point types override these Component methods. */
    /**@{**/
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;
    void extendRealizeTopology(SimTK::State& s) const override;
    /**@}**/

private:

    mutable SimTK::CacheEntryIndex groundLocationIndex;

//=============================================================================
};  // END of class Point
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_POINT_H_


