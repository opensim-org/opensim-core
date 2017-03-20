#ifndef OPENSIM_PATH_POINT_H_
#define OPENSIM_PATH_POINT_H_
/* -------------------------------------------------------------------------- *
 *                        OpenSim:  PathPoint.h                               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include "OpenSim/Simulation/Model/Station.h"
#include "OpenSim/Simulation/Model/AbstractPathPoint.h"

namespace OpenSim {

class PhysicalFrame;

//=============================================================================
//=============================================================================
/**
 * A path point that is stationary with respect to parent's PhysicalFrame
 */
class OSIMSIMULATION_API PathPoint : public AbstractPathPoint {
    OpenSim_DECLARE_CONCRETE_OBJECT(PathPoint, AbstractPathPoint);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
        "The fixed location of the path point expressed in its parent frame.");
//=============================================================================
// METHODS
//=============================================================================
    PathPoint();

    // A variant that uses basic types for use by GUI, e.g. OpenSimContext
    /** <b>(Deprecated)</b> Old PathPoint interface */
    DEPRECATED_14("Use setLocation() instead.")
    void setLocationCoord(const SimTK::State&s, int aXYZ, double aValue) {
        assert(aXYZ>=0 && aXYZ<=2);
        updStation().upd_location()[aXYZ]=aValue;
    }

    SimTK::Vec3 getLocation(const SimTK::State& s) const override {
        return getStation().get_location();
    }

    void setLocation(const SimTK::Vec3& location);

    void changeBodyPreserveLocation(const SimTK::State& s,
                                    const PhysicalFrame& body);

    void scale(const SimTK::Vec3& scaleFactors) override;

    SimTK::Vec3 getdPointdQ(const SimTK::State& s) const override {
        return SimTK::Vec3(0);
    }

protected:

    const Station& getStation() const { 
        return getMemberSubcomponent<Station>(stationIx);
    }
    Station& updStation() {
        return updMemberSubcomponent<Station>(stationIx);
    }

    // Component Interface
    void extendFinalizeFromProperties() override;
    void extendConnectToModel(Model& model) override;

private: 
    // Satisfy Point interface
    /* Calculate the location of this PathPoint in Ground as a function of
       the state. */
    SimTK::Vec3
        calcLocationInGround(const SimTK::State& state) const override final {
        return getStation().getLocationInGround(state);
    }
    /* Calculate the velocity of this PathPoint with respect to and expressed
       in Ground as a function of the state. */
    SimTK::Vec3
        calcVelocityInGround(const SimTK::State& state) const override final {
        return getStation().getVelocityInGround(state);
    }
    /* Calculate the acceleration of this PathPoint with respect to and
       expressed in ground as a function of the state. */
    SimTK::Vec3
        calcAccelerationInGround(const SimTK::State& state) const override final {
        return getStation().getAccelerationInGround(state);
    }

    void constructProperties();
private:

    MemberSubcomponentIndex stationIx{ constructSubcomponent<Station>("station") };

//=============================================================================
};  // END of class PathPoint
//=============================================================================
//=============================================================================


} // end of namespace OpenSim

#endif // OPENSIM_PATH_POINT_H_
