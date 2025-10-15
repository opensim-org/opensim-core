#ifndef OPENSIM_STATION_H_
#define OPENSIM_STATION_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Station.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2025 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth                                          *
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

#include "OpenSim/Simulation/Model/Point.h"
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

/**
 * A Station is a Point fixed to and located on a PhysicalFrame, which can be
 * a Body, Ground or any PhysicalOffsetFrame. Station%s are analogous to
 * PhysicalOffsetFrame%s where joints, constraints and forces can be attached
 * and/or applied.
 *
 * @author Ayman Habib, Ajay Seth
 */
class OSIMSIMULATION_API Station : public Point {
OpenSim_DECLARE_CONCRETE_OBJECT(Station, Point);
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(location, SimTK::Vec3,
        "The fixed location of the station expressed in its parent frame.");

//==============================================================================
// SOCKETS
//==============================================================================
    OpenSim_DECLARE_SOCKET(parent_frame, PhysicalFrame,
        "The frame to which this station is fixed.");

//==============================================================================
// METHODS
//==============================================================================
public:
    /**
     * Default constructor.
     */
    Station();

    /** Convenience constructor.
     *
     * @param[in] frame     PhysicalFrame in which the Station is located.
     * @param[in] location  Vec3 location of the station in its PhysicalFrame.
     */
    Station(const PhysicalFrame& frame, const SimTK::Vec3& location);

    /**
     * Destructor.
     */
    virtual ~Station();

    //** @name Accessors */
    // @{

    /**
     * Get the parent PhysicalFrame in which the Station is defined.
     */
    const PhysicalFrame& getParentFrame() const;

    /**
     * Set the parent PhysicalFrame in which the Station is defined.
     */
    void setParentFrame(const OpenSim::PhysicalFrame& aFrame);

    /**
     * Find this Station's location in any Frame.
     */
    SimTK::Vec3 findLocationInFrame(const SimTK::State& s,
                                    const OpenSim::Frame& frame) const;

    // @}

    //** @name ModelComponent interface */
    // @{
    void extendScale(const SimTK::State& s, const ScaleSet& scaleSet) override;
    // @}

    //** @name Component interface */
    // @{
    void generateDecorations(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& state,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;
    // @}

protected:
    // STATION INTERFACE
    // Derived classes of Station may override this method to
    // customize `generateDecorations()`.
    virtual void generateDecorationsImpl(
            bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& state,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const;

private:
    // POINT INTERFACE
    // Calculate the Station's location with respect to and expressed in Ground.
    SimTK::Vec3 calcLocationInGround(const SimTK::State& state) const override;
    // Calculate the velocity of this Station with respect to and expressed in
    // Ground as a function of the state.
    SimTK::Vec3 calcVelocityInGround(const SimTK::State& state) const override;
    // Calculate the acceleration of this Station with respect to and
    // expressed in ground as a function of the state.
    SimTK::Vec3 calcAccelerationInGround(
            const SimTK::State& state) const override;

    // CONVENIENCE METHODS
    void setNull();
    void constructProperties();

}; // class Station

} // namespace OpenSim

#endif // OPENSIM_STATION_H_
