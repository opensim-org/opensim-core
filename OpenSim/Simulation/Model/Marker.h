#ifndef OPENSIM_MARKER_H_
#define OPENSIM_MARKER_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Marker.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Peter Loan                                         *
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
#include "Station.h"

namespace OpenSim {

class Body;
class Model;


//=============================================================================
//=============================================================================
/**
 * A class implementing a Mocap marker.
 *
 * @author Ayman Habib, Peter Loan
 * @version 2.0
 */
class OSIMSIMULATION_API Marker : public Station {
    OpenSim_DECLARE_CONCRETE_OBJECT(Marker, Station);

class Body;
public:
//==============================================================================
// PROPERTIES
//==============================================================================
    OpenSim_DECLARE_PROPERTY(fixed, bool,
        "Flag (true or false) specifying whether the marker is fixed in its "
        "parent frame during the marker placement step of scaling.  If false, "
        "the marker is free to move within its parent Frame to match its "
        "experimental counterpart.");
//=============================================================================
// METHODS
//=============================================================================
public:
    /** Default constructor */
    Marker();

    /** Convenience constructor
    @param[in] name      Marker name string.
    @param[in] frame     PhysicalFrame in which the Marker is located.
    @param[in] location  Vec3 location of the station in its PhysicalFrame */
    Marker(const std::string& name, const PhysicalFrame& frame,
           const SimTK::Vec3& location);

    virtual ~Marker();

    /** Convenience method to get the 'parent_frame' Socket's connectee_name */
    const std::string& getParentFrameName() const;

    /** Convenience method to set the 'parent_frame' Socket's connectee_name.
        The the named parent frame is not connected and finalizeConnections()
        must be invoked to establish the connection. */
    void setParentFrameName(const std::string& parentFrameName);
    /** Change the parent PhysicalFrame that this marker is attached to. */
    void changeFrame(const PhysicalFrame& parentFrame);
    /**  Change the parent PhysicalFrame that this marker is attached to. In  
         addition, preserve the marker location in the inertial (Ground) frame
         by using the state to compute the location in the new parent frame and
         to set its location property. */
    void changeFramePreserveLocation(const SimTK::State& s, 
                                     const PhysicalFrame& newParentFrame );

    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
        int versionNumber = -1) override;

private:
    // STATION INTERFACE
    void generateDecorationsImpl(bool fixed, const ModelDisplayHints& hints,
            const SimTK::State& state,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) const override;

    void setNull();
    void constructProperties();
//=============================================================================
};  // END of class Marker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MARKER_H_


