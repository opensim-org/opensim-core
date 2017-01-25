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
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    Marker();
    virtual ~Marker();

    const std::string& getFrameName() const;

    void setFrameName(const std::string& aName);
    void changeFrame(const OpenSim::PhysicalFrame& aPhysicalFrame );
    void changeFramePreserveLocation(const SimTK::State& s, OpenSim::PhysicalFrame& aPhysicalFrame );
    void scale(const SimTK::Vec3& aScaleFactors);

    /** Override of the default implementation to account for versioning. */
    void updateFromXMLNode(SimTK::Xml::Element& aNode,
        int versionNumber = -1) override;
    void generateDecorations(bool fixed, const ModelDisplayHints& hints, const SimTK::State& state,
        SimTK::Array_<SimTK::DecorativeGeometry>& appendToThis) const override;

private:
    void setNull();
    void constructProperties();
//=============================================================================
};  // END of class Marker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MARKER_H_


