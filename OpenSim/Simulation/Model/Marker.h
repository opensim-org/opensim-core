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
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
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
#include <iostream>
#include <math.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "Station.h"
#include "SimTKcommon.h"

namespace OpenSim {

class Body;
class Model;
class VisibleObject;


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

private:
    void setNull();
    void setupProperties();
//=============================================================================
};  // END of class Marker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MARKER_H_


