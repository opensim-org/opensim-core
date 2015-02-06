#ifndef OPENSIM_APPEARANCE_MAP_H_
#define OPENSIM_APPEARANCE_MAP_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  AppearanceMap.h                             *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Object.h>
#include "Appearance.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class that holds the map between object PathName and Appearance
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMSIMULATION_API GeometryAppearance : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(GeometryAppearance, Object);
public:
    OpenSim_DECLARE_PROPERTY(geometryID, std::string,
        "PathName for the geometry.");
    OpenSim_DECLARE_UNNAMED_PROPERTY(Appearance, 
        "Desired Appearance for the specific geometry.");
public:
    GeometryAppearance() {
        constructProperties();
    };
    virtual ~GeometryAppearance() {};

private:
    void constructProperties(){
        constructProperty_geometryID("");
        constructProperty_Appearance(Appearance());
    }
};

class OSIMSIMULATION_API AppearanceMap : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(AppearanceMap, Object);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with AppearanceMap. **/
    /**@{**/
    OpenSim_DECLARE_LIST_PROPERTY(AppearanceList, GeometryAppearance,
        "Map between Geometry objects and user specified display preferences");
    /**@}**/

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    AppearanceMap(){
        constructProperties();
    };
    virtual ~AppearanceMap() {};

private:
    void constructProperties() {
        constructProperty_AppearanceList();
    }
//=============================================================================
};  // END of class Appearance
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_APPEARANCE_MAP_H_


