#ifndef OPENSIM_APPEARANCE_H_
#define OPENSIM_APPEARANCE_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Appearance.h                             *
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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "SimTKcommon.h"
#include <OpenSim/Common/Property.h>

namespace OpenSim {

class Body;
class Model;


//=============================================================================
//=============================================================================
/**
 * A class that holds the Display Attributes (Appearance) of an object displayed in GUI
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMSIMULATION_API BaseAppearance : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(BaseAppearance, Object);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with Appearance. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(color, SimTK::Vec3,
        "The color (rgb) used to display the object. ");
    OpenSim_DECLARE_PROPERTY(opacity, double,
        "The opacity (0-1) used to display the object. ");
    OpenSim_DECLARE_PROPERTY(representation, int,
        "The representation (0:Hide, 1:Points, 2:Wire 2:Shaded) used to display the object. ");
    /**@}**/

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    BaseAppearance() {
        constructProperties();
    }
    virtual ~BaseAppearance() {};

private:
    void constructProperties() {
        constructProperty_color(SimTK::Vec3(1.0));
        constructProperty_opacity(1.0);
        constructProperty_representation(3);
    }
    //=============================================================================
};	// END of class BaseAppearance


class OSIMSIMULATION_API Appearance : public BaseAppearance {
    OpenSim_DECLARE_CONCRETE_OBJECT(Appearance, BaseAppearance);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with Appearance. **/
    /**@{**/
    OpenSim_DECLARE_OPTIONAL_PROPERTY(texture_file, std::string,
        "Name of file containing texture. ");
    /**@}**/

	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
    Appearance() {
    }
    virtual ~Appearance() {};

    bool hasTexture() {
        return getProperty_texture_file().size() > 0;
    }
//=============================================================================
};	// END of class Appearance
//=============================================================================

//=============================================================================
class OSIMSIMULATION_API LineAppearance : public BaseAppearance {
    OpenSim_DECLARE_CONCRETE_OBJECT(LineAppearance, Object);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with Appearance. **/
    /**@{**/
    OpenSim_DECLARE_PROPERTY(radius, double,
        "The radius of the shape used to display the object. ");
    OpenSim_DECLARE_PROPERTY(size, double,
        "The length of the displayed line object. ");
    /**@}**/

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    LineAppearance() {
        constructProperties();
    }
    virtual ~LineAppearance() {};

private:
    void constructProperties() {
        constructProperty_radius(.05);
        constructProperty_size(1.0);
    }
    //=============================================================================
};	// END of class LineAppearance
} // end of namespace OpenSim

#endif // OPENSIM_APPEARANCE_H_


