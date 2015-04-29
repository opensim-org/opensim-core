#ifndef OPENSIM_MODEL_DISPLAY_H_
#define OPENSIM_MODEL_DISPLAY_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  ModelDisplay.h                             *
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
#include <OpenSim/Simulation/Model/ModelDisplayHints.h>

namespace OpenSim {

class Body;
class Model;


//=============================================================================
//=============================================================================
/**
 * A class that holds the Display Attributes (ModelDisplay) of a full Model 
 * displayed in Visualizer. Initially these are serializable ModelDisplayHints 
 * but in the future can be expanded to include search paths for Geometry, Texture
 * Renderer preferences, lights, cameras etc.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMSIMULATION_API ModelDisplay : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(ModelDisplay, Object);
public:
    //==============================================================================
    // PROPERTIES
    //==============================================================================
    /** @name Property declarations
    These are the serializable properties associated with ModelDisplay. **/
    /**@{**/
    OpenSim_DECLARE_UNNAMED_PROPERTY(ModelDisplayHints,
        "Model display preferences");

    /**@}**/

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    ModelDisplay() {
        constructProperties();
    }
    virtual ~ModelDisplay() {};

private:
    void constructProperties() {
        constructProperty_ModelDisplayHints(ModelDisplayHints());
    }
//=============================================================================
};  // END of class ModelDisplay
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_APPEARANCE_H_


