#ifndef OPENSIM_DISPLAYER_INTERFACE_H_
#define OPENSIM_DISPLAYER_INTERFACE_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  DisplayerInterface.h                 *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
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
#include "Simbody.h"
#include <OpenSim/Simulation/osimSimulationDLL.h>

namespace OpenSim {

class Body;
class Model;
class ModelDisplayHints;
class ModelComponent;
class PhysicalFrame;
//=============================================================================
//=============================================================================
/**
 * ModelComponents provide a mechanism to describe how they should be displayed
 * by implmeneting a method generateDecorations that creates a list of 
 * SimTK::DecorativeGeometry objects. The default implementation delegates the 
 * call to a specific implementation of this class DisplayerInterface to delay 
 * or modify how the list is generated.
 *
 * DisplayerInterface is abstract and have a few concrete implementations to 
 * handle custom visualizations (e.g. PathDisplayer). Users wanting to customize
 * their visualization can extend one of the concrete implementations or provide
 * a full implementation of their choice.
 *
 * @authors: Ayman Habib
 * @contributers:
 *
 * @version 1.0
 */
class OSIMSIMULATION_API DisplayerInterface {

protected:
    // Default constructor, does nothing
    DisplayerInterface() {};
public:
    // Main interface to be implemented by concrete implementations
    virtual void generateDecorations(const ModelComponent& mc,
        bool fixed,
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const = 0;
    // make a copy so that an object can make its own copy
    virtual DisplayerInterface* clone() const = 0;
    //Ddefault destructor 
    virtual ~DisplayerInterface() {};
//=============================================================================
};	// END of class DisplayerInterface
//=============================================================================

/**
* DefaultDisplayer provides a default implementation of DisplayerInterface to 
* describe how passed in ModelComponent should be displayed. It handles all 
* static geometry so that Geometry specified in model file under a ModelComponent
* can be communicated to the visualizaer without needing the ModelComponent to 
* implement the generateDecorations method.
*/
class OSIMSIMULATION_API DefaultDisplayer : public  DisplayerInterface {
public:
    DefaultDisplayer() {};
    virtual ~DefaultDisplayer() {};

    virtual DefaultDisplayer* clone() const {
        return new DefaultDisplayer(*this);
    }
    /**
    * Implementation of the DisplayerInterface.
    */
    void generateDecorations(const OpenSim::ModelComponent& mc,
        bool fixed,
        const OpenSim::ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const;
private:
    void generateDecorationsInFrame(const OpenSim::PhysicalFrame& frame,
        bool fixed,
        const OpenSim::ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const;

    void generateDecorationsArbitraryFrame(const OpenSim::ModelComponent& mc,
        bool fixed,
        const OpenSim::ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const;

    bool validateFile(const std::string& filename, const Model&) const;
    
};
/**
* DefaultDisplayer provides a default implementation of DisplayerInterface to
* describe how passed in ModelComponent should be displayed. It handles all
* static geometry so that Geometry specified in model file under a ModelComponent
* can be communicated to the visualizaer without needing the ModelComponent to
* implement the generateDecorations method.
*/
class OSIMSIMULATION_API PathDisplayer : public  DefaultDisplayer {
public:
    PathDisplayer() {};
    virtual ~PathDisplayer() {};

    virtual PathDisplayer* clone() const {
        return new PathDisplayer(*this);
    }
    void generateDecorations(const OpenSim::ModelComponent& gp,
        bool fixed,
        const OpenSim::ModelDisplayHints&           hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const;
};
//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_DISPLAYER_INTERFACE_H_


