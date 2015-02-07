#ifndef OPENSIM_GROUND_H_
#define OPENSIM_GROUND_H_
/* --------------------------------------------------------------------------*
*                            OpenSim:  Ground.h                              *
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
#include <OpenSim/Simulation/Model/PhysicalFrame.h>
namespace OpenSim {

//=============================================================================
//=============================================================================
/**
* Ground is a PhysicalFrame in which all Frames (transforms) and Point 
* (locations) can be expressed. As a PhysicalFrame it supports physical 
* connections (e.g. Joints, Constraints) and forces can be applied to it.
*
* @author Ajay Seth
*/

class OSIMSIMULATION_API Ground : public PhysicalFrame {
    OpenSim_DECLARE_CONCRETE_OBJECT(Ground, PhysicalFrame);
public:
    /** Default Constructor */
    Ground();
    virtual ~Ground() {}

protected:
    /** The transform X_GF for this Ground, F, in ground, G. */
    SimTK::Transform
        calcGroundTransform(const SimTK::State& state) const override final;

    /** Extending the Component interface. */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override;

};  // END of class Ground

//=============================================================================
} // end of namespace OpenSim

#endif // OPENSIM_GROUND_H_


