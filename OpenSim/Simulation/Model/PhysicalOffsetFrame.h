#ifndef OPENSIM_PHYSICAL_OFFSET_FRAME_H_
#define OPENSIM_PHYSICAL_OFFSET_FRAME_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  PhysicalOffsetFrame.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <OpenSim/Simulation/Model/OffsetFrame.h>
#include <OpenSim/Simulation/Model/PhysicalFrame.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
* A PhysicalOffsetFrame is a PhysicalFrame whose transform is specified as a 
* constant offset from another PhysicalFrame. PhysicalOffsetFrames can be used
* to specify the location of a Joint or Constraint on a Body or any other 
* PhysicalFrame. For example, the location and orientation of the knee joint 
* frame specified in the femur (thigh) and tibia (shank) Body reference frames.
* This class has the methods of both the OffsetFrame (template) and the
* PhysicalFrame class.
* 
* NOTE: PhysicalOffsetFrame is closed to extensions. Consider extending
* OffsetFrame and/or use the mixin with a derived class of PhysicalFrame.
*
* @author Ajay Seth
*/
class OSIMSIMULATION_API PhysicalOffsetFrame : public OffsetFrame<PhysicalFrame> {
    OpenSim_DECLARE_CONCRETE_OBJECT(PhysicalOffsetFrame, OffsetFrame<PhysicalFrame>);

//=============================================================================
// PUBLIC METHODS
//=============================================================================
public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /** Constructors are defined by the OffsetFrame base class */
    using OffsetFrame<PhysicalFrame>::OffsetFrame;

    ~PhysicalOffsetFrame() final {}

protected:
    /** Extend Component interface for adding the PhysicalOffsetFrame to the 
        underlying multibody system */
    void extendAddToSystem(SimTK::MultibodySystem& system) const override final;

private:


//=============================================================================
}; // END of class PhysicalOffsetFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PHYSICAL_OFFSET_FRAME_H_


