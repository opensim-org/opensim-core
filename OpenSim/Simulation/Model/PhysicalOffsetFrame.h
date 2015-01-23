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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Simulation/Model/OffsetFrame.h> 
#include <OpenSim/Simulation/Model/PhysicalFrame.h> 

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
* A PhysicalOffsetFrame is a PhysicalFrame whose trasform is specified as a 
* constant offset from another PhysicalFrame. Potential use cases for the 
* PhysicalOffsetFrames are to specify the location of a Joint or Constraint on
* a Body. For example, the location and orientation of the ankle joint frame
* specified in the shank (tibia) Body's reference frame.
* This class has the methods of both the OffsetFrame (template) and the
* PhysicalFrame class.
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
    /** By default, the frame is not connected to any parent frame,
     * and its transform is an identity transform.
     */
    PhysicalOffsetFrame();

    virtual ~PhysicalOffsetFrame() {};

    /**
    A convenience constructor that initializes the parent connection and
    offset property of this PhysicalOffsetFrame.

    @param[in] parent   The parent PhysicalOffsetFrame.
    @param[in] offset   The offset transform between this frame and its parent
    */
    PhysicalOffsetFrame(const PhysicalFrame& parent,
                        const SimTK::Transform& transform);

private:

    void setNull();

//=============================================================================
}; // END of class PhysicalOffsetFrame
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_PHYSICAL_OFFSET_FRAME_H_


