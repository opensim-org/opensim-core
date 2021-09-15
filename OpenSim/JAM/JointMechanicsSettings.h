
#ifndef OPENSIM_JOINT_MECHANICS_SETTINGS_H_
#define OPENSIM_JOINT_MECHANICS_SETTINGS_H_
/* -------------------------------------------------------------------------- *
 *                      OpenSim: JointMechanicsSettings.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Colin Smith                                                     *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied    *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

//============================================================================
// INCLUDE
//============================================================================
#include "osimJAMDLL.h"
#include <OpenSim/Common/Object.h>
#include <OpenSim/Simulation/Model/Frame.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * Object to define additional "Joints" to report the transforms between frames
 *
 * @author  Colin Smith
 */
class OSIMJAM_API JointMechanicsFrameTransform
        : public Object {
    OpenSim_DECLARE_CONCRETE_OBJECT(JointMechanicsFrameTransform, Object);

public:
    //=============================================================================
    // PROPERTIES
    //=============================================================================
    OpenSim_DECLARE_PROPERTY(parent_frame, std::string, "Path to parent frame.");

    OpenSim_DECLARE_PROPERTY(child_frame, std::string, "Path to child frame.");

    OpenSim_DECLARE_PROPERTY(rotation_type, std::string, "Report rotation angles "
        " in a 'space' or 'body' fixed . "
        "The default value is space.");

    OpenSim_DECLARE_LIST_PROPERTY_SIZE(rotation_sequence, std::string, 3, 
        "List of three axes ('x','y', or 'z') describing the rotation sequence for "
        "reporting the rotation angles. "
        "The default value is 'z x y'. "
        "");

    OpenSim_DECLARE_PROPERTY(output_coordinates, bool, 
        "Write joint coordinates"
        "(3 translations and 3 rotations) to the .h5 file."
        "The default value is true.")

    OpenSim_DECLARE_PROPERTY(output_transformation_matrix, bool,
        "Write 4 x 4 transformation matrix to the .h5 file."
        "The default value is false.")

    //=============================================================================
    // METHODS
    //=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION AND DESTRUCTION
    //--------------------------------------------------------------------------
    /** Default constructor. */
    JointMechanicsFrameTransform();

    JointMechanicsFrameTransform(
            const std::string& parent, const std::string& child);

private:
    // Connect properties to local pointers.  */
    void constructProperties();

    //=========================================================================
}; // END of class JointMechanicsFrameTransform

}; // namespace OpenSim
//=============================================================================
//=============================================================================

#endif // OPENSIM_JOINT_MECHANICS_SETTINGS_H_
