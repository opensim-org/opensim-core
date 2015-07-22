#ifndef OPENSIM_FRAME_SET_H_
#define OPENSIM_FRAME_SET_H_
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  FrameSet.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                      *
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

#include <OpenSim/Simulation/Model/Frame.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of Frames.
 *
 * @authors Ayman Habib
 * @version 1.0
 */

class OSIMSIMULATION_API FrameSet : public ModelComponentSet<Frame> {
OpenSim_DECLARE_CONCRETE_OBJECT(FrameSet, ModelComponentSet<Frame>);

public:
    FrameSet();
    FrameSet(Model& model);
    ~FrameSet();

    // Somehow the following function is not exported from base template
    FrameSet(Model& model, const std::string &aFileName,
             bool aUpdateFromXMLNode = true)
    :   Super(model, aFileName, aUpdateFromXMLNode) {}

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------


//=============================================================================
};  // END of class FrameSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_FRAME_SET_H_
