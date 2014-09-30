/* -------------------------------------------------------------------------- *
 *                             OpenSim:  Geometry.cpp                             *
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

//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Property.h>
#include <OpenSim/Common/Component.h>
#include "Frame.h"
#include "RigidFrame.h"
#include "Geometry.h"
#include "Model.h"
//=============================================================================
// STATICS
//=============================================================================
using namespace std;
using namespace OpenSim;

/**
 * Compute Transform of a Geometry w.r.t. passed in Frame
 * Both Frame(s) could be Bodies, state is assumed to be realized ro position
*/
SimTK::Transform OpenSim::Geometry::getTransform(const SimTK::State& state, const OpenSim::RigidFrame& frame) const {
    const OpenSim::Model& model = frame.getModel();
    const OpenSim::Frame& gFrame = model.getBodySet().contains(get_frame_name()) ? model.getBodySet().get(get_frame_name()) :
        model.getFrameSet().get(get_frame_name());
    if (model.getBodySet().contains(frame.getName()))
        return  gFrame.calcTransformToOtherFrame(state, model.getBodySet().get(frame.getName()));
    return gFrame.calcTransformToOtherFrame(state, frame);
}

