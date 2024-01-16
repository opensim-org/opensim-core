
#ifndef OPENSIM_TASK_SPACE_COMPUTE_CONTROLS_EVENT_HANDLER_H_
#define OPENSIM_TASK_SPACE_COMPUTE_CONTROLS_EVENT_HANDLER_H_
/* -------------------------------------------------------------------------- *
 *            OpenSim: TaskSpaceComputeControlsEventHandler.h                 *
 * -------------------------------------------------------------------------- *
 * Developed by CFD Research Corporation for a project sponsored by the US    *
 * Army Medical Research and Materiel Command under Contract No.              *
 * W81XWH-22-C-0020. Any views, opinions and/or findings expressed in this    *
 * material are those of the author(s) and should not be construed as an      *
 * official Department of the Army position, policy or decision unless so     *
 * designated by other documentation.                                         *
 *                                                                            *
 * Please refer to the following publication for mathematical details, and    *
 * cite this paper if you use this code in your own research:                 *
 *                                                                            *
 * Pickle and Sundararajan. "Predictive simulation of human movement in       *
 * OpenSim using floating-base task space control".                           *
 *                                                                            *
 * Copyright (c) 2023 CFD Research Corporation and the Authors                *
 *                                                                            *
 * Author(s): Aravind Sundararajan, Nathan Pickle, Garrett Tuer               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0          *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include "TaskSpaceTorqueController.h"

#include "SimTKcommon.h"

namespace OpenSim {

class ComputeControlsEventHandler : public SimTK::PeriodicEventHandler 
    {
        public:
            ComputeControlsEventHandler( TaskSpaceTorqueController *controller) :
                SimTK::PeriodicEventHandler(SimTK::Stage::Time),
                _controller( controller ) {
            }

            void handleEvent (SimTK::State& s, SimTK::Real accuracy, bool& terminate) const override {
                terminate = false;
                _controller->computeControls( s, _controller->_tau);
                _controller->setTargetTime(s.getTime() + _controller->getTargetDT());
            }

            SimTK::Real getNextEventTime( const SimTK::State& s, bool includeCurrent) const override {
                if( _controller->getCheckTargetTime() ) {
                    return( _controller->getTargetTime() );
                } else {
                    return( std::numeric_limits<SimTK::Real>::infinity() );
                }
            }
        TaskSpaceTorqueController* _controller;
    };
}

#endif