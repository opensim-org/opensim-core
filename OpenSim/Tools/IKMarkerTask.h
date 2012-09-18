#ifndef __IKMarkerTask_h__
#define __IKMarkerTask_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  IKMarkerTask.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Eran Guendelman                                                 *
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

#include "osimToolsDLL.h"
#include "IKTask.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @authors Eran Guendelman
 * @version 1.0
 */

class OSIMTOOLS_API IKMarkerTask : public IKTask {
OpenSim_DECLARE_CONCRETE_OBJECT(IKMarkerTask, IKTask);

public:
	IKMarkerTask();
	IKMarkerTask(const IKMarkerTask &aIKMarkerTask);

#ifndef SWIG
	IKMarkerTask& operator=(const IKMarkerTask &aIKMarkerTask);
#endif

//=============================================================================
};	// END of class IKMarkerTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKMarkerTask_h__
