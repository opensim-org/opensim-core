#ifndef __IKTask_h__
#define __IKTask_h__
/* -------------------------------------------------------------------------- *
 *                             OpenSim:  IKTask.h                             *
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
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * @author Eran Guendelman
 * @version 1.0
 */

class OSIMTOOLS_API IKTask : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(IKTask, Object);

protected:
	// whether or not this task will be used
	PropertyBool _applyProp;
	bool &_apply;

	PropertyDbl _weightProp;
	double &_weight;

public:
	IKTask();
	IKTask(const IKTask &aIKTask);

#ifndef SWIG
	IKTask& operator=(const IKTask &aIKTask);
#endif

	bool getApply() const { return _apply; }
	void setApply(bool aApply) { _apply = aApply; }

	double getWeight() { return _weight; }
	void setWeight(double weight) { _weight = weight; }

private:
	void setupProperties();
//=============================================================================
};	// END of class IKTask
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __IKTask_h__
