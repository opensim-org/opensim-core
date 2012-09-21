#ifndef __WrapObjectSet_h__
#define __WrapObjectSet_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  WrapObjectSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include "WrapObject.h"

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of wrap objects.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API WrapObjectSet : public Set<WrapObject> {
OpenSim_DECLARE_CONCRETE_OBJECT(WrapObjectSet, Set<WrapObject>);

private:
	void setNull();
public:
	WrapObjectSet();
	WrapObjectSet(const WrapObjectSet& aWrapObjectSet);
	~WrapObjectSet(void);
	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	WrapObjectSet& operator=(const WrapObjectSet &aWrapObjectSet);
#endif
//=============================================================================
};	// END of class WrapObjectSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __WrapObjectSet_h__
