#ifndef __ConstraintSet_h__
#define __ConstraintSet_h__
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  ConstraintSet.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/SimbodyEngine/Constraint.h>
#include <OpenSim/Simulation/Model/ModelComponentSet.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {


class Model;

//class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of constraints.
 *
 * @authors Ajay Seth
 * @version 1.0
 */

class OSIMSIMULATION_API ConstraintSet : public ModelComponentSet<Constraint> {
OpenSim_DECLARE_CONCRETE_OBJECT(ConstraintSet, ModelComponentSet<Constraint>);

private:
	void setNull();
public:
	ConstraintSet();
	ConstraintSet(Model& model);

    ConstraintSet(Model& model, const std::string &aFileName, 
                  bool aUpdateFromXMLNode=true)
	:   Super(model, aFileName, aUpdateFromXMLNode) {}
	ConstraintSet(const ConstraintSet& aAbsConstraintSet);
	~ConstraintSet(void);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	ConstraintSet& operator=(const ConstraintSet &aAbsConstraintSet);
#endif
	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------

	// SCALE
	void scale(const ScaleSet& aScaleSet);

//=============================================================================
};	// END of class ConstraintSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ConstraintSet_h__
