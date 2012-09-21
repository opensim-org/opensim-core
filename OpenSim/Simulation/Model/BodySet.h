#ifndef __BodySet_h__
#define __BodySet_h__
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  BodySet.h                             *
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
#include <OpenSim/Simulation/Model/ModelComponentSet.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class ScaleSet;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of bodies.
 *
 * @authors Peter Loan
 * @version 1.0
 */

class OSIMSIMULATION_API BodySet :	public ModelComponentSet<Body> {
OpenSim_DECLARE_CONCRETE_OBJECT(BodySet, ModelComponentSet<Body>);

private:
	void setNull();
public:
	BodySet();
	BodySet(Model& model);
	BodySet(const BodySet& aAbsBodySet);
	~BodySet(void);

	// Somehow the following function is not exported from base template
    BodySet(Model& model, const std::string &aFileName, 
            bool aUpdateFromXMLNode = true) 
    :   Super(model, aFileName, aUpdateFromXMLNode) {}

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	BodySet& operator=(const BodySet &aAbsBodySet);
#endif
	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void scale(const ScaleSet& aScaleSet, bool aScaleMass = false);

//=============================================================================
};	// END of class BodySet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __BodySet_h__
