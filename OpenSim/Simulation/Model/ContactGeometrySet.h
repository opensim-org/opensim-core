#ifndef __ContactGeometrySet_h__
#define __ContactGeometrySet_h__
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ContactGeometrySet.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Peter Eastman                                                   *
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
#include "OpenSim/Simulation/Model/ModelComponentSet.h"
#include "ContactGeometry.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A class for holding a set of ContactGeometry objects.
 *
 * @authors Peter Eastman
 * @version 1.0
 */

class OSIMSIMULATION_API ContactGeometrySet 
:   public ModelComponentSet<ContactGeometry> {
OpenSim_DECLARE_CONCRETE_OBJECT(ContactGeometrySet, 
                                ModelComponentSet<ContactGeometry>);

private:
	void setNull();
public:
	ContactGeometrySet();
	ContactGeometrySet(Model& model);
	ContactGeometrySet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode);
	ContactGeometrySet(const ContactGeometrySet& aContactGeometrySet);
	~ContactGeometrySet(void);

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	ContactGeometrySet& operator=(const ContactGeometrySet &aContactGeometrySet);
#endif
	//--------------------------------------------------------------------------
	// UTILITIES
	//--------------------------------------------------------------------------
	void scale(const ScaleSet& aScaleSet);
//=============================================================================
};	// END of class ContactGeometrySet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ContactGeometrySet_h__
