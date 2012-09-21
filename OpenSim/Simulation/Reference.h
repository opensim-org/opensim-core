#ifndef __Reference_h__
#define __Reference_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  Reference.h                            *
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
#include "OpenSim/Common/Object.h"
#include "SimTKcommon.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * This base (abstract) class defines the interface for objects repsonsible in 
 * identifying a model output and its Reference value to be achieved
 * via optimization and/or tracking. Also contains a weighting that identifies 
 * the relative importance of achieving one Reference relative to others. The 
 * specific value type to be defined by the concrete References.
 *
 * @author Ajay Seth
 * @version 1.0
 */
template<class T> class Reference_ : public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT_T(Reference_, T, Object);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
	// DO NOT CACHE THE Reference VALUE AS A MEMBER VARIABLE
	// In general the Reference is a function of the state, therefore it must be 
	// evaluated.
	//
	// Concrete References can use Functions or Measures to supply values.

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
	virtual ~Reference_() {}
	
	Reference_() {}
	Reference_(std::string name) { setName(name); }

	Reference_& operator=(const Reference_& source) {
        if (&source != this)
            Super::operator=(source); 
        return *this;
    }

	//--------------------------------------------------------------------------
	// Reference Interface
	//--------------------------------------------------------------------------
	/** get the number of referettes (individual signals) in this Reference. All
	    return arrays are gauranteed to be this length */
	virtual int getNumRefs() const = 0;
	/** get the time range for which the Reference is valid, which can and will be finite
	    if reference encapsulates experimental data. By defualt they are infinite */
	virtual SimTK::Vec2 getValidTimeRange() const { return SimTK::Vec2(-SimTK::Infinity, SimTK::Infinity); }
	/** get the name(s) of the reference or its referettes */
	virtual const SimTK::Array_<std::string>& getNames() const = 0;
	/** get the value of the Reference as a funcion of the state */
	virtual void getValues(const SimTK::State &s, SimTK::Array_<T> &values) const = 0;
	/** get the weighting (importance) of meeting this Reference */
	virtual void getWeights(const SimTK::State &s, SimTK::Array_<double>& weights) const = 0;


	//--------------------------------------------------------------------------
	// Convenience Interface
	//--------------------------------------------------------------------------
	/* getValues as above, but a copy is returned, which may be costly */
	virtual SimTK::Array_<T> getValues(const SimTK::State &s) const {
		SimTK::Array_<T> values(getNumRefs());
		getValues(s, values);
		return values;
	}
	/* getWeights as above, but a copy is returned, which may be costly */
	virtual SimTK::Array_<double> getWeights(const SimTK::State &s) const {
		SimTK::Array_<double> weights(getNumRefs());
		getWeights(s, weights);
		return weights;
	}
	
//=============================================================================
};	// END of class templatized Reference_<T>
//=============================================================================
}

#endif // __Reference_h__
