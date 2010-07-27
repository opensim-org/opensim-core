#ifndef __Reference_h__
#define __Reference_h__
// Reference.h
// Ajay Seth
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2010, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

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
template<class T> class Reference_ : public Object
{
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
	virtual ~Reference_() {};
	
	Reference_() {setType("Reference_"); }
	Reference_(std::string name) { setType("Reference_"); setName(name); }

	Reference_& operator=(const Reference_ &aRef) {Object::operator=(aRef); return(*this); };

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
