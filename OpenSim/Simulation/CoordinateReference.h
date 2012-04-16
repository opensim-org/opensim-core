#ifndef __CoordinateReference_h__
#define __CoordinateReference_h__
// CoordinateReference.h
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

#include "Reference.h"
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/Function.h>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * Reference value to be achieved for a specified coordinate that will be used
 * via optimization and/or tracking. Also contains a weighting that identifies
 * the relative importance of achieving one CoordinateReference relative to
 * others coordinates.
 *
 * @author Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API CoordinateReference : public Reference_<double> {
OpenSim_DECLARE_CONCRETE_OBJECT(CoordinateReference, Reference_<double>);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================

protected:

	/** Specify the Reference coordinate value as a function of time. */
	PropertyObjPtr<Function> _coordinateValueFunctionProp;
	Function *&_coordinateValueFunction;

	/** Specify the default weight for this coordinate reference.  */
	PropertyDbl _defaultWeightProp;
	double &_defaultWeight;

	SimTK::Array_<std::string> _names;

//=============================================================================
// METHODS
//=============================================================================
public:
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------

	/** Create a CoordinateReference
	* @param name of the reference to be found in the model and
	* @param referenceFunction that specifies the value of the coordinate
	*        to be matched at a given time
	*/
	CoordinateReference();
	CoordinateReference(const std::string name, Function &ReferenceFunction);
	CoordinateReference& operator=(const CoordinateReference &aRef);
	virtual ~CoordinateReference() {}

	//--------------------------------------------------------------------------
	// Reference Interface
	//--------------------------------------------------------------------------
	/** get the number of referettes (individual signals) in this Reference. All
	    return arrays are gauranteed to be this length */
	virtual int getNumRefs() const {return 1;} ;
	/** get the name(s) of the reference or its referettes */
	virtual const SimTK::Array_<std::string>& getNames() const;
	/** get the value of the Reference as a funcion of the state */
	virtual void getValues(const SimTK::State &s, SimTK::Array_<double> &values) const;
	/** get the weighting (importance) of meeting this Reference */
	virtual void getWeights(const SimTK::State &s, SimTK::Array_<double>& weights) const;


	//--------------------------------------------------------------------------
	// Convenience double interface
	//--------------------------------------------------------------------------
	/** get the value of the CoordinateReference */
	virtual double getValue(const SimTK::State &s) const;
	/** get the speed value of the CoordinateReference */
	virtual double getSpeedValue(const SimTK::State &s) const;
	/** get the speed value of the CoordinateReference */
	virtual double getAccelerationValue(const SimTK::State &s) const;
	/** get the weighting (importance) of meeting this CoordinateReference */
	virtual double getWeight(const SimTK::State &s) const;
	/** set the weighting (importance) of meeting this CoordinateReference */
	void setWeight(double weight);

	/** Set the coordinate value as a function of time. */
	void setValueFunction(const OpenSim::Function& function)
	{
		_coordinateValueFunction = function.clone();
	}

//=============================================================================
};	// END of class CoordinateReference
//=============================================================================
} // namespace

#endif // __CoordinateReference_h__
