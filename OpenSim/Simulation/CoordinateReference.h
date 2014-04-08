#ifndef __CoordinateReference_h__
#define __CoordinateReference_h__
/* -------------------------------------------------------------------------- *
 *                      OpenSim:  CoordinateReference.h                       *
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
 * other coordinates.
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
    CoordinateReference();

	/** Create a CoordinateReference
	* @param name of the reference to be found in the model and
	* @param ReferenceFunction that specifies the value of the coordinate
	*        to be matched at a given time
	*/
	CoordinateReference(const std::string name, Function &ReferenceFunction);

	CoordinateReference(const CoordinateReference& source);
	CoordinateReference& operator=(const CoordinateReference& source);
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
        delete _coordinateValueFunction; // delete before reassigning the pointer
		_coordinateValueFunction = function.clone();
	}
private:
    void copyData(const CoordinateReference& source);

//=============================================================================
};	// END of class CoordinateReference
//=============================================================================
} // namespace

#endif // __CoordinateReference_h__
