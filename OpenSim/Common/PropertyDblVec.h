#ifndef _PropertyDblVec_h_
#define _PropertyDblVec_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  PropertyDblVec.h                         *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
 * Author(s): Ayman Habib, Ajay Seth, Michael A. Sherman                      *
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

#include "osimCommonDLL.h"
#include <string>
#include "Property_Deprecated.h"
#include "SimTKcommon.h"

//=============================================================================
//=============================================================================
/**
 * Class PropertyDblVec_ extends class Property.  It consists of a small
 * vector of doubles (i.e., SimTK::Vec<M>) and the methods for accessing
 * and modifying this Vec<M>.
 *
 * @author Ayman Habib, Ajay Seth, Michael Sherman
 */
namespace OpenSim { 

template<int M> class PropertyDblVec_ : public Property_Deprecated
{

//=============================================================================
// DATA
//=============================================================================
private:
	// Store in an Array for serialization; we'll fake up the Vec<M> when
    // we need it using the Array's storage.
	Array<double> _dblvec;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	/** Default constructor */
	PropertyDblVec_() : Property_Deprecated(DblVec, "DblVec_PropertyName")
		{ setAllowableListSize(M);
          _dblvec.setSize(M);
		}
	/** Construct from name and value */
	PropertyDblVec_(const std::string &aName, const SimTK::Vec<M>& aVec)
    :   Property_Deprecated(DblVec, aName) 
		{ setAllowableListSize(M);
          _dblvec.setSize(M);
          setValue(aVec);
		}
	/** Construct from name and value as an Array<double> */
	PropertyDblVec_(const std::string &aName, const Array<double> &anArray)
    :   Property_Deprecated(DblVec, aName)
		{ setAllowableListSize(M);
          _dblvec.setSize(M);    
          setValue(anArray);
		}
	
    // default destructor, copy constructor, copy assignment

	/* Return a copy of this property */
	PropertyDblVec_* clone() const OVERRIDE_11 {
		PropertyDblVec_* prop = new PropertyDblVec_<M>(*this);
		return prop;
	}

public:

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	/** Get the type of this property as a string. */
	std::string getTypeName() const OVERRIDE_11 {
		return "double";
	}

	// VALUE
	/** set value of property from an equivalently sized Vec */
	void setValue(const SimTK::Vec<M> &aVec) { 
		SimTK::Vec<M>::updAs(&_dblvec[0])=aVec; 
	}
	/** set value of this property from an array of doubles of equal or greater length */
	void setValue(const Array<double> &anArray) OVERRIDE_11 {
		assert(anArray.getSize() >= M);
		for(int i=0;i<M; i++)
			_dblvec[i] = anArray[i];
	}
	/** get writable reference to the value */
	SimTK::Vec<M>& getValueDblVec() {return SimTK::Vec<M>::updAs(&_dblvec[0]); }
	/** get const (read-only) reference to the value */
	const SimTK::Vec<M>& getValueDblVec() const {return SimTK::Vec<M>::getAs(&_dblvec[0]); };
	/** set value from double array */ // to be used by the serialization code
	void setValue(int aSize, const double aArray[]) OVERRIDE_11 { 
		assert(aSize == M);
		setValue(SimTK::Vec<M>::getAs(aArray));
	};
#ifndef SWIG
	/** get value as double array */
    const Array<double>& getValueDblArray() const OVERRIDE_11 {return _dblvec;}
#endif
	/** Nonconst version of accessor for use by GUI. */
    Array<double>& getValueDblArray() OVERRIDE_11 {return _dblvec;}

	/** Get a constant String represeting the value of this property. */
	std::string toString()  const OVERRIDE_11 {
		std::string str = "(";
		char dbl[256];
			for(int i=0; i < M; i++){
				sprintf(dbl, "%g", _dblvec[i]);
				str += (i>0?" ":"") + std::string(dbl);
			}
		str += ")";
		return str;
	};
	// SIZE
	int getArraySize() const OVERRIDE_11 { return M; }

    bool isArrayProperty() const OVERRIDE_11 {return true;}

    int getNumValues() const OVERRIDE_11 {return M;}
//=============================================================================
};	// END of class PropertyDblVec_

}; //OpenSim namespace
//=============================================================================
//=============================================================================

#endif //__PropertyDblVec__h__
