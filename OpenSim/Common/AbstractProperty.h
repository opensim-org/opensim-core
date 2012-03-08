#ifndef OPENSIM_ABSTRACT_PROPERTY_H_
#define OPENSIM_ABSTRACT_PROPERTY_H_
// AbstractProperty.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2012, Stanford University. All rights reserved. 
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include "Array.h"
#include "ArrayPtrs.h"
#include "SimTKsimbody.h"

#include <string>
#include <cmath>

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A property consists of a type, name, and a value or an array of values.
 *
 * AbstractProperty is an abstract base class that provides the functionality
 * common to all property types.
 *
 * @author Cassidy Kelly, Ajay Seth
 */

class OSIMCOMMON_API AbstractProperty
{
public:
	/** Enumeration of recognized types. */
	enum PropertyType
	{
		None=0, Bool, Int, Dbl, Str, Obj, ObjPtr,
		BoolArray, IntArray, DblArray, StrArray, ObjArray,
		DblVec, DblVec3,
		Transform // 3 BodyFixed X,Y,Z Rotations followed by 3 Translations
		//Station	   Point on a Body: String, Vec3 
	};

	AbstractProperty();
	AbstractProperty(const std::string &aName, const std::string &aType, const std::string &aComment);
	AbstractProperty(const AbstractProperty &aAbstractProperty);
	AbstractProperty& operator=(const AbstractProperty &aAbstractProperty);

    // This is the interface that any concrete Property class must implement.
	virtual ~AbstractProperty() {}
	virtual AbstractProperty* copy() const = 0;
	virtual bool equals(AbstractProperty* aAbstractPropertyPtr) const = 0;
	virtual PropertyType getPropertyType() const = 0;


	std::string getName() const { return _name; }
	void setName(std::string aName){ _name = aName; }
	std::string getType() const { return _type; }
	void setType(std::string aType){ _type = aType; }
	std::string getComment() const { return _comment; }
	void setComment(std::string aComment){ _comment = aComment; }
	bool getUseDefault() const { return _useDefault; }
	void setUseDefault(bool aTrueFalse) { _useDefault = aTrueFalse; }
	bool getMatchName() const { return _matchName; }
	void setMatchName(bool aMatchName) { _matchName = aMatchName; }
	int getIndex() { return _index; }
	void setIndex(int aIndex) { _index = aIndex; }
	void setAllowableArraySize(int aMin, int aMax) { _minArraySize = aMin; _maxArraySize = aMax; }
	void setAllowableArraySize(int aNum) { _minArraySize = _maxArraySize = aNum; }
	int getMinArraySize() { return _minArraySize; }
	int getMaxArraySize() { return _maxArraySize; }

private:
	void setNull();

	std::string _name;
	std::string _type;
	std::string _comment;
	bool _useDefault;
	bool _matchName;
	int _index;
	int _minArraySize; // minimum number of elements for a property of array type
	int _maxArraySize; // maximum number of elements for a property of array type
};


}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ABSTRACT_PROPERTY_H_
