#ifndef _PropertySet_h_
#define _PropertySet_h_
// PropertySet.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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
 * Author: Frank C. Anderson, Ajay Seth 
 */
#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

// INCLUDES
#include "osimCommonDLL.h"
#include "ArrayPtrs.h"
#include "Property_Deprecated.h"
#include "PropertyGroup.h"
#include "PropertyDblVec.h"


#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

#ifndef SWIG
#ifdef WIN32
template class OSIMCOMMON_API OpenSim::ArrayPtrs<OpenSim::Property_Deprecated>;
#endif
#endif


namespace OpenSim { 

// convenient abbreviations
typedef PropertyDblVec_<2> PropertyDblVec2;
typedef PropertyDblVec_<3> PropertyDblVec3;
typedef PropertyDblVec_<4> PropertyDblVec4;
typedef PropertyDblVec_<5> PropertyDblVec5;
typedef PropertyDblVec_<6> PropertyDblVec6;

//=============================================================================
//=============================================================================
/**
 * A property set is simply a set of properties.  It provides methods for
 * adding, removing, and retrieving properties from itself.
 *
 * @version 1.0
 * @author Frank C. Anderson
 * @see Property
 */
class OSIMCOMMON_API PropertySet  
{

//=============================================================================
// DATA
//=============================================================================
public:
	/** Set of properties. */
	ArrayPtrs<Property_Deprecated> _array;

protected:
	/** Array of property groups. */
	ArrayPtrs<PropertyGroup> _propertyGroups;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertySet();
	PropertySet(const PropertySet &aSet);
	virtual ~PropertySet() { _array.setSize(0); };

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
#ifndef SWIG
	friend std::ostream& operator<<(std::ostream &aOut,
															const PropertySet &aSet) {
		aOut << "\nProperty Set:\n";
		for(int i=0;i<aSet.getSize();i++) aOut << *aSet.get(i) << "\n";
		return(aOut);
	}
#endif
	//--------------------------------------------------------------------------
	// ACCESS
	//--------------------------------------------------------------------------
public:
	// Empty?
	bool isEmpty() const;
	// Number of properties
	int getSize() const;
	// Get
	virtual Property_Deprecated* get(int i) throw (Exception);
#ifndef SWIG
	virtual const Property_Deprecated* get(int i) const;
#endif
	virtual Property_Deprecated* get(const std::string &aName) throw (Exception);
#ifndef SWIG
	virtual const Property_Deprecated* get(const std::string &aName) const;
#endif
	virtual const Property_Deprecated* contains(const std::string& aName) const;
#ifndef SWIG
	virtual Property_Deprecated* contains(const std::string& aName);
#endif
	// Append
	virtual void append(Property_Deprecated *aProperty);
	virtual void append(Property_Deprecated *aProperty, const std::string& aName);
	// Remove
	virtual void remove(const std::string &aName);
	// Clear
	virtual void clear();

   PropertyGroup* addGroup(std::string aGroupName);
   void addPropertyToGroup(std::string aGroupName, std::string aPropertyName);
   void addPropertyToGroup(PropertyGroup* aGroup, std::string aPropertyName);
	void addPropertyToGroup(PropertyGroup* aGroup, Property_Deprecated* aProperty);
	void addPropertyToGroup(std::string aGroupName, Property_Deprecated* aProperty);
	ArrayPtrs<PropertyGroup>& getGroups() { return _propertyGroups; }
	PropertyGroup* getGroupContaining(Property_Deprecated* aProperty);
	int getGroupIndexContaining(Property_Deprecated* aProperty);

//=============================================================================
};	// END of class PropertySet

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertySet_h__
