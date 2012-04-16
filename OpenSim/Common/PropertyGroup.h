#ifndef __PropertyGroup_h__
#define __PropertyGroup_h__

// PropertyGroup.h
// Author: Peter Loan
/*
 * Copyright (c)  2007, Stanford University. All rights reserved. 
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

#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

// INCLUDE
#include "osimCommonDLL.h"
#include "Property_Deprecated.h"
#include "Array.h"

namespace OpenSim {

#ifdef SWIG
	#ifdef OSIMCOMMON_API
		#undef OSIMCOMMON_API
		#define OSIMCOMMON_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A class implementing a property group.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMCOMMON_API PropertyGroup
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Name of the group. */
	std::string _name;

protected:
	/** Pointers to the properties in the group. */
	Array<Property_Deprecated*> _properties;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyGroup();
	PropertyGroup(std::string& aName);
	PropertyGroup(const PropertyGroup &aGroup);
	virtual ~PropertyGroup();
	virtual PropertyGroup* clone() const;

#ifndef SWIG
	PropertyGroup& operator=(const PropertyGroup &aGroup);
	bool operator<(const PropertyGroup &aGroup) const;
	bool operator==(const PropertyGroup& aGroup) const;
#endif
   void copyData(const PropertyGroup &aGroup);
	void clear();

	bool contains(const std::string& aName) const;
	void add(Property_Deprecated* aProperty);
	void remove(Property_Deprecated* aProperty);
	const Array<Property_Deprecated*>& getProperties() const { return _properties; }
	Property_Deprecated* get(int aIndex);
	int getPropertyIndex(Property_Deprecated* aProperty) const;

	// NAME
	void setName(const std::string &aName) { _name = aName; }
	const std::string& getName() const { return _name; }

private:
	void setNull();
//=============================================================================
};	// END of class PropertyGroup
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __PropertyGroup_h__


