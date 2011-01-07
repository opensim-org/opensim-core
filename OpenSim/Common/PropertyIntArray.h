#ifndef _PropertyIntArray_h_
#define _PropertyIntArray_h_
// PropertyIntArray.h
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
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include "osimCommonDLL.h"
#include <string>
#include "Property.h"
#ifdef WIN32
#pragma warning( disable : 4251 )
#endif

//=============================================================================
//=============================================================================
/**
 * Class PropertyIntArray extends class Property.  It consists of an
 * array of integers (i.e., Array<int>) and the methods for accessing
 * and modifying this array.
 *
 * @version 1.0
 * @author Frank C. Anderson
 */
namespace OpenSim { 

class OSIMCOMMON_API PropertyIntArray : public Property
{

//=============================================================================
// DATA
//=============================================================================
private:
	/** Array of integers. */
	Array<int> _array;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	PropertyIntArray();
	PropertyIntArray(const std::string &aName,
		const Array<int> &aArray);
	PropertyIntArray(const std::string &aName,
		int aSize,const int aArray[]);
	PropertyIntArray(const PropertyIntArray &aProperty);
	virtual Property* copy() const;
	virtual ~PropertyIntArray() { };

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
	PropertyIntArray& operator=(const PropertyIntArray &aProperty);

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// TYPE
	virtual const char* getTypeAsString() const;
	// VALUE
	virtual void setValue(const Array<int> &aArray);
	virtual void setValue(int aSize,const int aArray[]);
	virtual Array<int>& getValueIntArray();
	virtual const Array<int>& getValueIntArray() const;
	// SIZE
	virtual int getArraySize() const { return _array.getSize(); }
	// VALUE as String
	virtual const std::string &toString();

//=============================================================================
};	// END of class PropertyIntArray

}; //namespace
//=============================================================================
//=============================================================================

#endif //__PropertyIntArray_h__
