#ifndef _rdSerializableObject2_h_
#define _rdSerializableObject2_h_
// rdSerializableObject2.h:
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Property_Deprecated.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/PropertyBool.h>

//extern template class OSIMCOMMON_API Array<double>;

//=============================================================================
//=============================================================================
/**
* An object for mainly for testing XML serialization.
*
* @author Frank C. Anderson
* @version 1.0
*/
namespace OpenSim { 

	class rdSerializableObject2 : public Object
	{

		//=============================================================================
		// MEMBER DATA
		//=============================================================================

		//=============================================================================
		// METHODS
		//=============================================================================
	public:
		rdSerializableObject2(){
			setNull();
			setupSerializedMembers();
		};
		rdSerializableObject2(const std::string &aFileName) :
		Object(aFileName,false)
		{
			setNull();
			setupSerializedMembers();
			updateFromXMLDocument();
		};
		rdSerializableObject2(const rdSerializableObject2 &aObject){
			setNull();
			setupSerializedMembers();
			*this = aObject;
		};
		virtual Object* copy() const{
			rdSerializableObject2 *object = new rdSerializableObject2(*this);
			return(object);
		};
	private:
		void setNull(){
			setType("rdSerializableObject2");
		};
		void setupSerializedMembers(){
			// Bool
			PropertyBool pBool("Test_Bool2",false);
			_propertySet.append(pBool.copy());

			// DblArray
			Array<double> dblArray(0.1);
			dblArray.setSize(3);
			PropertyDblArray pDblArray("Test_DblArray2",dblArray);
			_propertySet.append(pDblArray.copy());
		};

		//--------------------------------------------------------------------------
		// OPERATORS
		//--------------------------------------------------------------------------
	public:
		rdSerializableObject2& operator=(const rdSerializableObject2 &aObject){
			Object::operator=(aObject);
			_propertySet.get(0)->setValue(aObject.getPropertySet().get(0)->getValueBool());
			_propertySet.get(1)->setValue(aObject.getPropertySet().get(1)->getValueDblArray());
			return(*this);
		};

		//=============================================================================
	};

	class rdSerializableObject3 : public Object
	{

		//=============================================================================
		// MEMBER DATA
		//=============================================================================

		//=============================================================================
		// METHODS
		//=============================================================================
	public:
		rdSerializableObject3(){
			setNull();
			setupSerializedMembers();
		};
		rdSerializableObject3(const std::string &aFileName) :
		Object(aFileName,false)
		{
			setNull();
			setupSerializedMembers();
			updateFromXMLDocument();
		};
		rdSerializableObject3(const rdSerializableObject3 &aObject){
			setNull();
			setupSerializedMembers();
			*this = aObject;
		};
		virtual Object* copy() const{
			rdSerializableObject3 *object = new rdSerializableObject3(*this);
			return(object);
		};
	private:
		void setNull(){
			setType("rdSerializableObject3");
		};
		void setupSerializedMembers(){
			// Bool
			PropertyBool pBool("Test_Bool2",false);
			_propertySet.append(pBool.copy());

			// DblArray
			Array<double> dblArray(0.1);
			dblArray.setSize(3);
			PropertyDblArray pDblArray("Test_DblArray2",dblArray);
			_propertySet.append(pDblArray.copy());
		};

		//--------------------------------------------------------------------------
		// OPERATORS
		//--------------------------------------------------------------------------
	public:
		rdSerializableObject3& operator=(const rdSerializableObject3 &aObject){
			Object::operator=(aObject);
			_propertySet.get(0)->setValue(aObject.getPropertySet().get(0)->getValueBool());
			_propertySet.get(1)->setValue(aObject.getPropertySet().get(1)->getValueDblArray());
			return(*this);
		};

		//=============================================================================
	};

    template<> struct PropertyTypeName<rdSerializableObject3> 
    {   static const char* name() {return "rdSerializableObject3";} };
	template <> inline AbstractProperty::PropertyType 
    Property2<rdSerializableObject3>::getPropertyType() const { return Obj; }

}; //namespace

//=============================================================================
//=============================================================================

#endif // __rdSerializableObject2_h__
