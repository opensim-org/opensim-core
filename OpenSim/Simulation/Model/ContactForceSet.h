#ifndef _ContactForceSet_h_
#define _ContactForceSet_h_
// ContactForceSet.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c) 2005, Stanford University. All rights reserved. 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions
* are met: 
*  - Redistributions of source code must retain the above copyright 
*    notice, this list of conditions and the following disclaimer. 
*  - Redistributions in binary form must reproduce the above copyright 
*    notice, this list of conditions and the following disclaimer in the 
*    documentation and/or other materials provided with the distribution. 
*  - Neither the name of the Stanford University nor the names of its 
*    contributors may be used to endorse or promote products derived 
*    from this software without specific prior written permission. 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
* POSSIBILITY OF SUCH DAMAGE. 
*/

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

#include "ActuatorSet.h"


//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of contacts for a model.  A contact
 * is distinguished from a general actuator in that it has no states and
 * no controls.  However, a contact can have pseudostates (variables that
 * are not only a function of the states, but also of the time history
 * of the states).
 *
 * @author Frank C. Anderson
 * @version 1.0
 * @todo Implement a getNumStates() method.  Contact forces could have states, just
 * no controls.
 */
namespace OpenSim { 

class ContactForce;
class AbstractBody;

class OSIMSIMULATION_API ContactForceSet : public ActuatorSet 
{
//=============================================================================
// MEMBERS
//=============================================================================
protected:

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ContactForceSet();
	ContactForceSet(const char *aFileName);
	ContactForceSet(const ContactForceSet &aContactForceSet);

	virtual ~ContactForceSet();
	Object* copy() const;
	void copyData(const ContactForceSet &aContactForceSet);
private:
	void setNull();
	void setupSerializedMembers();

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// CONTACT
	bool append(ContactForce *aContact);
	bool set(int aIndex,ContactForce *aContact);

	// Methods moved down from Model, need to be implemented
	AbstractBody* getContactBodyA(int aID) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	AbstractBody* getContactBodyB(int aID) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void setContactPointA(int aID,const double aPoint[3])
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactPointA(int aID,double rPoint[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void setContactPointB(int aID,const double aPoint[3])
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactPointB(int aID,double rPoint[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactForce(int aID,double rF[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactNormalForce(int aID,double rFP[3],double rFV[3],double rF[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactTangentForce(int aID,double rFP[3],double rFV[3],double rF[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactStiffness(int aID,const double aDX[3],double rDF[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactViscosity(int aID,const double aDV[3],double rDF[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactFrictionCorrection(int aID,double aDFFric[3]) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	double getContactForce(int aID) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	double getContactSpeed(int aID) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	double getContactPower(int aID) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	void computeContact();
	void updatePseudoStates();

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	void apply();

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	bool check() const;

//=============================================================================
};	// END of class ContactForceSet

}; //namespace
//=============================================================================
//=============================================================================


#endif // __ContactForceSet_h__


