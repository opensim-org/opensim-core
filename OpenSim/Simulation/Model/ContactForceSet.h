#ifndef _ContactForceSet_h_
#define _ContactForceSet_h_
// ContactForceSet.h
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

#include "ActuatorSet.h"
#include <string>


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
	ContactForceSet(const std::string &aFileName);
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
	bool append(ContactForceSet &aContactForceSet, bool aAllowDuplicateNames=false);
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
	void setContactPointA(int aID,const SimTK::Vec3& aPoint)
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactPointA(int aID,SimTK::Vec3& rPoint) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void setContactPointB(int aID,const SimTK::Vec3& aPoint)
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactPointB(int aID,SimTK::Vec3& rPoint) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactForce(int aID,SimTK::Vec3& rF) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactNormalForce(int aID,SimTK::Vec3& rFP,SimTK::Vec3& rFV,SimTK::Vec3& rF) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactTangentForce(int aID,SimTK::Vec3& rFP,SimTK::Vec3& rFV,SimTK::Vec3& rF) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactStiffness(int aID,const SimTK::Vec3& aDX,SimTK::Vec3& rDF) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactViscosity(int aID,const SimTK::Vec3& aDV,SimTK::Vec3& rDF) const
	{
		throw Exception("Not implemented yet", __FILE__, __LINE__);
	}
	void getContactFrictionCorrection(int aID,SimTK::Vec3& aDFFric) const
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


