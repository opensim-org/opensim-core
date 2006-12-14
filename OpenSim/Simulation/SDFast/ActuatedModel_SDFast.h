#ifndef _ActuatedModel_SDFast_h_
#define _ActuatedModel_SDFast_h_
// ActuatedModel_SDFast.h
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <OpenSim/Tools/rdTools.h>
#include "rdSDFastDLL.h"
#include <OpenSim/Simulation/Simm/AbstractModel.h>
#include <OpenSim/Simulation/Simm/ActuatorSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include "rdSDFast.h"


//=============================================================================
//=============================================================================
/**
 * A class that supports actuation and contact functionality to SDFast
 * models.
 */
namespace OpenSim { 

class AbstractBody;

class RDSDFAST_API ActuatedModel_SDFast : public rdSDFast
{
//=============================================================================
// DATA
//=============================================================================
protected:

private:
	/** Array of initial states. */
	Array<double> _yi;
	/** Array of initial pseudo-states. */
	Array<double> _ypi;

protected:
	ActuatorSet _actuatorSet;
	ContactForceSet _contactSet;
	
//=============================================================================
// METHODS
//=============================================================================
public:
	ActuatedModel_SDFast(const ActuatorSet *aActuators,
		const ContactForceSet *aContacts);
	ActuatedModel_SDFast(const std::string &aFileName,
							ActuatorSet *aActuators,
							ContactForceSet *aContacts);
	virtual ~ActuatedModel_SDFast();
private:
	void setNull();
	virtual void init();
protected:
	/* Register types to be used when reading an ActuatedModel_SDFast object from xml file. */
	static void RegisterTypes();

public:
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	// NUMBERS OF THINGS
	virtual int getNumControls() const;
	virtual int getNumActuators() const;
	virtual int getNumContacts() const;
	virtual int getNumStates() const;
	virtual int getNumPseudoStates() const;

	// NAMES
	virtual std::string getActuatorName(int aIndex) const;
	virtual std::string getControlName(int aIndex) const;
	virtual std::string getStateName(int aIndex) const;
	virtual std::string getPseudoStateName(int aIndex) const;

	// INDICES
	virtual int getActuatorIndex(const std::string &aName) const;
	virtual int getControlIndex(const std::string &aName) const;
	virtual int getStateIndex(const std::string &aName) const;
	virtual int getPseudoStateIndex(const std::string &aName) const;

	// ACTUATOR AND CONTACT SETS
	ActuatorSet* getActuatorSet();
	ContactForceSet* getContactForceSet();

	// CONTROLS
	virtual void setControl(int aIndex,double aValue);
	virtual void setControl(const std::string &aName,double aValue);
	virtual void setControls(const double aX[]);
	virtual double getControl(const std::string &aName) const;
	virtual double getControl(int aIndex) const;
	virtual void getControls(double aX[]) const;

	// STATES
	virtual void setStates(const double aY[]);
	virtual void getStates(double rY[]) const;
	virtual double getState(int aIndex) const;
	virtual double getState(const std::string &aName) const;
	
	// PSEUDOSTATES
	virtual void setPseudoStates(const double aYP[]);
	virtual void getPseudoStates(double rYP[]) const;
	virtual double getPseudoState(int aIndex) const;
	virtual double getPseudoState(const std::string &aName) const;

	// INITIAL PSEUDO STATES
	virtual void setInitialPseudoStates(const double aYPI[]);
	virtual double getInitialPseudoState(int aIndex) const;
	virtual double getInitialPseudoState(const std::string &aName) const;
	virtual void getInitialPseudoStates(double rYPI[]) const;

	// INITIAL STATES
	virtual void setInitialStates(const double aYI[]);
	virtual double getInitialState(int aIndex) const;
	virtual double getInitialState(const std::string &aName) const;
	virtual void getInitialStates(double rYI[]) const;

	//--------------------------------------------------------------------------
	// ACTUATION
	//--------------------------------------------------------------------------
	virtual void computeActuation();
	virtual void computeActuatorStateDerivatives(double rDY[]);
	virtual void applyActuatorForce(int aID);
	virtual void applyActuatorForces();
	virtual void setActuatorForce(int aID,double aForce);
	virtual double getActuatorForce(int aID) const;
	virtual double getActuatorStress(int aID) const;
	virtual double getActuatorSpeed(int aID) const;
	virtual double getActuatorPower(int aID) const;
	//--------------------------------------------------------------------------
	// CONTACT
	//--------------------------------------------------------------------------
	virtual void
		computeContact();
	virtual void
		applyContactForce(int aID);
	virtual void
		applyContactForces();
	virtual AbstractBody*
		getContactBodyA(int aID) const;
	virtual AbstractBody*
		getContactBodyB(int aID) const;
	virtual void
		setContactPointA(int aID,const double aPoint[3]);
	virtual void
		getContactPointA(int aID,double rPoint[3]) const;
	virtual void
		setContactPointB(int aID,const double aPoint[3]);
	virtual void
		getContactPointB(int aID,double rPoint[3]) const;
	virtual void
		getContactForce(int aID,double rF[3])const;
	virtual void
		getContactNormalForce(int aID,double rFP[3],double rFV[3],double rF[3])
		const;
	virtual void
		getContactTangentForce(int aID,double rFP[3],double rFV[3],double rF[3])
		const;
	virtual void
		getContactStiffness(int aID,const double aDX[3],double rDF[3]) const;
	virtual void
		getContactViscosity(int aID,const double aDV[3],double rDF[3]) const;
	virtual void
		getContactFrictionCorrection(int aID,double rDFFric[3]) const;
	virtual double
		getContactForce(int aID) const;
	virtual double
		getContactSpeed(int aID) const;
	virtual double
		getContactPower(int aID) const;

	//--------------------------------------------------------------------------
	// OPTIMIZATION
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const double aX[],double aDT);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeAuxiliaryDerivatives(double *dydt);

	//--------------------------------------------------------------------------
	// CALLBACKS
	//--------------------------------------------------------------------------
	/* function has no implementation and is not virtual! void
		integStepCallback(double *xtPrev,double *yPrev,int i,double dt,
		 double t,double *x,double *y,void *cd);*/


//=============================================================================
};	// END of class ActuatedModel_SDFast

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __ActuatedModel_SDFast_h__
