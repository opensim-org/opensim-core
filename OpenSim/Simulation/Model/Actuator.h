#ifndef _Actuator_h_
#define _Actuator_h_
// Actuator.h
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


#include "Model.h"
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyDbl.h>


//=============================================================================
//=============================================================================
/**
 * An abstract class for representing an actuator (e.g., a torque motor,
 * muscle, ...).
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

namespace OpenSim { 

class RDSIMULATION_API Actuator : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
	static const double LARGE;

protected:
	// PROPERTIES
	/** Cross-sectional area of the actuator. */
	PropertyDbl _propArea;
	/** Minimum force (or torque) that this actuator can apply. */
	PropertyDbl _propMinForce;
	/** Maximum force (or torque) that this actuator can apply. */
	PropertyDbl _propMaxForce;
	/** Optimal force. */
	PropertyDbl _propOptimalForce;

	// REFERENCES
	double &_area;
	double &_minForce;
	double &_maxForce;
	double &_optimalForce;

	/** Model which the actuator actuates. */
	Model *_model;

	/** Flag indicating whether the actuator applies a force or a torque. */
	bool _appliesForce;
	/** Force (or torque) magnitude that is applied to the model. */
	double _force;
	/** Speed of actuator (linear or angular). */
	double _speed;

private:
	/** Excitation (control 0). */
	double _x;


//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	Actuator(int aNX,int aNY,int aNYP);
	Actuator(int aNX,int aNY,int aNYP,DOMElement *aElement);
	Actuator(const Actuator &aActuator);
	virtual ~Actuator();
	virtual Object* copy() const = 0;
	virtual Object* copy(DOMElement *aElement) const = 0;
private:
	void setNull();
	void setupProperties();
	
	void constructControls();
	void constructStates();
	void constructPseudoStates();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Actuator& operator=(const Actuator &aActuator);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:
	void setControlName(int aIndex,const char *aName);
	void setStateName(int aIndex,const char *aName);
	void setPseudoStateName(int aIndex,const char *aName);
public:
	// MODEL
	void setModel(Model *aModel);
	Model* getModel() const;
	// CONTROLS
	virtual int getNX() const;
	virtual const std::string getControlName(int aIndex) const;
	virtual int getControlIndex(const std::string &aName) const;
	virtual void setControl(int aIndex,double aValue);
	virtual void setControl(const std::string &aName,double aValue);
	virtual void setControls(const double aX[]);
	virtual double getControl(int aIndex) const;
	virtual double getControl(const std::string &aName) const;
	virtual void getControls(double rX[]) const;
	// STATES
	virtual int getNY() const;
	virtual const std::string& getStateName(int aIndex) const;
	virtual int getStateIndex(const std::string &aName) const;
	virtual void setState(int aIndex,double aValue);
	virtual void setState(const std::string &aName,double aValue);
	virtual void setStates(const double aY[]);
	virtual double getState(int aIndex) const;
	virtual double getState(const std::string &aName) const;
	virtual void getStates(double rY[]) const;
	// PSEUDOSTATES
	virtual int getNYP() const;
	virtual const std::string& getPseudoStateName(int aIndex) const;
	virtual int getPseudoStateIndex(const std::string &aName) const;
	virtual void setPseudoState(int aIndex,double aValue);
	virtual void setPseudoState(const std::string &aName,double aValue);
	virtual void setPseudoStates(const double aY[]);
	virtual double getPseudoState(int aIndex) const;
	virtual double getPseudoState(const std::string &aName) const;
	virtual void getPseudoStates(double rY[]) const;
	// AREA
	void setArea(double aArea);
	double getArea() const;
	// FORCE
protected:
	void setAppliesForce(bool aTrueFalse);
public:
	bool getAppliesForce() const;
	void setForce(double aForce);
	double getForce() const;
	double getStress() const;
	// SPEED
	double getSpeed() const;
	// POWER
	double getPower() const;
	// MAX FORCE
	void setMaxForce(double aMax);
	double getMaxForce() const;
	// MIN FORCE
	void setMinForce(double aMin);
	double getMinForce() const;
	// OPTIMAL FORCE
	void setOptimalForce(double aOptimalForce);
	double getOptimalForce() const;


	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const double aX[],double aDT);
	virtual void computeActuation() = 0;
	virtual void computeStateDerivatives(double rDYDT[]);
	virtual void updatePseudoStates();

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void apply() = 0;

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const;

//=============================================================================
};	// END of class Actuator

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Actuator_h__


