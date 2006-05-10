#ifndef _ActuatorSet_h_
#define _ActuatorSet_h_
// ActuatorSet.h
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


// INCLUDES
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/Set.h>
#include "Model.h"
#include "Actuator.h"


#ifndef SWIG
template class RDSIMULATION_API OpenSim::Set<OpenSim::Actuator>;
//template class RDTOOLS_API Array<int>;
#endif


//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of actuators for a model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
namespace OpenSim { 

class RDSIMULATION_API ActuatorSet : public Set<Actuator>
{

//=============================================================================
// DATA
//=============================================================================
protected:
	/** Model that is actuated. */
	Model *_model;

	/** Control to actuator map. */
	Array<int> _controlToActuator;
	/** Controls index. */
	Array<int> _actuatorToControl;

	/** State to actuator map. */
	Array<int> _stateToActuator;
	/** Actuator to state. */
	Array<int> _actuatorToState;

	/** Pseudo-state to actuator map. */
	Array<int> _pseudoToActuator;
	/** Actuator to pseudo-state map. */
	Array<int> _actuatorToPseudo;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ActuatorSet();
	ActuatorSet(const char *aFileName);
	virtual ~ActuatorSet();
private:
	void setNull();
	void setupSerializedMembers();
	
	void constructControlMaps();
	void constructStateMaps();
	void constructPseudoStateMaps();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	ActuatorSet& operator=(const ActuatorSet &aSet);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
public:
	// MODEL
	void setModel(Model *aModel);
	Model* getModel() const;

	// ACTUATOR
	bool remove(int aIndex);
	bool append(Actuator *aActuator);
	bool set(int aIndex,Actuator *aActuator);

	// CONTROLS
	int getNX() const;
	int mapActuatorToControl(int aActuatorIndex) const;
	int mapControlToActuator(int aControlIndex) const;

	int getControlIndex(const std::string &aName) const;
	std::string getControlName(int aIndex) const;

	void setControl(int aIndex,double aValue);
	void setControl(const std::string &aName,double aValue);
	void setControls(const double aX[]);

	double getControl(int aIndex) const;
	double getControl(const std::string &aName) const;
	void getControls(double rX[]) const;

	// STATES
	int getNY() const;
	int mapActuatorToState(int aActuatorIndex) const;
	int mapStateToActuator(int aStateIndex) const;

	int getStateIndex(const std::string &aName) const;
	std::string getStateName(int aIndex) const;

	void setState(int aIndex,double aValue);
	void setState(const std::string &aName,double aValue);
	void setStates(const double aY[]);

	double getState(int aIndex) const;
	double getState(const std::string &aName) const;
	void getStates(double rY[]) const;

	// PSEUDOSTATES
	int getNYP() const;
	int mapActuatorToPseudoState(int aActuatorIndex) const;
	int mapPseudoStateToActuator(int aPseudoStateIndex) const;

	int getPseudoStateIndex(const std::string &aName) const;
	std::string getPseudoStateName(int aIndex) const;

	void setPseudoState(int aIndex,double aValue);
	void setPseudoState(const std::string &aName,double aValue);
	void setPseudoStates(const double aY[]);

	double getPseudoState(int aIndex) const;
	double getPseudoState(const std::string &aName) const;
	void getPseudoStates(double rY[]) const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	void promoteControlsToStates(const double aX[],double aDT);
	void computeActuation();
	void computeStateDerivatives(double rDY[]);
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
};	// END of class ActuatorSet

}; //namespace
//=============================================================================
//=============================================================================


#endif // __ActuatorSet_h__


