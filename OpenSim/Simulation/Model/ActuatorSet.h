#ifndef __AbsActuatorSet_h__
#define __AbsActuatorSet_h__

// ActuatorSet.h
// Author: Frank C. Anderson, Peter Loan
/*
 * Copyright (c)  2006, Stanford University. All rights reserved. 
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


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "AbstractActuator.h"

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of actuators for a model.
 * This class is based on ActuatorSet, written by Frank C. Anderson for
 * Realistic Dynamics, Inc.
 *
 * @authors Frank C. Anderson, Peter Loan
 * @version 1.0
 */

//=============================================================================
//=============================================================================
/**
 * A class for holding and managing a set of actuators for a model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API ActuatorSet : public Set<AbstractActuator>
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

	bool _computeActuationEnabled;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	ActuatorSet();
	ActuatorSet(const std::string &aFileName, bool aUpdateFromXMLNode = true);
	ActuatorSet(const ActuatorSet &aActuatorSet);
	virtual ~ActuatorSet();
	virtual Object* copy() const;
	void copyData(const ActuatorSet &aAbsActuatorSet);
private:
	void setNull();
	void setupSerializedMembers();
	void copyActuator(AbstractActuator* aFrom, AbstractActuator* aTo);

	void constructMap(const Array<int> &numValuesPerActuator, Array<int> &rActuatorToValue, Array<int> &rValueToActuator);
	void constructMaps();

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
	void setup(Model* aModel);
	void updateDisplayers();

	// MODEL
	void setModel(Model *aModel);
	Model* getModel() const;

	// ACTUATOR
	bool remove(int aIndex);
	bool append(AbstractActuator *aActuator);
	bool append(ActuatorSet &aActuatorSet, bool aAllowDuplicateNames=false);
	bool set(int aIndex, AbstractActuator *aActuator);
	AbstractActuator* changeActuatorType(AbstractActuator* aActuator, const std::string& aNewTypeName);
	void replaceActuator(AbstractActuator* aOldActuator, AbstractActuator* aNewActuator);

	// CONTROLS
	int getNumControls() const;
	int mapActuatorToControl(int aActuatorIndex) const;
	int mapControlToActuator(int aControlIndex) const;
	AbstractActuator *mapControlToActuator(int aControlIndex, int &rLocalIndex) const;

	int getControlIndex(const std::string &aName) const;
	std::string getControlName(int aIndex) const;

	void setControl(int aIndex,double aValue);
	void setControl(const std::string &aName,double aValue);
	void setControls(const double aX[]);

	double getControl(int aIndex) const;
	double getControl(const std::string &aName) const;
	void getControls(double rX[]) const;

	// STATES
	int getNumStates() const;
	int mapActuatorToState(int aActuatorIndex) const;
	int mapStateToActuator(int aStateIndex) const;
	AbstractActuator *mapStateToActuator(int aStateIndex, int &rLocalIndex) const;

	int getStateIndex(const std::string &aName) const;
	std::string getStateName(int aIndex) const;
	void getStateNames(Array<std::string> &rNames) const;

	void setState(int aIndex,double aValue);
	void setState(const std::string &aName,double aValue);
	void setStates(const double aY[]);

	double getState(int aIndex) const;
	double getState(const std::string &aName) const;
	void getStates(double rY[]) const;

	// PSEUDOSTATES
	int getNumPseudoStates() const;
	int mapActuatorToPseudoState(int aActuatorIndex) const;
	int mapPseudoStateToActuator(int aPseudoStateIndex) const;
	AbstractActuator *mapPseudoStateToActuator(int aPseudoStateIndex, int &rLocalIndex) const;

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
	void setComputeActuationEnabled(bool aTrueFalse);
	void promoteControlsToStates(const double aX[],double aDT);
	void computeActuation();
	void computeStateDerivatives(double rDY[]);
	void computeEquilibrium();
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
//=============================================================================
//=============================================================================

} // end of namespace OpenSim


#endif // __AbsActuatorSet_h__


