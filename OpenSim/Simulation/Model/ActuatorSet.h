#ifndef __AbsActuatorSet_h__
#define __AbsActuatorSet_h__

// ActuatorSet.h
// Author: Frank C. Anderson, Peter Loan
/*
 * Copyright (c) 2006, Stanford University. All rights reserved. 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including 
 * without limitation the rights to use, copy, modify, merge, publish, 
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included 
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */


// INCLUDES
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/Set.h>
#include "AbstractActuator.h"

namespace OpenSim {

class AbstractModel;

#ifndef SWIG
template class OSIMSIMULATION_API Set<AbstractActuator>;
#endif

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
	AbstractModel *_model;

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
	ActuatorSet(const std::string &aFileName);
	ActuatorSet(const ActuatorSet &aActuatorSet);
	virtual ~ActuatorSet();
	virtual Object* copy() const;
	void copyData(const ActuatorSet &aAbsActuatorSet);
private:
	void setNull();
	void setupSerializedMembers();
	
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
	void setup(AbstractModel* aModel);
	void updateGeometry();

	// MODEL
	void setModel(AbstractModel *aModel);
	AbstractModel* getModel() const;

	// ACTUATOR
	bool remove(int aIndex);
	bool append(AbstractActuator *aActuator);
	bool append(ActuatorSet &aActuatorSet, bool aAllowDuplicateNames=false);
	bool set(int aIndex, AbstractActuator *aActuator);

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


