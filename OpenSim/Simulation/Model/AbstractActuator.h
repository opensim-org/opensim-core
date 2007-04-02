#ifndef __AbstractActuator_h__
#define __AbstractActuator_h__

// AbstractActuator.h
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ScaleSet.h>

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
	#endif
#endif

namespace OpenSim {

class Model;
class VisibleObject;

//=============================================================================
//=============================================================================
/**
 * An abstract class for representing an actuator (e.g., a torque motor,
 * muscle, ...).
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractActuator : public Object
{

//=============================================================================
// DATA
//=============================================================================
public:
	static const double LARGE;
protected:
	/** Model which the actuator actuates. */
	Model *_model;

	/** Flag indicating whether the actuator applies a force or a torque. */
	bool _appliesForce;

	/** Force (or torque) magnitude that is applied to the model. */
	double _force;

	/** Speed of actuator (linear or angular). */
	double _speed;

	/** Controls -- typically point to local members of derived classes. */
	Array<double*> _x;
	/** States -- typically point to local members of derived classes. */
	Array<double*> _y;
	/** Pseudo states -- typically point to local members of derived classes. */
	Array<double*> _yp;

	/** Name suffixes. */
	Array<std::string> _controlSuffixes;
	Array<std::string> _stateSuffixes;
	Array<std::string> _pseudoStateSuffixes;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractActuator();
	AbstractActuator(const AbstractActuator &aActuator);
	virtual ~AbstractActuator();
	virtual Object* copy() const = 0;
private:
	void setNull();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	AbstractActuator& operator=(const AbstractActuator &aActuator);
#endif
	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
protected:
	void setNumControls(int aNumControls);
	void setNumStates(int aNumStates);
	void setNumPseudoStates(int aNumPseudoStates);
	void bindControl(int aIndex,double &x,const std::string &aSuffix);
	void bindState(int aIndex,double &y,const std::string &aSuffix);
	void bindPseudoState(int aIndex,double &yp,const std::string &aSuffix);
public:
	virtual void setup(Model *aModel);
	// MODEL
	void setModel(Model *aModel) { _model = aModel; }
	Model* getModel() const { return _model; }
	// CONTROLS
	virtual int getNumControls() const { return _x.getSize(); }
	virtual const std::string getControlName(int aIndex) const;
	virtual int getControlIndex(const std::string &aName) const;
	virtual void setControl(int aIndex,double aValue);
	virtual void setControls(const double aX[]);
	virtual double getControl(int aIndex) const;
	virtual void getControls(double rX[]) const;
	// STATES
	virtual int getNumStates() const { return _y.getSize(); }
	virtual const std::string getStateName(int aIndex) const;
	virtual int getStateIndex(const std::string &aName) const;
	virtual void setState(int aIndex,double aValue);
	virtual void setStates(const double aY[]);
	virtual double getState(int aIndex) const;
	virtual void getStates(double rY[]) const;
	// PSEUDOSTATES
	virtual int getNumPseudoStates() const { return _yp.getSize(); }
	virtual const std::string getPseudoStateName(int aIndex) const;
	virtual int getPseudoStateIndex(const std::string &aName) const;
	virtual void setPseudoState(int aIndex,double aValue);
	virtual void setPseudoStates(const double aY[]);
	virtual double getPseudoState(int aIndex) const;
	virtual void getPseudoStates(double rY[]) const;
	// Visible Object Support
	virtual VisibleObject* getDisplayer() const { return NULL; }
	// Update the geometry attached to the actuator. Use inertial frame.
	virtual void updateGeometry();
	OPENSIM_DECLARE_DERIVED(AbstractActuator, Object);

protected:
	// FORCE
	void setAppliesForce(bool aTrueFalse) { _appliesForce = aTrueFalse; }

public:
	bool getAppliesForce() const { return _appliesForce; }
	virtual void setForce(double aForce) { _force = aForce; }
	virtual double getForce() const { return _force; }
	virtual double getSpeed() const { return _speed; }
	virtual double getPower() const { return _force*_speed; }
	virtual double getStress() const;
	// GROUPS (for grouping sets of muscles)
	virtual const Array<std::string>* getGroupNames() const { return NULL; }

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const double aX[],double aDT) { }
	virtual void computeActuation() = 0;
	virtual void computeStateDerivatives(double rDYDT[]) { }
	virtual void updatePseudoStates() { }

	//--------------------------------------------------------------------------
	// APPLICATION
	//--------------------------------------------------------------------------
	virtual void apply() = 0;

	//--------------------------------------------------------------------------
	// CHECK
	//--------------------------------------------------------------------------
	virtual bool check() const { return true; }

	//--------------------------------------------------------------------------
	// SCALING
	//--------------------------------------------------------------------------
	virtual void preScale(const ScaleSet& aScaleSet) { }
	virtual void scale(const ScaleSet& aScaleSet) { }
	virtual void postScale(const ScaleSet& aScaleSet) { }

//=============================================================================
};	// END of class AbstractActuator
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractActuator_h__


