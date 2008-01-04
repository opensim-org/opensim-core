#ifndef __AbstractActuator_h__
#define __AbstractActuator_h__

// AbstractActuator.h
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

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/ScaleSet.h>
#include <OpenSim/Common/Function.h>

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
	virtual void copyPropertyValues(AbstractActuator& aActuator) { }
	static void deleteActuator(AbstractActuator* aActuator) { if (aActuator) delete aActuator; }

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
	virtual void updateDisplayer() { }
	virtual void replaceFunction(Function* aOldFunction, Function* aNewFunction);
	OPENSIM_DECLARE_DERIVED(AbstractActuator, Object);

protected:
	// FORCE
	void setAppliesForce(bool aTrueFalse) { _appliesForce = aTrueFalse; }
	// Update the geometry attached to the actuator. Use inertial frame.
	virtual void updateGeometry();

public:
	bool getAppliesForce() const { return _appliesForce; }
	virtual void setForce(double aForce) { _force = aForce; }
	virtual double getForce() const { return _force; }
	virtual double getSpeed() const { return _speed; }
	virtual double getPower() const { return _force*_speed; }
	virtual double getStress() const;

	//--------------------------------------------------------------------------
	// COMPUTATIONS
	//--------------------------------------------------------------------------
	virtual void promoteControlsToStates(const double aX[],double aDT) { }
	virtual void computeActuation() = 0;
	virtual void computeStateDerivatives(double rDYDT[]) { }
	virtual void computeEquilibrium() { }
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


