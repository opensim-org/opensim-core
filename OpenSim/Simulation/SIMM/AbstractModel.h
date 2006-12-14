#ifndef __AbstractModel_h__
#define __AbstractModel_h__

// AbstractModel.h
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
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
#include <string>
#include <OpenSim/Simulation/rdSimulationDLL.h>
#include <OpenSim/Tools/Storage.h>
#include <OpenSim/Tools/XMLDocument.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/tools/PropertyObj.h>
#include <OpenSim/Tools/PropertyObjArray.h>
#include <OpenSim/Tools/PropertyDblArray.h>
#include <OpenSim/Tools/ScaleSet.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
#include <OpenSim/Simulation/Model/ContactForceSet.h>
#include "AbstractDynamicsEngine.h"
#include "ActuatorSet.h"
#include "SimmMuscleGroup.h"
#include "SimmUnits.h"


namespace OpenSim {

class AbstractBody;
class IntegCallback;
class IntegCallbackSet;
class DerivCallback;
class DerivCallbackSet;

#ifdef SWIG
	#ifdef RDSIMULATION_API
		#undef RDSIMULATION_API
		#define RDSIMULATION_API
	#endif
#endif

//=============================================================================
//=============================================================================
/**
 * A base class that specifies the interface for a musculoskeletal model.
 * This class is based on Model, written by Frank C. Anderson for Realistic
 * Dynamics, Inc., but all of the data and methods relating to kinematics
 * engines or dynamics engines have been removed.
 *
 * @authors Frank C. Anderson, Peter Loan
 * @version 1.0
 */

class RDSIMULATION_API AbstractModel  : public Object
{

//=============================================================================
// DATA
//=============================================================================
private:

	/** Name of file from which the model was constructed. */
	std::string _fileName;

	/** Units for all lengths, distances. */
	PropertyStr _lengthUnitsStrProp;
	std::string& _lengthUnitsStr;
	SimmUnits _lengthUnits;

	/** Units for all forces. */
	PropertyStr _forceUnitsStrProp;
	std::string& _forceUnitsStr;
	SimmUnits _forceUnits;

	/** Array containg the acceleration due to gravity. */
	PropertyDblArray _gravityProp;
	Array<double> &_gravity;

	/** Dynamics Engine. */
	PropertyObjArray _dynamicsEngineProp;
	ArrayPtrs<AbstractDynamicsEngine> &_dynamicsEngine;

	// SETS OF THINGS
	/** Actuators. */
	PropertyObj _actuatorSetProp;
	ActuatorSet& _actuatorSet;

	/** Contacts. */
	PropertyObj _contactSetProp;
	ContactForceSet& _contactSet;

	/** Muscle Groups. */
	Set<SimmMuscleGroup> _muscleGroups;

	/** Analyses. */
	AnalysisSet *_analysisSet;

	/** Integration callbacks. */
	IntegCallbackSet *_integCallbackSet;

	/** Derivative callbacks. */
	DerivCallbackSet *_derivCallbackSet;

	// WORK VARIABLES
	/** Time */
	double _time;

	/** Time normalization. */
	double _tNormConst;

	/** Initial states. */
	Array<double> _yi;

	/** Initial pseudo-states. */
	Array<double> _ypi;

	/** Were all model components properly defined? */
	bool _builtOK;

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION AND DESTRUCTION
	//--------------------------------------------------------------------------
public:
	AbstractModel();
	AbstractModel(const std::string &aFileName)
#ifdef SWIG
		throw(OpenSim::Exception)
#endif
	;
	AbstractModel(DOMElement *aElement);
	AbstractModel(const AbstractModel& aModel);
	virtual ~AbstractModel();
	virtual Object* copy() const;
	virtual Object* copy(DOMElement *aElement) const;
	void copyData(const AbstractModel &aModel);
	void setupProperties();
	//static void registerTypes();

	/**
	 * Dynamic casting across JNI is messy. This method does the upCasting on C++ side
	 */
	AbstractModel* clone()
	{
		return reinterpret_cast<AbstractModel*>(this->copy()); 
	}

	SimmMuscleGroup* enterGroup(const std::string& aName);
	virtual void setup();
	bool builtOK() { return _builtOK; }

protected:
#ifndef SWIG
	AbstractModel& operator=(const AbstractModel &AbstractModel);
#endif

private:
	void setNull();

public:

	//--------------------------------------------------------------------------
	// FILE NAME
	//--------------------------------------------------------------------------
	virtual const std::string& getInputFileName() const { return _fileName; }

	//--------------------------------------------------------------------------
	// UNITS
	//--------------------------------------------------------------------------
	virtual const SimmUnits& getLengthUnits() const { return _lengthUnits; }
	virtual const SimmUnits& getForceUnits() const { return _forceUnits; }

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	virtual void getGravity(double rGrav[3]) const;
	virtual bool setGravity(double aGrav[3]);

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------
	virtual int getNumControls() const;
	virtual int getNumStates() const;
	virtual int getNumPseudoStates() const;
	virtual int getNumBodies() const;
	virtual int getNumJoints() const;
	virtual int getNumCoordinates() const;
	virtual int getNumSpeeds() const;
	virtual int getNumActuators() const;
	virtual int getNumMuscleGroups() const {	return _muscleGroups.getSize(); }
	virtual int getNumContacts() const;
	virtual int getNumAnalyses() const;
	int getNumConfigurations() const { return getNumCoordinates() + getNumSpeeds(); }

	//--------------------------------------------------------------------------
	// DYNAMICS ENGINE
	//--------------------------------------------------------------------------
	virtual AbstractDynamicsEngine& getDynamicsEngine() const;
	virtual void setDynamicsEngine(AbstractDynamicsEngine &aEngine);

	//--------------------------------------------------------------------------
	// SET TIME, CONTROLS, AND STATES
	//--------------------------------------------------------------------------
	virtual void set(double aT,const double aX[],const double aY[]);

	//--------------------------------------------------------------------------
	// TIME
	//--------------------------------------------------------------------------
	virtual void setTime(double aTime);
	virtual double getTime() const;

	//--------------------------------------------------------------------------
	// TIME NORMALIZATION
	//--------------------------------------------------------------------------
	virtual void setTimeNormConstant(double aNormConst);
	virtual double getTimeNormConstant() const;

	//--------------------------------------------------------------------------
	// CONTROLS
	//--------------------------------------------------------------------------
	virtual void setControls(const double aX[]);
	virtual void setControl(int aIndex, double aValue);
	virtual void setControl(const std::string &aName, double aValue);
	virtual void getControls(double rX[]) const;
	virtual double getControl(int aIndex) const;
	virtual double getControl(const std::string &aName) const;
	virtual std::string getControlName(int aIndex) const;

	//--------------------------------------------------------------------------
	// STATES
	//--------------------------------------------------------------------------
	virtual void getStateNames(Array<std::string> &rStateNames) const;
	virtual void setStates(const double aY[]);
	virtual void getStates(double rY[]) const;
	//virtual void setState(const std::string &aName,double aY);
	//virtual double getState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// INITIAL STATES
	//--------------------------------------------------------------------------
	virtual void setInitialStates(const double aYI[]);
	virtual void getInitialStates(double rYI[]) const;
	//virtual void setInitialState(const std::string &aName,double aYI);
	//virtual double getInitialState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// PSEUDOSTATES
	//--------------------------------------------------------------------------
	virtual int getPseudoStateNames(Array<std::string> &rStateNames) const;
	virtual void setPseudoStates(const double aYP[]);
	virtual void getPseudoStates(double rYP[]) const;
	//virtual void setPseudoState(const std::string &aName,double aYP);
	//virtual double getPseudoState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// INITIAL PSEUDO STATES
	//--------------------------------------------------------------------------
	virtual void setInitialPseudoStates(const double aYPI[]);
	virtual void getInitialPseudoStates(double rYPI[]) const;
	//virtual void setInitialPseudoState(const std::string &aName,double aYPI);
	//virtual double getInitialPseudoState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// ACTUATORS
	//--------------------------------------------------------------------------
	ActuatorSet* getActuatorSet();
	const ActuatorSet* getActuatorSet() const;
	SimmMuscleGroup* enterMuscleGroup(const std::string& aName)
	{
		throw Exception("enterMuscleGroup not implemented.", __FILE__, __LINE__);
	}

	//--------------------------------------------------------------------------
	// CONTACT
	//--------------------------------------------------------------------------
	ContactForceSet* getContactSet();
	const ContactForceSet* getContactSet() const;

	//--------------------------------------------------------------------------
	// INTEGRATION CALLBACKS
	//--------------------------------------------------------------------------
	virtual IntegCallbackSet* getIntegCallbackSet();
	virtual const IntegCallbackSet* getIntegCallbackSet() const;
	virtual void addIntegCallback(IntegCallback *aCallback);

	//--------------------------------------------------------------------------
	// DERIVATIVE CALLBACKS
	//--------------------------------------------------------------------------
	virtual DerivCallbackSet *getDerivCallbackSet();
	virtual const DerivCallbackSet *getDerivCallbackSet() const;
	virtual void addDerivCallback(DerivCallback *aCallback);

	//--------------------------------------------------------------------------
	// ANALYSES
	//--------------------------------------------------------------------------
	virtual AnalysisSet* getAnalysisSet();
	virtual const AnalysisSet* getAnalysisSet() const;
	virtual void addAnalysis(Analysis *aAnalysis);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeDerivatives(double rDYDT[]);
	virtual void computeAuxiliaryDerivatives(double rDYDT[]);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------
	virtual bool scale(const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);

	//--------------------------------------------------------------------------
	// PRINT
	//--------------------------------------------------------------------------
	void printBasicInfo(std::ostream &aOStream) const;
	void printDetailedInfo(std::ostream &aOStream) const;

	//--------------------------------------------------------------------------
	// TEST
	//--------------------------------------------------------------------------
	virtual void peteTest() const;
	void kinTest();

//=============================================================================
};	// END of class AbstractModel
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractModel_h__

