#ifndef __AbstractModel_h__
#define __AbstractModel_h__

// Model.h
// Authors: Frank C. Anderson, Peter Loan, Ayman Habib
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
#include <string>
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/PropertyObjPtr.h>
#include <OpenSim/Common/PropertyDblArray.h>
#include <OpenSim/Common/ScaleSet.h>
#include "Analysis.h"
#include "AnalysisSet.h"
#include "ContactForceSet.h"
#include "AbstractDynamicsEngine.h"
#include "ActuatorSet.h"
#include <OpenSim/Common/Units.h>


namespace OpenSim {

class AbstractBody;
class IntegCallback;
class IntegCallbackSet;
class DerivCallback;
class DerivCallbackSet;

#ifdef SWIG
	#ifdef OSIMSIMULATION_API
		#undef OSIMSIMULATION_API
		#define OSIMSIMULATION_API
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

class OSIMSIMULATION_API Model  : public Object
{

//=============================================================================
// DATA
//=============================================================================
private:

	/** Name of file from which the model was constructed. */
	std::string _fileName;

	/** Model credits Info. */
	PropertyStr _creditsStrProp;
	std::string& _creditsStr;

	/** Publications and References. */
	PropertyStr _publicationsStrProp;
	std::string& _publicationsStr;

	/** Units for all length. */
	PropertyStr _lengthUnitsStrProp;
	std::string& _lengthUnitsStr;
	Units _lengthUnits;

	/** Units for all forces. */
	PropertyStr _forceUnitsStrProp;
	std::string& _forceUnitsStr;
	Units _forceUnits;

	/** Dynamics Engine. */
	PropertyObjPtr<AbstractDynamicsEngine> _dynamicsEngineProp;
	AbstractDynamicsEngine *&_dynamicsEngine;

	// SETS OF THINGS
	/** Actuators. */
	PropertyObj _actuatorSetProp;
	ActuatorSet& _actuatorSet;

	/** Contacts. */
	PropertyObj _contactSetProp;
	ContactForceSet& _contactSet;

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
	Model();
	Model(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;
	Model(const Model& aModel);
	virtual ~Model();
	virtual Object* copy() const;
	void copyData(const Model &aModel);
	void setupProperties();
	//static void registerTypes();

	/**
	 * Dynamic casting across JNI is messy. This method does the upCasting on C++ side
	 */
	Model* clone()
	{
		return static_cast<Model*>(this->copy()); 
	}

	virtual void setup() SWIG_DECLARE_EXCEPTION;
	void cleanup();

	bool builtOK() { return _builtOK; }

protected:
#ifndef SWIG
	Model& operator=(const Model &Model);
#endif

private:
	void setNull();

public:

	//--------------------------------------------------------------------------
	// FILE NAME
	//--------------------------------------------------------------------------
	virtual const std::string& getInputFileName() const { return _fileName; }
	virtual void setInputFileName(const std::string& fileName) { _fileName = fileName; }

	//--------------------------------------------------------------------------
	// CREDITS
	//--------------------------------------------------------------------------
	virtual const std::string& getCredits() const { return _creditsStr; }
	virtual void setAuthors(const std::string& aCredits) { _creditsStr = aCredits; }
	virtual const std::string& getPublications() const { return _publicationsStr; }
	virtual void setPublications(const std::string& aPublications) { _publicationsStr = aPublications; }

	//--------------------------------------------------------------------------
	// UNITS
	//--------------------------------------------------------------------------
	virtual const Units& getLengthUnits() const { return _lengthUnits; }
	virtual const Units& getForceUnits() const { return _forceUnits; }

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	virtual void getGravity(SimTK::Vec3& rGrav) const;
	virtual bool setGravity(SimTK::Vec3& aGrav);

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
	virtual int getNumContacts() const;
	virtual int getNumAnalyses() const;
	int getNumConfigurations() const { return getNumCoordinates() + getNumSpeeds(); }

	//--------------------------------------------------------------------------
	// DYNAMICS ENGINE
	//--------------------------------------------------------------------------
	bool hasDynamicsEngine() const;
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
	virtual void setStates(const Array<double> &aY) { setStates(&aY[0]); }
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
#ifndef SWIG
	const ActuatorSet* getActuatorSet() const;
#endif

	//--------------------------------------------------------------------------
	// CONTACT
	//--------------------------------------------------------------------------
	ContactForceSet* getContactSet();
#ifndef SWIG
	const ContactForceSet* getContactSet() const;
#endif

	//--------------------------------------------------------------------------
	// INTEGRATION CALLBACKS
	//--------------------------------------------------------------------------
	virtual IntegCallbackSet* getIntegCallbackSet();
#ifndef SWIG
	virtual const IntegCallbackSet* getIntegCallbackSet() const;
#endif
	virtual void addIntegCallback(IntegCallback *aCallback);
	virtual void removeIntegCallback(IntegCallback *aCallback);

	//--------------------------------------------------------------------------
	// DERIVATIVE CALLBACKS
	//--------------------------------------------------------------------------
	virtual DerivCallbackSet *getDerivCallbackSet();
#ifndef SWIG
	virtual const DerivCallbackSet *getDerivCallbackSet() const;
#endif
	virtual void addDerivCallback(DerivCallback *aCallback);
	virtual void removeAllDerivCallbacks();

	//--------------------------------------------------------------------------
	// ANALYSES
	//--------------------------------------------------------------------------
	virtual AnalysisSet* getAnalysisSet();
#ifndef SWIG
	virtual const AnalysisSet* getAnalysisSet() const;
#endif
	virtual void addAnalysis(Analysis *aAnalysis);
	virtual void removeAnalysis(Analysis *aAnalysis);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------
	virtual void computeDerivatives(double rDYDT[]);
	virtual void computeAuxiliaryDerivatives(double rDYDT[]);
	virtual void computeEquilibriumForAuxiliaryStates(double rY[]);

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
	void kinTest();

//=============================================================================
};	// END of class Model
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractModel_h__

