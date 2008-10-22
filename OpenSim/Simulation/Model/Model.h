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

	/** Default constructor. */
	Model();

	/** 
	 * Constructor from an XML file. 
	 *
	 * @param aFileName XML file name.
	 */
	Model(const std::string &aFileName) SWIG_DECLARE_EXCEPTION;

	/**
	 * Copy constructor.
	 *
	 * @param aModel Model to be copied.
	 */
	Model(const Model& aModel);

	/** Destructor. */
	virtual ~Model();

	/**
	 * Copy this Model and return a pointer to the copy.
	 * The copy constructor for this class is used.
	 *
	 * @return Pointer to a copy of this Model.
	 */
	virtual Object* copy() const;

	/**
	 * Copy the member variables of the model.
	 *
	 * @param aModel model to be copied
	 */
	void copyData(const Model &aModel);

	/** Connect properties to local pointers. */
	void setupProperties();

	//Register the types used by this class.
	//static void registerTypes();

	/**
	 * Dynamic casting across JNI is messy. This method does the upCasting on C++ side
	 */
	Model* clone()
	{
		return static_cast<Model*>(this->copy()); 
	}

	/**
	 * Perform some set up functions that happen after the
	 * object has been deserialized. TODO: this method is
	 * not yet designed to be called after a model has been
	 * copied.
	 */
	virtual void setup() SWIG_DECLARE_EXCEPTION;

	/**
	 * Perform some clean up functions that are normally done 
	 * from the destructor however this gives the GUI a way to 
	 * proactively do the cleaning without waiting for garbage
	 * collection to do the actual cleanup.
	 */
	void cleanup();

	/**
	 * Indicates whether or not problems were encountered during
	 * model setup().
	 *
	 * @return True if the model has a dynamics engine and the 
	 * number of bodies is > 0.
	 */
	bool builtOK() { return _builtOK; }

protected:
#ifndef SWIG
	/**
	 * Assignment operator.
	 *
	 * @return Reference to this object.
	 */
	Model& operator=(const Model &Model);
#endif

private:

	/** Set the values of all data members to an appropriate "null" value. */
	void setNull();

public:

	//--------------------------------------------------------------------------
	// FILE NAME
	//--------------------------------------------------------------------------

	/** 
	 * Get the XML file name used to construct the model. 
	 *
	 * @return XML file name string.	
	 */
	virtual const std::string& getInputFileName() const { return _fileName; }

	/** 
	 * Set the XML file name used to construct the model.
	 *
	 * @param fileName The XML file name.
	 */
	virtual void setInputFileName(const std::string& fileName) { _fileName = fileName; }

	//--------------------------------------------------------------------------
	// CREDITS
	//--------------------------------------------------------------------------

	/** 
	 * Get the credits (e.g., model author names) associated with the model. 
	 *
 	 * @return Credits string.
	 */
	virtual const std::string& getCredits() const { return _creditsStr; }

	/** 
	 * Set the credits (e.g., model author names) associated with the model.
	 *
	 * @param aCredits The string of credits.
	 */
	virtual void setAuthors(const std::string& aCredits) { _creditsStr = aCredits; }

	/** 
	 * Get the publications associated with the model. 
	 *
	 * @return Publications string.
	 */
	virtual const std::string& getPublications() const { return _publicationsStr; }
	
	/** 
	 * Set the publications associated with the model. 
	 *
	 * @param aPublications The string of publications.
	 */
	virtual void setPublications(const std::string& aPublications) { _publicationsStr = aPublications; }

	//--------------------------------------------------------------------------
	// UNITS
	//--------------------------------------------------------------------------
	
	/** 
	 * Get the length units associated with the model. 
	 *
 	 * @return Length units.
	 */
	virtual const Units& getLengthUnits() const { return _lengthUnits; }
	
	/** 
	 * Get the force units associated with the model. 
	 *
	 * @return Force units
	 */
	virtual const Units& getForceUnits() const { return _forceUnits; }

	//--------------------------------------------------------------------------
	// GRAVITY
	//--------------------------------------------------------------------------
	
	/**
	 * Get the gravity vector in the gloabl frame.
	 *
	 * @param rGrav The XYZ gravity vector in the global frame is returned here.
	 */
	virtual void getGravity(SimTK::Vec3& rGrav) const;
	
	/**
	 * Set the gravity vector in the gloabl frame.
	 *
	 * @param aGrav The XYZ gravity vector
	 * @return Whether or not the gravity vector was successfully set.
	 */
	virtual bool setGravity(SimTK::Vec3& aGrav);

	//--------------------------------------------------------------------------
	// NUMBERS
	//--------------------------------------------------------------------------

	/**
	 * Get the number of controls in the model.
	 *
	 * @return Number of controls.
	 */
	virtual int getNumControls() const;

	/**
	 * Get the total number of states in the model.
	 *
	 * @return Number of states.
	 */
	virtual int getNumStates() const;

	/**
	 * Get the total number of pseudostates in the model.
	 *
	 * @return Number of pseudostates.
	 */
	virtual int getNumPseudoStates() const;

	/**
	 * Get the total number of bodies in the model.
	 *
	 * @return Number of bodies.
	 */
	virtual int getNumBodies() const;

	/**
	 * Get the total number of joints in the model.
	 *
	 * @return Number of joints.
	 */
	virtual int getNumJoints() const;

	/**
	 * Get the total number of coordinates in the model.
	 *
	 * @return Number of coordinates.
	 */
	virtual int getNumCoordinates() const;

	/**
	 * Get the total number of speeds in the model.
	 *
	 * @return Number of speeds.
	 */
	virtual int getNumSpeeds() const;

	/**
	 * Get the number of actuators in the model.
	 *
	 * @return The number of actuators.
	 */
	virtual int getNumActuators() const;

	/**
	 * Get the number of contacts in the model.
	 *
	 * @return The number of contacts.
	 */
	virtual int getNumContacts() const;

	/**
	 * Get the number of analyses in the model.
	 *
	 * @return The number of analyses.
	 */
	virtual int getNumAnalyses() const;

	/**
	 * Get the number of configurations in the model.
	 *
	 * @return The number of coordinates plus speeds.
	 */
	int getNumConfigurations() const { return getNumCoordinates() + getNumSpeeds(); }

	//--------------------------------------------------------------------------
	// DYNAMICS ENGINE
	//--------------------------------------------------------------------------
		
	/**
	 * Determines whether or not the model has a dynamics engine.
	 *
	 * @return True if model has a dynamics engine.
	 */	
	bool hasDynamicsEngine() const;

	/**
	 * Get the model's dynamics engine
	 *
	 * @return Reference to the abstract dynamics engine
	 */
	virtual AbstractDynamicsEngine& getDynamicsEngine() const;

	/**
	 * Set the model's dynamics engine
	 *
	 * @param aEngine The abstract dynamics engine to set to
	 */
	virtual void setDynamicsEngine(AbstractDynamicsEngine &aEngine);

	//--------------------------------------------------------------------------
	// SET TIME, CONTROLS, AND STATES
	//--------------------------------------------------------------------------

	/**
	 * Set the model time, controls, and states.
	 *
	 * @param aT Time.
	 * @param aX Controls at time aT.
	 * @param aY States at time aT.
	 */
	virtual void set(double aT,const double aX[],const double aY[]);

	//--------------------------------------------------------------------------
	// TIME
	//--------------------------------------------------------------------------

	/**
	 * Set the current time.
	 *
	 * @param Current time.
	 */
	virtual void setTime(double aTime);

	/**
	 * Get the current time.
	 *
	 * @return Current time.
	 */
	virtual double getTime() const;

	//--------------------------------------------------------------------------
	// TIME NORMALIZATION
	//--------------------------------------------------------------------------
	
	/**
	 * Set the constant by which time is normalized.
	 * The normalization constant must be greater than or equal to the constant
	 * rdMath::ZERO.
	 *
	 * @param Time normalization constant.
	 */	
	virtual void setTimeNormConstant(double aNormConst);

	/**
	 * Get the constant by which time is normalized.
	 * By default, the time normalization constant is 1.0.
	 *
	 * @return Current time normalization constant.
	 */
	virtual double getTimeNormConstant() const;

	//--------------------------------------------------------------------------
	// CONTROLS
	//--------------------------------------------------------------------------
	
	/**
	 * Set the controls in the model.
	 *
	 * @param aX Array of control values.
	 */	
	virtual void setControls(const double aX[]);

	/**
	 * Set a control value, specified by index.
	 *
	 * @param aIndex The index of the control to set.
	 * @param aValue The control value.
	 */
	virtual void setControl(int aIndex, double aValue);

	/**
	 * Set a control value, specified by name.
	 *
	 * @param aName The name of the control to set.
	 * @param aValue The control value.
	 */
	virtual void setControl(const std::string &aName, double aValue);

	/**
	 * Get the control values in the model.
	 *
	 * @param rX The control values are returned here.
	 */
	virtual void getControls(double rX[]) const;

	/**
	 * Get a control value, specified by index.
	 *
	 * @param aIndex The index of the control to get.
	 * @return The control value.
	 */
	virtual double getControl(int aIndex) const;

	/**
	 * Get a control value, specified by name.
	 *
	 * @param aName The name of the control to get.
	 * @return The control value.
	 */
	virtual double getControl(const std::string &aName) const;

	/**
	 * Get the name of a control.
	 *
	 * @param aIndex Index of the control whose name to get.
	 * @return Name of the control.
	 */
	virtual std::string getControlName(int aIndex) const;

	//--------------------------------------------------------------------------
	// STATES
	//--------------------------------------------------------------------------

	/**
	 * Get the names of the states.
	 *
	 * @param rStateNames Array of state names..
	 */
	virtual void getStateNames(Array<std::string> &rStateNames) const;

	/**
	 * Set the states for this model.
	 *
	 * @param aY Array of states.  The size of aY must be at least the number
	 * of states, which can be found by calling getNumStates().
	 */
	virtual void setStates(const Array<double> &aY) { setStates(&aY[0]); }

	/**
	 * Set the states for this model.
	 *
	 * @param aY Array of states.  The size of aY must be at least the number
	 * of states, which can be found by calling getNumStates().
	 */
	virtual void setStates(const double aY[]);

	/**
	 * Get the states for this model.
	 *
	 * @param rY Array of states.  The size of rY must be at least the number
	 * of states, which can be found by calling getNumStates().
	 */
	virtual void getStates(double rY[]) const;

	//virtual void setState(const std::string &aName,double aY);
	//virtual double getState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// INITIAL STATES
	//--------------------------------------------------------------------------

	/**
	 * Set the initial states for this model.
	 *
	 * @param aYI Array of states.  The size of aYI must be at least the number
	 * of states, which can be found by calling getNumStates().
	 */
	virtual void setInitialStates(const double aYI[]);

	/**
	 * Get the initial states for this model.
	 *
	 * @param rYI Array of states.  The size of rYI must be at least the number
	 * of states, which can be found by calling getNumStates().
	 */
	virtual void getInitialStates(double rYI[]) const;

	//virtual void setInitialState(const std::string &aName,double aYI);
	//virtual double getInitialState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// PSEUDOSTATES
	//--------------------------------------------------------------------------

	/**
	 * Get the names of the pseudo-states.
	 *
	 * @param rStateNames Array of pseudo-state names.
	 * @return Number of pseudo-states.
	 */
	virtual int getPseudoStateNames(Array<std::string> &rStateNames) const;

	/**
	 * Set the pseudo-states for this model.  Pseudo-states are quantities that
	 * depend on the time history of an integration but are not integrated.
	 * Pseudo-states cannot be computed from the states.
	 *
	 * @param aYP Array of pseudo-states.  The size of aYP must be at least the
	 * number of pseudo-states, which can be found by calling getNumPseudoStates().
	 */
	virtual void setPseudoStates(const double aYP[]);

	/**
	 * Get the pseudo-states for this model.  Pseudo-states are quantities that
	 * depend on the time history of an integration but are not integrated.
	 * Pseudo-states cannot be computed from the states.
	 *
	 * @param rYP Array of pseudo-states.  The size of rYP must be at least the
	 * number of pseudo-states, which can be found by calling getNumPseudoStates().
	 */
	virtual void getPseudoStates(double rYP[]) const;

	//virtual void setPseudoState(const std::string &aName,double aYP);
	//virtual double getPseudoState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// INITIAL PSEUDO STATES
	//--------------------------------------------------------------------------

	/**
	 * Set the initial pseudo-states for this model.
	 *
	 * @param aYPI Array of pseudo-states.  The size of aYPI must be at least the
	 * number of pseudo-states, which can be found by calling getNumPseudoStates().
	 */
	virtual void setInitialPseudoStates(const double aYPI[]);

	/**
	 * Get the initial pseudo-states for this model.
	 *
	 * @param rYPI Array of pseudo-states.  The size of rYPI must be at least the
	 * number of pseudo-states, which can be found by calling getNumPseudoStates().
	 */
	virtual void getInitialPseudoStates(double rYPI[]) const;

	//virtual void setInitialPseudoState(const std::string &aName,double aYPI);
	//virtual double getInitialPseudoState(const std::string &aName) const;

	//--------------------------------------------------------------------------
	// ACTUATORS
	//--------------------------------------------------------------------------

	/**
	 * Get the actuator set for the model.
	 *
	 * @return Pointer to the actuator set.
	 */
	ActuatorSet* getActuatorSet();
#ifndef SWIG
	/**
	 * Get the actuator set for the model.
	 *
	 * @return Pointer to the actuator set.
	 */
	const ActuatorSet* getActuatorSet() const;
#endif

	//--------------------------------------------------------------------------
	// CONTACT
	//--------------------------------------------------------------------------

	/**
	 * Get the contact set for the model.
	 *
	 * @return Pointer to the contact set.
	 */
	ContactForceSet* getContactSet();
#ifndef SWIG
	/**
	 * Get the contact set for the model.
	 *
	 * @return Pointer to the contact set.
	 */
	const ContactForceSet* getContactSet() const;
#endif

	//--------------------------------------------------------------------------
	// INTEGRATION CALLBACKS
	//--------------------------------------------------------------------------

	/**
	 * Get the set of integration callbacks.
	 *
	 * @retun Integration callback set of this model.
	 */
	virtual IntegCallbackSet* getIntegCallbackSet();
#ifndef SWIG
	/**
	 * Get the set of integration callbacks.
	 *
	 * @retun Integration callback set of this model.
	 */
	virtual const IntegCallbackSet* getIntegCallbackSet() const;
#endif

	/**
	 * Add an integration callback to the model
	 *
	 * @param aCallback Pointer to the integration callback to add.
	 */
	virtual void addIntegCallback(IntegCallback *aCallback);

	/**
	 * Remove an integration callback from the model
	 *
	 * @param aCallback Pointer to the integration callback to remove.
	 */
	virtual void removeIntegCallback(IntegCallback *aCallback);

	//--------------------------------------------------------------------------
	// DERIVATIVE CALLBACKS
	//--------------------------------------------------------------------------

	/**
	 * Get the set of derivative callbacks.
	 *
	 * @return Derivative callback set of this model.
	 */
	virtual DerivCallbackSet *getDerivCallbackSet();
#ifndef SWIG
	/**
	 * Get the set of derivative callbacks.
	 *
	 * @return Derivative callback set of this model.
	 */
	virtual const DerivCallbackSet *getDerivCallbackSet() const;
#endif

	/**
	 * Add a derivative callback to the model
	 *
	 * @param aCallback Pointer to the derivative callback to add.
	 */
	virtual void addDerivCallback(DerivCallback *aCallback);

	/**
	 * Remove getDerivCallbackSet from the model and free resources
	 */
	virtual void removeAllDerivCallbacks();

	//--------------------------------------------------------------------------
	// ANALYSES
	//--------------------------------------------------------------------------
	
	/**
	 * Get the set of analyses.
	 *
	 * @return Analysis set of this model.
	 */	
	virtual AnalysisSet* getAnalysisSet();
#ifndef SWIG
	/**
	 * Get the set of analyses.
	 *
	 * @return Analysis set of this model.
	 */
	virtual const AnalysisSet* getAnalysisSet() const;
#endif

	/**
	 * Add an analysis to the model.
	 *
	 * @param aAnalysis pointer to the analysis to add
	 */
	virtual void addAnalysis(Analysis *aAnalysis);

	/**
	 * Remove an analysis from the model
	 *
	 * @param aAnalysis Pointer to the analysis to remove.
	 */
	virtual void removeAnalysis(Analysis *aAnalysis);

	//--------------------------------------------------------------------------
	// DERIVATIVES
	//--------------------------------------------------------------------------

	/**
	 * Compute the derivatives of the states of the model.  For this method
	 * to return valid derivatives, the following methods should have been
	 * called:
	 * 1. Model::set()
	 * 2. ActuatorSet::computeActuation()
	 * 3. ActuatorSet::apply()
	 * 4. ContactSet::computeContact()
	 * 5. ContactSet::apply()
	 *
	 * @param rDYDT Derivatives of the states.  These have not been time
	 * normalized for integration, but are in un-normalized units.  The
	 * length of rDYDT should be at least getNumStates().
	 */
	virtual void computeDerivatives(double rDYDT[]);

	/**
	 * Compute the derivatives of the auxiliary states (the actuator and contact
	 * states).  The auxiliary states are any integrated variables that are
	 * not the coordinates or speeds.
	 *
	 * @param rDYDT Derivatives of the auxiliary states.  Note that this is a shorter
	 * array than the rDYDT used with computeDerivatives (above) as it omits the
	 * q and u derivatives. In particular, the length of rDYDT should be at least 
	 * _actuatorSet.getNumStates()+_contactSet.getNumStates()
	 * These have not been time normalized for integration, but are in un-normalized units.
	 */
	virtual void computeAuxiliaryDerivatives(double rDYDT[]);

	/**
	 * Compute values for the auxiliary states (i.e., states other than the
	 * generalized coordinates and speeds) that are in quasi-static equilibrium.
	 * The auxiliary states usually belong to the actuators (e.g., muscle
	 * activation and muscle fiber length).  The equilibrium computations
	 * are passed on to the owner of the the states.
	 *
	 * This methods is useful for computing initial conditions for a simulation
	 * or for computing torque-angle curves, for example.
	 *
	 * @param rY Array of states. The values sent in are used as the initial
	 * guess for equilibrium. The values returned are those that satisfy
	 * equilibrium.
	 */
	virtual void computeEquilibriumForAuxiliaryStates(double rY[]);

	//--------------------------------------------------------------------------
	// OPERATIONS
	//--------------------------------------------------------------------------

	/**
	 * Scale the model
	 *
	 * @param aScaleSet the set of XYZ scale factors for the bodies
	 * @param aFinalMass the mass that the scaled model should have
	 * @param aPreserveMassDist whether or not the masses of the
	 *        individual bodies should be scaled with the body scale factors.
	 * @return Whether or not scaling was successful.
	 */
	virtual bool scale(const ScaleSet& aScaleSet, double aFinalMass = -1.0, bool aPreserveMassDist = false);

	//--------------------------------------------------------------------------
	// PRINT
	//--------------------------------------------------------------------------

	/**
	 * Print some basic information about the model.
	 *
	 * @param aOStream Output stream.
	 */
	void printBasicInfo(std::ostream &aOStream) const;

	/**
	 * Print detailed information about the model.
	 *
	 * @param aOStream Output stream.
	 */
	void printDetailedInfo(std::ostream &aOStream) const;

	//--------------------------------------------------------------------------
	// TEST
	//--------------------------------------------------------------------------
	
	/** Test kinematics */
	void kinTest();

//=============================================================================
};	// END of class Model
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __AbstractModel_h__

