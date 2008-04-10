#ifndef _AbstractTool_h_
#define _AbstractTool_h_
// AbstractTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/*
* Copyright (c)  2005, Stanford University. All rights reserved. 
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

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include "AnalysisSet.h"

namespace OpenSim { 

class Model;
class ActuatorSet;
class ContactForceSet;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractTool: public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Pointer to the model being investigated. */
	Model *_model;

	// SERIALIZED PROPERTIES
	/** Name of the xml file used to deserialize or construct a model. */
	PropertyStr _modelFileProp;
	std::string &_modelFile;
	/** Whether the actuator set included in the model file is replaced 
	(if true) or appended to (if false) with actuator sets read in from file */
	PropertyBool _replaceActuatorSetProp;
	bool &_replaceActuatorSet;
	/** Names of the xml files used to construct an actuator set for the
	model. */
	PropertyStrArray _actuatorSetFilesProp;
	Array<std::string> &_actuatorSetFiles;
   /** Whether the contact set included in the model file is replaced
   (if true) or appended to (if false) with actuator sets read in from file */
   PropertyBool _replaceContactForceSetProp;
   bool &_replaceContactForceSet;
	/** Name of the xml file used to construct a contact force set for the
	model. */
	PropertyStrArray _contactForceSetFilesProp;
	Array<std::string> &_contactForceSetFiles;
	/** Directory used for writing results. */
	PropertyStr _resultsDirProp;
	std::string &_resultsDir;
	/** Output precision. */
	PropertyInt _outputPrecisionProp;
	int &_outputPrecision;
	/** Initial time for the investigation. */
	PropertyDbl _tiProp;
	double &_ti;
	/** Final time for the investigation. */
	PropertyDbl _tfProp;
	double &_tf;
	/** A flag used to specify whether or not equilibrium is solved for for
	the auxiliary states.  This often needs to be done auxiliary sates whose
	starting values are unknown (e.g., muscle fiber lengths). */
	OpenSim::PropertyBool _solveForEquilibriumForAuxiliaryStatesProp;
	bool &_solveForEquilibriumForAuxiliaryStates;
	/** Maximum number of steps for the integrator. */
	PropertyInt _maxStepsProp;
	int &_maxSteps;
	/** Maximum integration step size. */
	PropertyDbl _maxDTProp;
	double &_maxDT;
	/** Integrator error tolerance. When the error is greater, the 
	integrator step size is decreased. */
	PropertyDbl _errorToleranceProp;
	double &_errorTolerance;
	/** Integrator fine tolerance. When the error is less, the
	integrator step size is increased. */
	PropertyDbl _fineToleranceProp;
	double &_fineTolerance;
	/** Set of analyses to be run during the investigation. */
	PropertyObj _analysisSetProp;
	AnalysisSet &_analysisSet;
	/** Whether the tool owns the model it operates on. Important for cleanup when done */
	bool _toolOwnsModel;
//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~AbstractTool();
	AbstractTool();
	AbstractTool(const std::string &aFileName, bool aUpdateFromXMLNode = true);
	AbstractTool(const AbstractTool &aObject);
	//Object* copy() const;

private:
	// Keep pointers to analyses being added to model so that they can be removed later
	AnalysisSet _analysisCopies;	 

	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	AbstractTool& operator=(const AbstractTool &aTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model *aModel) SWIG_DECLARE_EXCEPTION;
	virtual Model* getModel() const;

	bool getReplaceActuatorSet() const { return _replaceActuatorSet; }
	void setReplaceActuatorSet(bool aReplace) { _replaceActuatorSet = aReplace; }

	Array<std::string> &getActuatorSetFiles() { return _actuatorSetFiles; }
	void setActuatorSetFiles(const Array<std::string> &aActuatorSetFiles) { _actuatorSetFiles = aActuatorSetFiles; }

	int getOutputPrecision() const { return _outputPrecision; }
	void setOutputPrecision(int aPrecision) { _outputPrecision = aPrecision; }

	AnalysisSet& getAnalysisSet() const;

	// Results Directory
	const std::string& getResultsDir() const { return _resultsDir; }
	void setResultsDir(const std::string& aString) { _resultsDir = aString; }

	// AbstractTool time range
	double getInitialTime() const { return _ti; }
	double getFinalTime() const { return _tf; }
	void setInitialTime(const double aInitialTime) { _ti=aInitialTime; }
	void setFinalTime(const double aFinalTime) { _tf=aFinalTime; }
	// DEPRACATED: should use "initial" instead of "start"
	double getStartTime() const { return _ti; }
	void setStartTime(const double aStartTime) { _ti=aStartTime; } // depracated: should use "initial" instead of "start"

	// Integrator settings
	int getMaximumNumberOfSteps() const { return _maxSteps; }
	void setMaximumNumberOfSteps(int aMaxSteps) { _maxSteps = aMaxSteps; }

	double getMaxDT() const { return _maxDT; }
	void setMaxDT(double aMaxDT) { _maxDT = aMaxDT; }

	double getErrorTolerance() const { return _errorTolerance; }
	void setErrorTolerance(double aErrorTolerance) { _errorTolerance = aErrorTolerance; }

	double getFineTolerance() const { return _fineTolerance; }
	void setFineTolerance(double aFineTolerance) { _fineTolerance = aFineTolerance; }

	// Model xml file
	const std::string& getModelFilename() const { return _modelFile; }
	void setModelFilename(const std::string& aModelFile) { _modelFile = aModelFile; }

	bool getSolveForEquilibrium() const { return _solveForEquilibriumForAuxiliaryStates; }
	void setSolveForEquilibrium(bool aSolve) { _solveForEquilibriumForAuxiliaryStates = aSolve; }

	//--------------------------------------------------------------------------
	// MODEL LOADING
	//--------------------------------------------------------------------------
	void loadModel(const std::string &aToolSetupFileName, ActuatorSet *rOriginalActuatorSet = 0, ContactForceSet *rOriginalContactForceSet = 0);
	void updateModelActuatorsAndContactForces(Model *model, const std::string &aToolSetupFileName, ActuatorSet *rOriginalActuatorSet = 0, ContactForceSet *rOriginalContactForceSet = 0)  SWIG_DECLARE_EXCEPTION;
	void addAnalysisSetToModel();
	void removeAnalysisSetFromModel();
	void setToolOwnsModel(const bool trueFalse) { _toolOwnsModel=trueFalse; };
	const bool getToolOwnsModel() const { return _toolOwnsModel; };
	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual bool run() SWIG_DECLARE_EXCEPTION=0;
	virtual void printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class AbstractTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __AbstractTool_h__


