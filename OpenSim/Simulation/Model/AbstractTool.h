#ifndef _SimulationTool_h_
#define _SimulationTool_h_
// SimulationTool.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyStrArray.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/PropertyObj.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
namespace OpenSim { 

class AbstractModel;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class RDSIMULATION_API SimulationTool: public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Pointer to the model being investigated. */
	AbstractModel *_model;

	// SERIALIZED PROPERTIES
	/** Name of the model library to load.  Do not include the library
	extension (e.g., .dll or .lib). */
	PropertyStr _modelLibraryProp;
	std::string &_modelLibrary;
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
	/** Name of the xml file used to construct a contact force set for the
	model. */
	PropertyStr _contactForceSetFileProp;
	std::string &_contactForceSetFile;
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

//=============================================================================
// METHODS
//=============================================================================
	//--------------------------------------------------------------------------
	// CONSTRUCTION
	//--------------------------------------------------------------------------
public:
	virtual ~SimulationTool();
	SimulationTool();
	SimulationTool(const std::string &aFileName);
	SimulationTool(const SimulationTool &aObject);
	//Object* copy() const;

private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	SimulationTool& operator=(const SimulationTool &aTool);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(AbstractModel *aModel);
	virtual AbstractModel* getModel() const;
	void setOutputPrecision(int aPrecision);
	int getOutputPrecision() const;
	AnalysisSet& getAnalysisSet() const;
	// Results Directory
	const std::string& getResultsDir() const {
		return _resultsDir;
	};
	void setResultsDir(const std::string& aString)
	{
		_resultsDir = aString;
	};
	// SimulationTool time range
	double getStartTime() const
	{
		return _ti;
	}
	double getFinalTime() const
	{
		return _tf;
	}
	// Model xml file
	const std::string& getModelFilename()
	{
		return _modelFile;
	}
	void setModelFilename(const std::string& aModelFile)
	{
		_modelFile = aModelFile;
	}
	//--------------------------------------------------------------------------
	// MODEL LOADING
	//--------------------------------------------------------------------------
	void loadModel();
	void addAnalysisSetToModel();

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual void run() = 0;
	virtual void printResults(const std::string &aBaseName,const std::string &aDir="",
		double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};	// END of class SimulationTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __SimulationTool_h__


