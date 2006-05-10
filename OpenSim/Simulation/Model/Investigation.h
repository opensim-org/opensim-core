#ifndef _Investigation_h_
#define _Investigation_h_
// Investigation.h
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#include <OpenSim/Tools/Object.h>
#include <OpenSim/Tools/PropertyBool.h>
#include <OpenSim/Tools/PropertyStr.h>
#include <OpenSim/Tools/PropertyInt.h>
#include <OpenSim/Tools/ArrayPtrs.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/AnalysisSet.h>
namespace OpenSim { 

class XMLDocument;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class RDSIMULATION_API Investigation: public Object
{
//=============================================================================
// DATA
//=============================================================================
protected:
	/** Pointer to the model being investigated. */
	Model *_model;

	// SERIALIZED PROPERTIES
	/** Name of the model library to load.  Do not include the library
	extension (e.g., .dll or .lib). */
	PropertyStr _modelLibraryProp;
	std::string &_modelLibrary;
	/** Name of the xml file used to deserialize or construct a model. */
	PropertyStr _modelFileProp;
	std::string &_modelFile;
	/** Name of the xml file used to construct an actuator set for the
	model. */
	PropertyStr _actuatorSetFileProp;
	std::string &_actuatorSetFile;
	/** Name of the xml file used to construct a contact force set for the
	model. */
	PropertyStr _contactForceSetFileProp;
	std::string &_contactForceSetFile;
	/** Name of the params files used for a SIMM Pipeline model. */
	PropertyStr _paramsFileProp;
	std::string &_paramsFile;
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
	virtual ~Investigation();
	Investigation();
	Investigation(const std::string &aFileName);
	Investigation(DOMElement *aElement);
	Investigation(const Investigation &aObject);
	//Object* copy() const;
	//Object* copy(DOMElement *aElement) const;

private:
	void setNull();
	void setupProperties();

	//--------------------------------------------------------------------------
	// OPERATORS
	//--------------------------------------------------------------------------
public:
#ifndef SWIG
	Investigation& operator=(const Investigation &aInvestigation);
#endif

	//--------------------------------------------------------------------------
	// GET AND SET
	//--------------------------------------------------------------------------
	virtual void setModel(Model *aModel);
	virtual Model* getModel() const;
	void setOutputPrecision(int aPrecision);
	int getOutputPrecision() const;
	AnalysisSet& getAnalysisSet() const;
	const std::string& getResultsDir() const {
		return _resultsDir;
	};
	void setResultsDir(const std::string& aString)
	{
		_resultsDir = aString;
	};

	//--------------------------------------------------------------------------
	// MODEL LOADING
	//--------------------------------------------------------------------------
	void constructCommandLineForLoadModel(Array<std::string> &args);
	void loadModel();

	//--------------------------------------------------------------------------
	// INTERFACE
	//--------------------------------------------------------------------------
	virtual void run() = 0;
	virtual void printResults(const char *aBaseName,const char *aDir=NULL,
		double aDT=-1.0,const char *aExtension=".sto");

//=============================================================================
};	// END of class Investigation

}; //namespace
//=============================================================================
//=============================================================================

#endif // __Investigation_h__


