#ifndef _AbstractTool_h_
#define _AbstractTool_h_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  AbstractTool.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson                                               *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Object.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyStr.h>
#include <OpenSim/Common/PropertyStrArray.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/PropertyObj.h>
#include "ControllerSet.h"
#include "AnalysisSet.h"
#include "ForceSet.h"
#include "ExternalLoads.h"

namespace OpenSim { 

class Model;
class ForceSet;


//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an investigation.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API AbstractTool: public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(AbstractTool, Object);

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
    
    /** Whether the force set included in the model file is replaced 
    (if true) or appended to (if false) with force sets read in from file */
    PropertyBool _replaceForceSetProp;
    bool &_replaceForceSet;
    
    /** Names of the xml files used to construct a force set for the
    model. */
    PropertyStrArray _forceSetFilesProp;
    Array<std::string> &_forceSetFiles;
    
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
    
    /** A flag used to specify whether or not equilibrium is solved for
    the auxiliary states.  This often needs to be done for auxiliary states
    whose starting values are unknown (e.g., muscle fiber lengths). */
    OpenSim::PropertyBool _solveForEquilibriumForAuxiliaryStatesProp;
    bool &_solveForEquilibriumForAuxiliaryStates;
    
    /** Maximum number of steps for the integrator. */
    PropertyInt _maxStepsProp;
    int &_maxSteps;
        
    /** Maximum integration step size. */
    PropertyDbl _maxDTProp;
    double &_maxDT;
    
    /** Minimum integration step size. */
    PropertyDbl _minDTProp;
    double &_minDT;
    
    /** Integrator error tolerance. When the error is greater, the 
    integrator step size is decreased. */
    PropertyDbl _errorToleranceProp;
    double &_errorTolerance;
    
    /** Set of analyses to be run during the study. */
    PropertyObj _analysisSetProp;
    AnalysisSet &_analysisSet;
    
    // CONTROLLERS
    PropertyObj _controllerSetProp;
    ControllerSet& _controllerSet;
    
    /** Whether the tool owns the model it operates on. Important for cleanup when done */
    bool _toolOwnsModel;

    // EXTERNAL LOADS
    /** Name of the file containing the external loads applied to the model. */
    OpenSim::PropertyStr _externalLoadsFileNameProp;
    std::string &_externalLoadsFileName;
    
    /** ExternalLoads member for creating and editing applied external forces
        (e.g. GRFS through the GUI) prior to running the Tool */
    ExternalLoads   _externalLoads;
    // Reference to external loads added to the model but not owned by the Tool
    SimTK::ReferencePtr<ExternalLoads> _modelExternalLoads;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    
    /**
    * Destructor.
    */
    virtual ~AbstractTool();
    
    /**
    * Default constructor.
    */
    AbstractTool();
    
    /**
    * Construct from file, and an optional GuiModel
    *
    * The object is constructed from the root element of the XML document.
    * The type of object is the tag name of the XML root element.
    *
    * @param aFileName File name of the document.
    * @param aUpdateFromXMLNode
    */
    AbstractTool(const std::string &aFileName, bool aUpdateFromXMLNode = true);
    
    /**
    * Copy constructor.
    *
    * Copy constructors for all SimulationTools only copy the non-XML variable
    * members of the object; that is, the object's DOMnode and XMLDocument
    * are not copied but set to NULL.  This is because the object and all its 
    * derived classes need to establish the correct connection to the XML 
    * document nodes. Thus the object needs to reconstruct based on the XML 
    * document, not the values of the object's member variables.
    *
    * There are three proper ways to generate an XML document for a AbstractTool:
    *
    * 1) Construction based on XML file (@see AbstractTool(const char *aFileName)).
    * In this case, the XML document is created by parsing the XML file.
    *
    * 2) Construction by AbstractTool(const XMLDocument *aDocument).
    * This constructor explicitly requests construction based on an
    * XML document.  In this way the proper connection between an object's node
    * and the corresponding node within the XML document is established.
    * This constructor is a copy constructor of sorts because all essential
    * AbstractTool member variables should be held within the XML document.
    * The advantage of this style of construction is that nodes
    * within the XML document, such as comments that may not have any
    * associated AbstractTool member variable, are preserved.
    *
    * 3) A call to generateXMLDocument().
    * This method generates an XML document for the AbstractTool from scratch.
    * Only the essential document nodes are created (that is, nodes that
    * correspond directly to member variables.).
    *
    * @param aObject Object to be copied.
    * @see AbstractTool(const XMLDocument *aDocument)
    * @see AbstractTool(const char *aFileName)
    */
    AbstractTool(const AbstractTool &aObject);

private:
    // Keep pointers to analyses being added to model so that they can be removed later
    AnalysisSet _analysisCopies;     
    ControllerSet _controllerCopies;     

    /**
    * %Set all member variables to their null or default values.
    */
    void setNull();
    
    /**
    * Connect properties to local pointers.
    */
    void setupProperties();
    
    /**
    * Verify that column labels are unique.
    */
    bool verifyUniqueColumnLabels(const Storage& aStorage) const;
    std::string createExternalLoadsFile(const std::string& oldFile, 
                                          const std::string& body1, 
                                          const std::string& body2);

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    
    /**
    * Assignment operator.
    *
    * @return Reference to this object.
    */
    AbstractTool& operator=(const AbstractTool &aTool);

#endif

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    
    /**
    * %Set the model to be investigated.
    * NOTE: setup() should have been called on the model prior to calling this method
    */
    virtual void setModel(Model& aModel) SWIG_DECLARE_EXCEPTION;
    
    /**
    * Get the model to be investigated.
    */
    virtual Model& getModel() const;

    bool getReplaceForceSet() const { return _replaceForceSet; }
    void setReplaceForceSet(bool aReplace) { _replaceForceSet = aReplace; }

    std::string getNextAvailableForceName(const std::string prefix="Force") const;

    const ExternalLoads& getExternalLoads() const { return _externalLoads; }
    ExternalLoads& updExternalLoads() { return _externalLoads; }
    void setExternalLoads(ExternalLoads& el) { _externalLoads = el; }
    bool modelHasExternalLoads() {return !_modelExternalLoads.empty(); }

    // External loads get/set
    const std::string &getExternalLoadsFileName() const { return _externalLoadsFileName; }
    void setExternalLoadsFileName(const std::string &aFileName) { _externalLoadsFileName = aFileName; }

    Array<std::string> &getForceSetFiles() { return _forceSetFiles; }
    void setForceSetFiles(const Array<std::string> &aForceSetFiles) { _forceSetFiles = aForceSetFiles; }

    int getOutputPrecision() const { return _outputPrecision; }
    void setOutputPrecision(int aPrecision) { _outputPrecision = aPrecision; }

    const AnalysisSet& getAnalysisSet() const { return _analysisSet; }
    AnalysisSet& updAnalysisSet() const { return _analysisSet; }
    const ControllerSet& getControllerSet() const { return _controllerSet; }
    ControllerSet& updControllerSet() const { return _controllerSet; }

    /** 
    * Get Results Directory
    */
    const std::string& getResultsDir() const { return _resultsDir; }
    void setResultsDir(const std::string& aString) { _resultsDir = aString; }

    // AbstractTool time range
    double getInitialTime() const { return _ti; }
    double getFinalTime() const { return _tf; }
    void setInitialTime(const double aInitialTime) { _ti=aInitialTime; }
    void setFinalTime(const double aFinalTime) { _tf=aFinalTime; }
    
    // DEPRECATED: should use "initial" instead of "start"
    double getStartTime() const { return _ti; }
    void setStartTime(const double aStartTime) { _ti=aStartTime; } // deprecated: should use "initial" instead of "start"

    // Integrator settings
    int getMaximumNumberOfSteps() const { return _maxSteps; }
    void setMaximumNumberOfSteps(int aMaxSteps) { _maxSteps = aMaxSteps; }

    double getMaxDT() const { return _maxDT; }
    void setMaxDT(double aMaxDT) { _maxDT = aMaxDT; }

    double getMinDT() const { return _minDT; }
    void setMinDT(double aMinDT) { _minDT = aMinDT; }

    double getErrorTolerance() const { return _errorTolerance; }
    void setErrorTolerance(double aErrorTolerance) { _errorTolerance = aErrorTolerance; }

    // Model xml file
    const std::string& getModelFilename() const { return _modelFile; }
    void setModelFilename(const std::string& aModelFile) { _modelFile = aModelFile; }

    bool getSolveForEquilibrium() const { return _solveForEquilibriumForAuxiliaryStates; }
    void setSolveForEquilibrium(bool aSolve) { _solveForEquilibriumForAuxiliaryStates = aSolve; }

    //--------------------------------------------------------------------------
    // MODEL LOADING
    //--------------------------------------------------------------------------
    
    /**
    * Load and construct a model based on the property settings of
    * this investigation.
    */
    void loadModel(const std::string &aToolSetupFileName,
            ForceSet *rOriginalForceSet = 0 ) SWIG_DECLARE_EXCEPTION;
    
    /**
    * Update the forces applied to a model.
    */
    void updateModelForces(Model& model, const std::string &aToolSetupFileName, ForceSet *rOriginalForceSet = 0 )  SWIG_DECLARE_EXCEPTION;
    
    /**
    * Adds Analysis objects from analysis set to model.
    *
    * NOTE: Makes copies of analyses.  Also, both this tool and the model have ownership of their analysis
    * objects, therefore making a copy is necessary so a single analysis won't be deleted twice.
    *
    * To avoid leaking when the tool is run from the GUI, pointers to the model's copy of the analyses
    * are kept around so that they can be removed at the end of tool execution.
    *  _analysisCopies is used to do this book keeping.
    */
    void addAnalysisSetToModel();
    void addControllerSetToModel();
    
    /**
    * Remove Analysis objects that were added earlier from analysis set to model.
    */
    void removeControllerSetFromModel();
    void removeAnalysisSetFromModel();
    void setToolOwnsModel(const bool trueFalse) { _toolOwnsModel=trueFalse; };
    bool getToolOwnsModel() const { return _toolOwnsModel; };

    // Interface to build controller from a ControlSet file
    std::string getControlsFileName() const;
    void setControlsFileName(const std::string& controlsFilename);
    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    virtual bool run() SWIG_DECLARE_EXCEPTION=0;
    
    /**
    * Print the results of the analysis.
    *
    * @param aBaseName Basename file to which to print the data.
    * @param aDir Directory to which to print the data into.
    * @param aDT Time interval between results (linear interpolation is used).
    * If not included as an argument or negative, all time steps are printed
    * without interpolation.
    * @param aExtension Extension for written files.
    */
    void printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto");

    bool createExternalLoads( const std::string& externalLoadsFileName,
                              Model& model);
    void removeExternalLoadsFromModel();

    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber) override;
    virtual void loadQStorage (const std::string& statesFileName, Storage& rQStore) const;

protected:

    /// Obtain a string of the provided time using asctime(). This function
    /// removes the newline that asctime() includes at the end of the string.
    std::string getTimeString(const time_t& t) const;

//=============================================================================
};  // END of class AbstractTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __AbstractTool_h__


