#ifndef OPENSIM_ANALYSIS_H
#define OPENSIM_ANALYSIS_H
/* -------------------------------------------------------------------------- *
 *                            OpenSim:  Analysis.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth                                    *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */

//============================================================================

#include <OpenSim/Simulation/osimSimulationDLL.h>
#include <OpenSim/Common/PropertyBool.h>
#include <OpenSim/Common/PropertyDbl.h>
#include <OpenSim/Common/PropertyInt.h>
#include <OpenSim/Common/ArrayPtrs.h>
#include <OpenSim/Common/Array.h>
#include <OpenSim/Common/Storage.h>

namespace SimTK {
class State;
}

namespace OpenSim { 

class Model;

//=============================================================================
//=============================================================================
/**
 * An abstract class for specifying the interface for an analysis
 * plugin.
 *
 * @author Frank C. Anderson, Ajay Seth
 * @version 1.0
 */
class OSIMSIMULATION_API Analysis: public Object {
OpenSim_DECLARE_ABSTRACT_OBJECT(Analysis, Object);

//=============================================================================
// DATA
//=============================================================================
public:
    Model* _model;
    const Storage* _statesStore;

private:
    /** Whether or not to write output of angles in degrees. */
    PropertyBool _inDegreesProp;
    bool &_inDegrees;

    // WORK ARRAYS
    /** Column labels. */
    Array<std::string> _labels;


protected:

    /** Step interval. */
    PropertyInt _stepIntervalProp;
    int &_stepInterval;

    /** On, off flag. */
    PropertyBool _onProp;
    bool &_on;

    /** Start time for the callback in normalized time. */
    PropertyDbl _startTimeProp;
    double &_startTime;

    /** End time for the callback in normalized time. */
    PropertyDbl _endTimeProp;
    double &_endTime;
    ArrayPtrs<Storage> _storageList;
    bool _printResultFiles;

//=============================================================================
// METHODS
//=============================================================================
private:
    void setNull();
    void setupProperties();

public:
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    /**
     * Default constructor.
     *
     * @param aModel Model on which the analysis is to be performed.
     */
    explicit Analysis(Model *aModel=0);

    /**
     * Construct an object from file.
     * The object is constructed from the root element of the XML document.
     * The type of object is the tag name of the XML root element.
     * @param aFileName File name of the document.
     * @param aUpdateFromXMLNode
     */
    Analysis(const std::string &aFileName, bool aUpdateFromXMLNode = true);

    /**
     * Copy constructor.
     * @param aAnalysis Object to be copied.
     * @see Analysis(const XMLDocument *aDocument)
     * @see Analysis(const char *aFileName)
     * @see generateXMLDocument()
     */
    Analysis(const Analysis &aAnalysis);

    virtual ~Analysis();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    /**
     * Assignment operator.
     * @return Reference to this object.
     */
    Analysis& operator=(const Analysis &aAnalysis);
#endif

    virtual int begin(const SimTK::State& s);
    virtual int step( const SimTK::State& s, int stepNumber);
    virtual int end( const SimTK::State& s);


    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // MODEL
    /**
     * set pointer to model to be analyzed.
     * @param aModel
     */
    virtual void setModel(Model& aModel);
    // STATES STORAGE
    /**
     * set states storage for analysis.
     * @param aStatesStore
     */
    virtual void setStatesStore(const Storage& aStatesStore);

    // ON,OFF
    void setOn(bool aTrueFalse);
    bool getOn() const;

    // START,END
    void setStartTime(double aStartTime);
    double getStartTime() const;

    void setEndTime(double aEndTime);
    double getEndTime() const;

    // DEGREES/RADIANS
    /**
     * %Set whether or not to write the output of angles in degrees.
     * This flag must be set before an analysis is performed to ensure that
     * the results are in the proper format.
     * @param aTrueFalse Output will be in degrees if "true" and in radians
     * if "false".
     */
    void setInDegrees(bool aTrueFalse);
    bool getInDegrees() const;

    virtual bool proceed(int aStep=0);

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setStepInterval(int aStepInterval);
    int getStepInterval() const;

    // COLUMN LABELS
    /**
     * %Set the column labels for this analysis.
     * @param aLabels an Array of strings (labels).
     */
    void setColumnLabels(const Array<std::string> &aLabels);
    const Array<std::string> &getColumnLabels() const;

#ifndef SWIG
    // These symbols are swigged out because they are not defined and never used!
    // STORAGE INTERVAL
    void setStorageInterval(int aInterval);
    int getStorageInterval() const;
#endif
    virtual ArrayPtrs<Storage>& getStorageList();
    void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }
    bool getPrintResultFiles() const { return _printResultFiles; }

    //--------------------------------------------------------------------------
    // RESULTS
    //--------------------------------------------------------------------------
    /**
     * Print the results of the analysis.
     *
     * @param aBaseName Base name of file to which to print the data.
     * @param aDir      Directory name.
     * @param aDT       Time interval between results (linear interpolation 
     *                  is used). If not supplied as an argument or negative, 
     *                  all time steps are printed without interpolation.
     * @param aExtension    File extension if not the default ".sto".
     *
     * @return -1 on error, 0 otherwise.
     */ 
    virtual int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto");

//=============================================================================
};  // END of class Analysis

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ANALYSIS_H


