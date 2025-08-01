#ifndef __ForwardTool_h__
#define __ForwardTool_h__
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  ForwardTool.h                           *
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
#include <OpenSim/Simulation/Model/AbstractTool.h>

#include "osimToolsDLL.h"

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim { 

class Body;
class ControlSet;
class Manager;
class PrescribedForce;
class Storage;


//=============================================================================
//=============================================================================
/**
 * A concrete tool for performing forward dynamics simulations
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMTOOLS_API ForwardTool: public AbstractTool {
OpenSim_DECLARE_CONCRETE_OBJECT(ForwardTool, AbstractTool);

//=============================================================================
// MEMBER VARIABLES
//=============================================================================
protected:
    // BASIC INPUT

    /** Name of the states file.  The states file must at a minimum contain the
    initial states for a simulation.  If a complete set of states is available,
    the time stamps can be used to specify the integration steps and corrective
    springs, which allow perturbations, can be added to the simulation. */
    PropertyStr _statesFileNameProp;
    std::string &_statesFileName;

    /** Storage for the input states. */
    Storage *_yStore;

    /** Flag indicating whether or not to write to the results (GUI will set this to false). */
    bool _printResultFiles;

    /** pointer to the simulation Manager */
    Manager* _manager;

    /*** Private place to save some deserialization info in case needed later */
    std::string _parsingLog;
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    virtual ~ForwardTool();
    ForwardTool();
    ForwardTool(const std::string &aFileName,bool aUpdateFromXMLNode=true,bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;
    ForwardTool(const ForwardTool &aObject);

private:
    void setNull();
    void setupProperties();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    ForwardTool&
        operator=(const ForwardTool &aForwardTool);
#endif

    void updateFromXMLNode(SimTK::Xml::Element& aNode, int versionNumber=-1) override;
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------

    void setManager( Manager& m );
    const Manager& getManager() const; 

    const std::string &getStatesFileName() const { return _statesFileName; }
    void setStatesFileName(const std::string &aFileName) { _statesFileName = aFileName; }

    void setPrintResultFiles(bool aToWrite) { _printResultFiles = aToWrite; }

    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    bool run() override SWIG_DECLARE_EXCEPTION;
    /// <b>(Deprecated)</b> Use setPrintResultFiles(true) and run() instead.
    [[deprecated("Use setPrintResultFiles(true) and run() instead.")]]
    void printResults();
private:
    void printResultsInternal();
public:

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    static double Step(double t, double t0, double t1); 
    static double SigmaUp(double tau,double to,double t);
    static double SigmaDn(double tau,double to,double t);

    void loadStatesStorage (std::string& statesFileName, Storage*& rYStore) const; 
    const std::string& getParsingLog() { return _parsingLog; };
protected:
    void setDesiredStatesForControllers(Storage& rYStore);
    int determineInitialTimeFromStatesStorage(double &rTI);
private:

//=============================================================================
};  // END of class ForwardTool

}; //namespace
//=============================================================================
//=============================================================================

#endif // __ForwardTool_h__


