#ifndef _SymbolicExpressionReporter_h_
#define _SymbolicExpressionReporter_h_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  SymbolicExpressionReporter.h                   *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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


//=============================================================================
// INCLUDES
//=============================================================================
#include <map>
#include "OpenSim/OpenSim.h"
#include "osimExpPluginDLL.h"


#ifdef SWIG
    #ifdef OSIMEXPPLUGIN_API
        #undef OSIMEXPPLUGIN_API
        #define OSIMEXPPLUGIN_API
    #endif
#endif
//=============================================================================
//=============================================================================
namespace OpenSim { 

/**
 * A class for recording the states of a model
 * during a simulation.
 *
 * @author Ayman Habib
 * @version 1.0
 */
class OSIMEXPPLUGIN_API SymbolicExpressionReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(SymbolicExpressionReporter, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:
    std::map<std::string, double> _variables;


protected:
    /** Expression that's a function of state variables and primitive model properties. */
    PropertyStr _expressionStrProp;
    std::string& _expressionStr;

    /** States storage. */
    Storage _resultStore;

//=============================================================================
// METHODS
//=============================================================================
public:
    SymbolicExpressionReporter(Model *aModel=0);
    SymbolicExpressionReporter(const std::string &aFileName);
    // Copy constructor and virtual copy 
    SymbolicExpressionReporter(const SymbolicExpressionReporter &aObject);

    virtual ~SymbolicExpressionReporter();
private:
    void setNull();
    void constructDescription();
    void constructColumnLabels();
    void setupStorage();
    void setupProperties();

public:
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    SymbolicExpressionReporter& operator=(const SymbolicExpressionReporter &aRporter);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // STORAGE
    const Storage& getResultStorage() const
    {
        return _resultStore;
    };
    Storage& updResultStorage()
    {
        return _resultStore;
    }
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
#ifndef SWIG
    int
        begin(const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end(const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
#endif
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class SymbolicExpressionReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __SymbolicExpressionReporter_h__
