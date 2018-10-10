#ifndef _statesReporter_h_
#define _statesReporter_h_
/* -------------------------------------------------------------------------- *
 *                         OpenSim:  StatesReporter.h                         *
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
#include <OpenSim/Common/Storage.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimAnalysesDLL.h"


#ifdef SWIG
    #ifdef OSIMANALYSES_API
        #undef OSIMANALYSES_API
        #define OSIMANALYSES_API
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
class OSIMANALYSES_API StatesReporter : public Analysis {
OpenSim_DECLARE_CONCRETE_OBJECT(StatesReporter, Analysis);

//=============================================================================
// DATA
//=============================================================================
private:

protected:
    /** States storage. */
    Storage _statesStore;

//=============================================================================
// METHODS
//=============================================================================
public:
    StatesReporter(Model *aModel=0);
    StatesReporter(const std::string &aFileName);
    StatesReporter(const StatesReporter &aObject);
    virtual ~StatesReporter();

private:
    void setNull();
    void constructDescription();
    void constructColumnLabels();
    void setupStorage();

public:
    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
#ifndef SWIG
    StatesReporter& operator=(const StatesReporter &aRporter);
#endif
    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    // STORAGE
    const Storage& getStatesStorage() const
    {
        return _statesStore;
    };
    Storage& updStatesStorage()
    {
        return _statesStore;
    }
    //--------------------------------------------------------------------------
    // ANALYSIS
    //--------------------------------------------------------------------------
    int
        begin(const SimTK::State& s ) override;
    int
        step(const SimTK::State& s, int setNumber ) override;
    int
        end(const SimTK::State& s ) override;
protected:
    virtual int
        record(const SimTK::State& s );
    //--------------------------------------------------------------------------
    // IO
    //--------------------------------------------------------------------------
public:
    int
        printResults(const std::string &aBaseName,const std::string &aDir="",
        double aDT=-1.0,const std::string &aExtension=".sto") override;

//=============================================================================
};  // END of class StatesReporter

}; //namespace
//=============================================================================
//=============================================================================


#endif // #ifndef __StatesReporter_h__
