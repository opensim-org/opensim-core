#ifndef OPENSIM_ANALYSIS_SET_H_
#define OPENSIM_ANALYSIS_SET_H_
/* -------------------------------------------------------------------------- *
 *                          OpenSim:  AnalysisSet.h                           *
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

/* Note: This code was originally developed by Realistic Dynamics Inc. 
 * Author: Frank C. Anderson 
 */


// INCLUDES
#include <string>
#include <OpenSim/Common/Set.h>
#include "Analysis.h"


//=============================================================================
//=============================================================================
namespace OpenSim { 

class Model;

/**
 * A class for holding and managing a set of integration callbacks for
 * a model.
 *
 * @author Frank C. Anderson
 * @version 1.0
 */
class OSIMSIMULATION_API AnalysisSet : public Set<Analysis> {
OpenSim_DECLARE_CONCRETE_OBJECT(AnalysisSet, Set<Analysis>);

//=============================================================================
// DATA
//=============================================================================
public:
#ifndef SWIG
   AnalysisSet&
        operator=(const AnalysisSet &aAnalysisSet);
#endif
protected:
    /** Model on which the callbacks have been set. */
    Model *_model;

    // testing for memory free error
    OpenSim::PropertyBool _enableProp;
    bool &_enable;
//
//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    AnalysisSet();
    AnalysisSet(Model *aModel);
    AnalysisSet(const std::string &aFileName);
    AnalysisSet(const AnalysisSet &aSet);
    virtual ~AnalysisSet();

private:
    void setNull();
    void setupProperties();
public:

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setModel(Model& aModel);
    Model& getModel();
    void setOn(bool aTrueFalse);
    void setOn(const Array<bool> &aOn);
    Array<bool> getOn() const;

    //--------------------------------------------------------------------------
    // CALLBACKS
    //--------------------------------------------------------------------------
    void begin(const SimTK::State& s );
    void step(const SimTK::State& s, int stepNumber );
    void end(const SimTK::State& s );

    //--------------------------------------------------------------------------
    // RESULTS
    //--------------------------------------------------------------------------
    virtual void
        printResults(const std::string &aBaseName,const std::string &aPath="",
        double aDT=-1.0,const std::string &aExtension=".sto");

    //--------------------------------------------------------------------------
    // UTILITY
    //--------------------------------------------------------------------------
    static void getAvailableAnalyses(AnalysisSet& analysisset);

//=============================================================================
};  // END of class AnalysisSet

}; //namespace
//=============================================================================
//=============================================================================

#endif // OPENSIM_ANALYSIS_SET_H_


