#ifndef OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
#define OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
/* -------------------------------------------------------------------------- *
 *                    OpenSim:  AnalysisPlugin_Template.h                     *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Frank C. Anderson, Ajay Seth, Tim Dorn                          *
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
// Headers define the various property types that OpenSim objects can read 
#include <OpenSim/Simulation/Model/Analysis.h>
#include "osimPluginDLL.h"      // Header to define plugin (DLL) interface


//=============================================================================
//=============================================================================
/*
 * A class template for writing a custom Analysis 
 * Currenlty reports the position and orientation of bodies listed in its
 * properties.
 *
 * @author Frank C. Anderson, Ajay Seth, Tim Dorn
 * @version 3.0
 */
namespace OpenSim { 

class OSIMPLUGIN_API AnalysisPlugin_Template : public Analysis 
{
OpenSim_DECLARE_CONCRETE_OBJECT(AnalysisPlugin_Template, Analysis);
public:
//=======================================================================
// PROPERTIES
//=======================================================================

    /** String list property containing the name of the body names*/
    OpenSim_DECLARE_LIST_PROPERTY(body_names, std::string, 
    "Names of the bodies on which to perform the analysis."
    "The key word 'All' indicates that the analysis should be performed for all bodies.");

    // Here are some examples of other scalar property types.
    // Uncomment them as you need them.
    // ------------------------------------------------------
    //// My string property
    //OpenSim_DECLARE_PROPERTY(string_property, std::string, 
    //"My string property."); 

    //// My int property
    //OpenSim_DECLARE_PROPERTY(int_property, int, 
    //"My int property."); 

    //// My bool property
    //OpenSim_DECLARE_PROPERTY(bool_property, bool, 
    //"My bool property."); 

    //// My double property
    //OpenSim_DECLARE_PROPERTY(double_property, double, 
    //"My double property."); 


//=======================================================================
// INTERNAL MEMBER VARIABLES
//=======================================================================

    // In addition to properties, add any additional member variables
    // you need for your analysis.  These variables are not read from
    // or written to file.  They are just variables you use to execute
    // your analysis.  For example, you will almost certainly need a
    // storage object for storing the results of your analysis.

    // Storage object for storing and writing out results.  In general,
    // each storage file that you create will contain only one kind of data.
    // Create a different storage object for each kind of data.  For example,
    // create a _storePos for positions, _storeVel for velocities,
    // _storeAcc for accelerations, etc. */

    // Indices of bodies for kinematics to be reported
    Array<int> _bodyIndices;

    /** Storage for recording body positions. */
    Storage _storePos;

    /** Internal work arrays to hold body positions at each time step. */
    Array<double> _bodypos;

//=============================================================================
// METHODS
//=============================================================================
private:
    /** Construct default values for internal member variables, */
    /** i.e., zero data and set pointers to Null */
    void setNull();

    /** Construct default values for properties */
    void constructProperties();

public:
    /** Default constructor */
    AnalysisPlugin_Template();

    /** setModel */
    void setModel(Model& aModel) override;

    //-------------------------------------------------------------------------
    // METHODS THAT MUST BE OVERRIDDEN
    //-------------------------------------------------------------------------
    int begin(const SimTK::State& s) override;
    int step(const SimTK::State& s, int stepNumber) override;
    int end(const SimTK::State& s) override;

    //-------------------------------------------------------------------------
    // IO
    //-------------------------------------------------------------------------
    int printResults(const std::string &aBaseName,
                     const std::string &aDir="",
                     double aDT=-1.0,
                     const std::string &aExtension=".sto") override;


protected:
    //========================== Internal Methods =============================
    int record(const SimTK::State& s);
    void constructDescription();
    void constructColumnLabels();
    void setupStorage();

//=============================================================================
}; // END of class AnalysisPlugin_Template
}; //namespace
//=============================================================================
//=============================================================================

#endif // #ifndef OPENSIM_ANALYSIS_PLUGIN_TEMPLATE_H_
