#ifndef SMC_Joint_h__
#define SMC_Joint_h__
/* -------------------------------------------------------------------------- *
 *                           OpenSim:  SMC_Joint.h                            *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Contributor(s): Frank C. Anderson                                          *
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
// This software, originally developed by Realistic Dynamics, Inc., was
// transferred to Stanford University on November 1, 2006.
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//============================================================================
// INCLUDE
//============================================================================
#include "CMC_Joint.h"

namespace OpenSim {

//=============================================================================
//=============================================================================
/**
 * A class for specifying the tracking task for a joint.
 *
 * @author Ko Sasaki
 * @version 1.0
 */
class OSIMTOOLS_API SMC_Joint : public CMC_Joint {
OpenSim_DECLARE_CONCRETE_OBJECT(SMC_Joint, CMC_Joint);

//=============================================================================
// DATA
//=============================================================================
protected:
    /** Parameter specifying the boundary of the error surface. */
    PropertyDbl _propS;
    double &_s;


//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    SMC_Joint(const std::string &aCoordinateName = "");
    SMC_Joint(const SMC_Joint &aTask);
    virtual ~SMC_Joint();

private:
    void setNull();
    void setupProperties();
    void copyData(const SMC_Joint &aTask);
    void updateWorkVariables();

    //--------------------------------------------------------------------------
    // OPERATORS
    //--------------------------------------------------------------------------
public:
#ifndef SWIG
    SMC_Joint& operator=(const SMC_Joint &aTask);
#endif

    //--------------------------------------------------------------------------
    // COMPUTATIONS
    //--------------------------------------------------------------------------
    void computeDesiredAccelerations(const SimTK::State& s, double aT) override;


//=============================================================================
};  // END of class SMC_Joint
//=============================================================================
//=============================================================================

}; // end namespace

#endif // SMC_Joint_h__


