#ifndef __SimmFileWriter_h__
#define __SimmFileWriter_h__
/* -------------------------------------------------------------------------- *
 *                              SimmFileWriter.h                              *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, P2C HD065690, U54 EB020405)   *
 * and by DARPA through the Warrior Web program.                              *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Peter Loan                                                      *
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


// INCLUDE
#include <iostream>
#include <fstream>
#include <string>
#include "osimSimmFileWriterDLL.h"
#include "SimTKcommon.h"
#include <OpenSim/Common/Array.h>

#ifdef SWIG
    #ifdef OSIMSIMMFILEWRITER_API
        #undef OSIMSIMMFILEWRITER_API
    #endif
    #define OSIMSIMMFILEWRITER_API
#endif

namespace OpenSim {

class Model;
class SimbodySimmModel;

//=============================================================================
//=============================================================================
/**
 * A class for writing SIMM joint and muscle files for an OpenSim Model. Right
 * now all of the work is done by the SimbodySimmModel object, but someday there
 * could be additions to the SimmFileWriter interface, like setting directories
 * and other preferences.
 *
 * @author Peter Loan
 * @version 1.0
 */
class OSIMSIMMFILEWRITER_API SimmFileWriter
{

//=============================================================================
// DATA
//=============================================================================
protected:
    const Model* _model;

    SimbodySimmModel* _simbodySimmModel;

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    SimmFileWriter();
    SimmFileWriter(const Model& aModel);
    virtual ~SimmFileWriter();

   bool writeMuscleFile(const std::string& aFileName);
   bool writeJointFile(const std::string& aFileName);

//=============================================================================
};  // END of class SimmFileWriter
//=============================================================================
//=============================================================================
} // end of namespace OpenSim

#endif // __SimmFileWriter_h__


