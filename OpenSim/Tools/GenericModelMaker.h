#ifndef OPENSIM_GENERIC_MODEL_MAKER_H_
#define OPENSIM_GENERIC_MODEL_MAKER_H_
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  GenericModelMaker.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "osimToolsDLL.h"
#include <OpenSim/Common/Object.h>

#ifdef SWIG
    #ifdef OSIMTOOLS_API
        #undef OSIMTOOLS_API
        #define OSIMTOOLS_API
    #endif
#endif

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * A class implementing a set of parameters describing
 * a generic musculoskeletal model.
 *
 * @author Peter Loan
 */
class OSIMTOOLS_API GenericModelMaker : public Object {
OpenSim_DECLARE_CONCRETE_OBJECT(GenericModelMaker, Object);

//=============================================================================
// DATA
//=============================================================================
public:
    OpenSim_DECLARE_PROPERTY(model_file, std::string,
            "Model file (.osim) for the unscaled model."); 

    OpenSim_DECLARE_PROPERTY(marker_set_file, std::string,
        "Set of model markers used to scale the model. "
        "Scaling is done based on distances between model markers compared to "
        "the same distances between the corresponding experimental markers.");

//=============================================================================
// METHODS
//=============================================================================
    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
public:
    GenericModelMaker();
    virtual ~GenericModelMaker() = default;

    Model* processModel(const std::string& aPathToSubject="");

    /**
     * Get file name for generic model
     */
    const std::string& getModelFileName() const
    {
        return get_model_file();
    }

    // Set model file name
    void setModelFileName(const std::string& fileName)
    {
        set_model_file(fileName);
    }

    const std::string& getMarkerSetFileName() const
    {
        return get_marker_set_file();
    }

    void setMarkerSetFileName(const std::string& fileName)
    {
        set_marker_set_file(fileName);
    }

private:
    void setNull();
    void constructProperties();
//=============================================================================
};  // END of class GenericModelMaker
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_GENERIC_MODEL_MAKER_H_ 


