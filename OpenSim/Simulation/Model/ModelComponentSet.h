#ifndef OPENSIM_MODEL_COMPONENT_SET_H
#define OPENSIM_MODEL_COMPONENT_SET_H
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ModelComponentSet.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
 * Author(s): Ajay Seth                                                       *
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

// INCLUDES
#include "ModelComponent.h"

#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/Set.h>
#include <OpenSim/Simulation/osimSimulationDLL.h>

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

class Model;

//=============================================================================
//=============================================================================
/**
 * This is the base class for sets of ModelComponent subclasses. It provides
 * methods for invoking all of the ModelComponent methods on each member of the
 * set.
 *
 * @tparam  T   This must be a concrete class derived from ModelComponent.
 */

template <typename T> using SetTModelComponent = Set<T, ModelComponent>;

template <class T = ModelComponent>
class ModelComponentSet : public Set<T, ModelComponent> {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(
            ModelComponentSet, T, SetTModelComponent<T>);

//============================================================================
// METHODS
//=============================================================================
public:
    using Super::Super;
    void extendFinalizeFromProperties() override final {
        Super::extendFinalizeFromProperties();
        // ModelComponentSets are unnamed properties of models, but as
        // components they must have a unique name. There is also nothing
        // stopping users from editing the XML to add a name.
        // We maintain consistency by overwriting any user set names with
        // the class name, which is also the default for the unnamed property.
        const std::string& name = this->getName();
        if (name != IO::Lowercase(getConcreteClassName())) {
            const std::string new_name = IO::Lowercase(getConcreteClassName());
            this->setName(new_name);
            log_info("'{}' was renamed and is being set to '{}'.",
                    getConcreteClassName(), new_name);
        }
    }

//=============================================================================
};  // END of class ModelComponentSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MODEL_COMPONENT_SET_H
