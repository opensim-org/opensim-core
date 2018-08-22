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
#include <OpenSim/Simulation/osimSimulationDLL.h>
#include "OpenSim/Common/Set.h"
#include "ModelComponent.h"

#ifdef SWIG
    #ifdef OSIMSIMULATION_API
        #undef OSIMSIMULATION_API
        #define OSIMSIMULATION_API
    #endif
#endif

namespace OpenSim {

class Model;
class ModelDisplayHints;

//=============================================================================
//=============================================================================
/**
 * This is the base class for sets of ModelComponent subclasses. It provides
 * methods for invoking all of the ModelComponent methods on each member of the
 * set.
 *
 * @tparam  T   This must be a concrete class derived from ModelComponent.
 */

template <class T=ModelComponent>
class ModelComponentSet : public Set<T, ModelComponent> {
    OpenSim_DECLARE_CONCRETE_OBJECT_T(ModelComponentSet, T, Set);

private:
    SimTK::ReferencePtr<Model> _model;

//=============================================================================
// METHODS
//=============================================================================
public:
    /** Default constructor creates an empty Set with no associated Model. **/
    ModelComponentSet() : _model(nullptr)
    {
    }
    /** Create an empty set associated to the specified Model. **/
    explicit ModelComponentSet(Model& model) : _model(&model)
    {
    }
    /**
     * Construct from file.
     *
     * @param[in]   model       The Model to which this set is associated.
     * @param[in]   fileName    Name of the file.
     * @param[in]   aUpdateFromXMLNode  
     *                          (Advanced) Used to avoid duplicate XML parsing.
     */
    ModelComponentSet(Model& model, const std::string& fileName, 
                      bool aUpdateFromXMLNode = true) 
    :   Super(fileName, aUpdateFromXMLNode), _model(&model)
    {
    }


    /** Does this Set have a Model associated with it? */
    bool hasModel() const { return !_model.empty(); }
    /**
     * Get this Model this set is part of.
     */
    const Model& getModel() const
    {
        if (hasModel()){
            return _model.getRef();
        }
        else{
            std::string msg = getClassName();
            msg += "::getModel() - has no associated Model (nullptr)";
            throw Exception(msg);
        }
    }
    /**
     * Get a modifiable reference to the Model this set is part of.
     */
    Model& updModel()
    {
        return *this->_model.get();
    }

    void setModel(Model& model) { _model = &model; }

    /**
     * Adding an object to the set causes its Model field to be set.
     */
    bool insert(int aIndex, T* object) override
    {
        return Super::insert(aIndex, object);
    }
    /**
     * Adding an object to the set causes its Model field to be set.
     */
    bool set(int aIndex, T* aObject, bool preserveGroups = false) override
    {
        return Super::set(aIndex, aObject, preserveGroups);
    }

//=============================================================================
};  // END of class ModelComponentSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // OPENSIM_MODEL_COMPONENT_SET_H

