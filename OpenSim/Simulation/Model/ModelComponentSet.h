#ifndef __ModelComponentSet_h__
#define __ModelComponentSet_h__
/* -------------------------------------------------------------------------- *
 *                       OpenSim:  ModelComponentSet.h                        *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2012 Stanford University and the Authors                *
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
#include "Simbody.h"
#include "ModelComponent.h"

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

template <class T>
class ModelComponentSet : public Set<T> {
OpenSim_DECLARE_CONCRETE_OBJECT_T(ModelComponentSet, T, Set<T>);

protected:
    Model* _model;

//=============================================================================
// METHODS
//=============================================================================
public:
    /** Default constructor creates an empty Set with no associated Model. **/
    ModelComponentSet() : _model(NULL)
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
    :   Set<T>(fileName, aUpdateFromXMLNode), _model(&model)
    {
    }
    /**
     * Copy constructor.
     *
     * @param[in]   source      Set to be copied.
     */
    ModelComponentSet(const ModelComponentSet<T>& source) : Set<T>(source)
    {
    }

#ifndef SWIG
    /**
     * Get this Model this set is part of.
     */
    const Model& getModel() const
    {
        return *this->_model;
    }
    /**
     * Get a modifiable reference to the Model this set is part of.
     */
    Model& updModel()
    {
        return *this->_model;
    }
#endif
    /**
     * Adding an object to the set causes its Model field to be set.
     */
    bool insert(int aIndex, T* aObject)
    {
        return Set<T>::insert(aIndex, aObject);
    }
    /**
     * Adding an object to the set causes its Model field to be set.
     */
    bool set(int aIndex, T* aObject, bool preserveGroups = false)
    {
        return Set<T>::set(aIndex, aObject, preserveGroups);
    }

    // The following methods dispatch calls to the corresponding ModelComponent
    // methods. We have to upcast each concrete ModelComponent of type T to
    // ModelComponent so that we can invoke the methods, which are protected.
    // ModelComponent however declares ModelComponentSet as a friend.
    //
    // These methods are virtual because some derived sets need to override
    // them.

    /**
     * Set the Model this object is part of and allow each contained
     * ModelComponent to connect itself to the Model by invoking its 
     * connectToModel() method.
     * @see ModelComponent::connectToModel()
     */
    virtual void invokeConnectToModel(Model& model)
    {
        _model = &model;
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<ModelComponent&>(Set<T>::get(i)).connectToModel(model);
		
		Set<T>::setupGroups(); // make sure group members are populated
    }

    /**
     * Invoke addToSystem() on each element of the Set.
     * @see ModelComponent::addToSystem()
     */
    virtual void invokeAddToSystem(SimTK::MultibodySystem& system) const
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<const ModelComponent&>(Set<T>::get(i)).addToSystem(system);
    }

    /**
     * Invoke initStateFromProperties() on each element of the Set.
     * @see ModelComponent::initStateFromProperties()
     */
    virtual void invokeInitStateFromProperties(SimTK::State& state) const
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<const ModelComponent&>(Set<T>::get(i)).initStateFromProperties(state);
    }

    /**
     * Invoke setPropertiesFromState() on each element of the Set.
     * @see ModelComponent::setPropertiesFromState()
     */
    virtual void invokeSetPropertiesFromState(const SimTK::State& state)
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<ModelComponent&>(Set<T>::get(i)).setPropertiesFromState(state);
    }

    /** 
     * Invoke generateDecorations() on each of the contained 
     * ModelComponent objects. 
     * @see ModelComponent::generateDecorations()
     */
    virtual void invokeGenerateDecorations
       (bool                                        fixed, 
        const ModelDisplayHints&                    hints,
        const SimTK::State&                         state,
        SimTK::Array_<SimTK::DecorativeGeometry>&   appendToThis) const
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<const ModelComponent&>(Set<T>::get(i))
                    .generateDecorations(fixed,hints,state,appendToThis);
    }


//=============================================================================
};	// END of class ModelComponentSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ModelComponentSet_h__

