#ifndef __ModelComponentSet_h__
#define __ModelComponentSet_h__

// ModelComponentSet.h
// Authors: Peter Eastman, Ajay Seth
/*
 * Copyright (c) 2009-12, Stanford University. All rights reserved. 
* Use of the OpenSim software in source form is permitted provided that the following
* conditions are met:
* 	1. The software is used only for non-commercial research and education. It may not
*     be used in relation to any commercial activity.
* 	2. The software is not distributed or redistributed.  Software distribution is allowed 
*     only through https://simtk.org/home/opensim.
* 	3. Use of the OpenSim software or derivatives must be acknowledged in all publications,
*      presentations, or documents describing work in which OpenSim or derivatives are used.
* 	4. Credits to developers may not be removed from executables
*     created from modifications of the source.
* 	5. Modifications of source code must retain the above copyright notice, this list of
*     conditions and the following disclaimer. 
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
*  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
*  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
*  SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
*  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
*  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR BUSINESS INTERRUPTION) OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
*  WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

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
    bool append(T* aObject)
    {
        return Set<T>::append(aObject);
    }
#ifndef SWIG
	bool append(T& aObject)
    {
        return Set<T>::append(aObject.clone());
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

