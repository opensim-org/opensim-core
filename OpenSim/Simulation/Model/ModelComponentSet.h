#ifndef __ModelComponentSet_h__
#define __ModelComponentSet_h__

// ModelComponentSet.h
// Authors: Peter Eastman, Ajay Seth
/*
 * Copyright (c) 2009, Stanford University. All rights reserved. 
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
#include "OpenSim/Common/Set.h"
#include "SimTKsimbody.h"
#include <OpenSim/Simulation/osimSimulationDLL.h>
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
 * This is the base class for sets of ModelComponent subclasses.  It provides
 * methods for invoking all of the ModelComponent methods on each member of the
 * set.
 */

template <class T>
class OSIMSIMULATION_API ModelComponentSet : public Set<T>
{
protected:
    	Model* _model;

//=============================================================================
// METHODS
//=============================================================================
public:
    friend class Model;
    ModelComponentSet() : _model(NULL)
    {
    }
    ModelComponentSet(Model& model) : _model(NULL)
    {
    }
    /**
     * Construct from file.
     *
     * @param aFileName Name of the file.
     */
    ModelComponentSet(Model& model, const std::string &aFileName, bool aUpdateFromXMLNode = true) :
        Set<T>(aFileName, aUpdateFromXMLNode), _model(&model)
    {
    }
    /**
     * Copy constructor.
     *
     * @param aSet Set to be copied.
     */
    ModelComponentSet(const ModelComponentSet<T> &aSet) : Set<T>(aSet)
    {
    }
    /**
     * Create a duplicate of this set.
     */
    Object* copy() const
    {
	    ModelComponentSet<T>* retObj = new ModelComponentSet<T>();
	    *retObj = *this;
	    return retObj;
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
    bool append(T *aObject)
    {
        return Set<T>::append(aObject);
    }
#ifndef SWIG
	bool append(T &aObject)
    {
        return Set<T>::append((T*) aObject.copy());
    }
#endif
    /**
     * Adding an object to the set causes its Model field to be set.
     */
    bool insert(int aIndex, T *aObject)
    {
        return Set<T>::insert(aIndex, aObject);
    }
    /**
     * Adding an object to the set causes its Model field to be set.
     */
    bool set(int aIndex, T *aObject, bool preserveGroups = false)
    {
        return Set<T>::set(aIndex, aObject, preserveGroups);
    }

protected:
    /**
     * Set the Model this object is part of.
     */
    virtual void setup(Model& model)
    {
        _model = &model;
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<ModelComponent&>(Set<T>::get(i)).setup(model);
		
		Set<T>::setup(); // make sure group members are populated
    }

    /**
     * Invoke createSystem() on each element of the Set.
     */
    virtual void createSystem(SimTK::MultibodySystem& system) const
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<const ModelComponent&>(Set<T>::get(i)).createSystem(system);
    }

    /**
     * Invoke initState() on each element of the Set.
     */
    virtual void initState(SimTK::State& state) const
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<const ModelComponent&>(Set<T>::get(i)).initState(state);
    }

    /**
     * Invoke setDefaultsFromState() on each element of the Set.
     */
    virtual void setDefaultsFromState(const SimTK::State& state)
    {
        for (int i = 0; i < Set<T>::getSize(); i++)
            static_cast<ModelComponent&>(Set<T>::get(i)).setDefaultsFromState(state);
    }


//=============================================================================
};	// END of class ModelComponentSet
//=============================================================================
//=============================================================================

} // end of namespace OpenSim

#endif // __ModelComponentSet_h__

