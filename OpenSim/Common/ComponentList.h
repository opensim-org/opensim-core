#ifndef OPENSIM_COMPONENT_LIST_H_
#define OPENSIM_COMPONENT_LIST_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: ComponentList.h                           *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2014 Stanford University and the Authors                *
 * Author(s): Ayman Habib                                                     *
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
#include <OpenSim/Common/osimCommonDLL.h>
#include "OpenSim/Common/Object.h"
#include "Simbody.h"

namespace OpenSim {

class Component;

template <typename T> class ComponentListIterator;
//==============================================================================
//                            OPENSIM ComponentList
//==============================================================================
// Class used to help iterate over specific Components
//
class ComponentFilter {
public:
    ComponentFilter() {};
    virtual ~ComponentFilter() {};
    virtual bool accept(const Component& comp) const = 0;
};
template <typename T>
class ComponentFilterByType : public ComponentFilter {
public:
    ComponentFilterByType() {};
    virtual ~ComponentFilterByType() {};
    bool accept(const Component& comp) const override {
        return dynamic_cast<const T*>(&comp);
    }
};

/**
 * example Compoenents that have geometry i.e. comp.hasGeometry() returns true
 * By having filters as classes rather than baked in it's more flexible and they can be used from Scripting
 * class CompoenentWithGeometryFilter: public ComponentFilter {
 * public:
 *   bool accept(const Component& comp) const override {
 *        return comp.hasGeometry();
    }
 * }
*/
template <typename T>
class ComponentList {
public:
    typedef ComponentListIterator<T> iterator;
    ComponentList(const Component* root) : m_root(root) {}
    iterator begin() {
        return ComponentListIterator<T>(m_root);
    }
    iterator end() {
        return nullptr;
    }
private:
    const Component* m_root;
    friend class ComponentListIterator<T>;
};

//==============================================================================
//                            OPENSIM ComponentListIterator
//==============================================================================
// Class used to iterate over subcomponents of specific type, default to all Compoentns 
//
/** Usa as:
@code
ComponentList<GeometryPath> geomPathList = model.getComponentList<GeometryPath>();
for (const GeometryPath& gpath : geomPathList) {
    // do something with gpath
}
@endcode
*/
template <typename T>
class ComponentListIterator {
    friend class ComponentList<T>;
public:
    bool operator==(const ComponentListIterator& iter) const {
        return m_node == &*iter;
    }
    bool operator!=(const ComponentListIterator& iter) const {
         return m_node != &*iter;
    }
    const T& operator*() const { return *dynamic_cast<const T*>(m_node); } // m_node need to be kept pointing at correct type otherwise will crash
    const T* operator->() const { return dynamic_cast<const T*>(m_node); }
    ComponentListIterator<T>& operator++();
    ComponentListIterator<T>& next() { return ++(*this); }
private:
    void advanceToNextValidComponent();
    const Component* m_node;
    ComponentListIterator(const Component* node) :
        m_node(node){
        advanceToNextValidComponent(); // in case node is not of type T
    };
};

} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_LIST_H_

