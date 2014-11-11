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
/**
* A class to specifiy a filter to be used to iterate thru compoenents. More flexible than filtering
* based on Type only. To write your custom filter, extend this class and implement the choose method.
* This's more typical visitor design pattern.
*/
class ComponentFilter {
public:
    ComponentFilter() {};
    virtual bool isMatch(const Component* comp) const = 0;
    virtual ~ComponentFilter() {}
};
/**
* A class to filter components based on Type, used as default filter for iterator
*/
template <typename T>
class ComponentFilterByType : public ComponentFilter {
public:
    ComponentFilterByType() {};
    bool isMatch(const Component* comp) const {
        return dynamic_cast<const T*>(comp) != nullptr;
    };
    virtual ~ComponentFilterByType() {}

};
/**
* Collection of components to iterate through
*/
template <typename T>
class ComponentList {
public:
    typedef ComponentListIterator<T> iterator;
    typedef ComponentFilter filter;
#ifndef SWIG
    ComponentList(const Component* root, filter* f = new ComponentFilterByType<T>()) : m_root(root), m_filter(f){}
#else
    ComponentList(const Component* root) : m_root(root), m_filter(new ComponentFilterByType<T>()){}
#endif
    virtual ~ComponentList() { delete m_filter;  }
    ComponentListIterator<T> begin() {
        return ComponentListIterator<T>(m_root, m_filter);
    }
    void setFilter(filter* filter){
        m_filter = filter;
    }
    ComponentListIterator<T> end() {
        return nullptr;
    }
private:
    const Component* m_root;
    filter* m_filter;
    friend class ComponentListIterator<T>;
};

//==============================================================================
//                            OPENSIM ComponentListIterator
//==============================================================================
// Class used to iterate over subcomponents of specific type, default to all Components
//
/** Use as:
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
    const Component* m_node; // Pointer to current Component that the iterator is processing
    const Component* _root; // Root of subtree of Compoenents that we're iterating over
    const ComponentFilter* m_filter; // Optional filter to further select Components under _root, defaults to Filter by type
#ifndef SWIG
    ComponentListIterator(const Component* node, ComponentFilter* filter = new ComponentFilterByType<T>()) :
        m_node(node),
        _root(node),
        m_filter(filter){
        if (filter == nullptr)
            filter = new ComponentFilterByType<T>();
        advanceToNextValidComponent(); // in case node is not of type T
    }
#else
    ComponentListIterator(const Component* node) :
        m_node(node),
        _root(node),
        m_filter(new ComponentFilterByType<T>()){
        advanceToNextValidComponent(); // in case node is not of type T
    }
#endif
};

} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_LIST_H_

