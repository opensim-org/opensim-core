#ifndef OPENSIM_COMPONENT_LIST_H_
#define OPENSIM_COMPONENT_LIST_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: ComponentList.h                       *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2014-2014 Stanford University and the Authors                *
 * Authors: Ayman Habib                                                       *
 * Contributers : Chris Dembia                                                *
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
* A class to specifiy a filter to be used to iterate thru components. More 
* flexible than filtering based on Type only. To write your custom filter, 
* extend this class and implement the isMatch method.
*/
// This is more typical visitor design pattern.
class ComponentFilter {
public:
    /** Default constructor of ComponentFilter */
    ComponentFilter() {}
    /** This is the meat of the ComponentFilter, returns true if comp should
    be iterated over. */
    virtual bool isMatch(const Component& comp) const = 0;
    /** Destructor of ComponentFilter. */
    virtual ~ComponentFilter() {}
    /** clone() method so that the the ilter is cached once passed in  */
    virtual ComponentFilter* clone() const = 0;
};
/**
* A class to filter components based on Type, used as default filter for
* iterator.
*/
template <typename T>
class ComponentFilterByType : public ComponentFilter {
public:
    /** Construct a ComponentFilter that selects Components based on type T. */
    ComponentFilterByType() {}
    /** The method isMatch() returns true if comp is of type T or a derived 
    class. */
    bool isMatch(const Component& comp) const {
        return dynamic_cast<const T*>(&comp) != nullptr;
    }
    /** Destructor of ComponentFilterByType. */
    ~ComponentFilterByType() {}
    /** Method to clone the filter for internal use. */
    ComponentFilterByType<T>* clone() const {
        return new ComponentFilterByType<T>(*this);
    }
};
/**
* Collection (linked list) of components to iterate through.  Typical use is to
* call getComponentList() to obtain an instance of this class, begin() to get
* iterator pointing to the first entry in the list then advance the iterator
* until end().
*/
template <typename T>
class ComponentList {
public:
    /// An iterator for iterating through a ComponentList<T>.
    /// The const means that the iterator provides const references/pointers.
    typedef ComponentListIterator<T> iterator;
    /** Constructor that takes a Component to iterate over (itself and
     *  descendents) and an optional ComponentFilter. If ComponentFilter is not
     *  specified then ComponentFilterByType is used. You can
     * change the filter using setFilter(). The filter is cloned on construction
     * and can only be changed using setFilter().
     */
    ComponentList(const Component& root, ComponentFilter* f =nullptr
        ) : _root(root), _filter(f){
        if (f == nullptr){
            ComponentFilter* defaultFilter = new ComponentFilterByType<T>();
            _filter = *defaultFilter; // this makes a clone/local copy
            delete defaultFilter;
        }
    }
    /// Destructor.
    virtual ~ComponentList() {}
    /// Return iterator over the tree of Components rooted at the Component
    /// passed to ComponentList constructor.
    ComponentListIterator<T> begin() {
        return ComponentListIterator<T>(&_root, _filter.getPtr());
    }
    /// Allow users to specify a custom ComponentFilter.  If a filter is passed
    /// Specify a filter through this list. An iterator over this list will
    /// yield Component's that match this filter. This object makes clone of 
    /// the filter; you can delete the filter.
    void setFilter(ComponentFilter* filter){
        _filter.clear();
         _filter = *filter;
    }
    /// Use this to check if you have reached the end of the list.
    /// This points past the end of the list, *not* to the last item in the
    /// list.
    ComponentListIterator<T> end() {
        return ComponentListIterator<T>(nullptr, _filter.getPtr());
    }
private:
    const Component& _root; // root of subtree to be iterated over
    SimTK::ClonePtr<ComponentFilter> _filter; //     ComponentFilter to choose components 
    friend class ComponentListIterator<T>;
};

//==============================================================================
//                            OPENSIM ComponentListIterator
//==============================================================================
/** Class used to iterate over subcomponents of a specific type (by default,
 * any Component).
 * This is const iterator that returns const ref or a pointer on dereferencing.
 * This works only in forward direction (not bidirectional)
 * Use as:
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
        return _node == &*iter;
    }
    bool operator!=(const ComponentListIterator& iter) const {
         return _node != &*iter;
    }
    /// Dereference the iterator to get a Component of proper type (matching
    /// Filter if specified) this is const iterator, use only const methods 
    const T& operator*() const { return *dynamic_cast<const T*>(_node); } 
    const T* operator->() const { return dynamic_cast<const T*>(_node); }
    /** Prefix increment operator to get the next item in the ComponentList.
     * Only pre-increment is implemented, use ++iter and not iter++. */
    ComponentListIterator<T>& operator++();
    /// Method equivalent to increment operator for operator-deficient
    /// languages.
    ComponentListIterator<T>& next() { return ++(*this); }
private:
    // Internal method to advance iterator to next valid component.
    void advanceToNextValidComponent();
    // Pointer to current Component that the iterator is processing.
    const Component* _node;
    // Root of subtree of Compoenents that we're iterating over.
    const Component& _root;
    // Optional filter to further select Components under _root, defaults to
    // Filter by type.
    const ComponentFilter* _filter;
    /** Constructor that takes a Component and optional ComponentFilter.
     * If a ComponentFilter is passed in, the iterator takes ownership of it.
     */
    ComponentListIterator(const Component* node, const ComponentFilter* filter) :
        _node(node),
        _root(*node),
        _filter(filter) {
        advanceToNextValidComponent(); // in case node is not of type T
    }
};

} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_LIST_H_

