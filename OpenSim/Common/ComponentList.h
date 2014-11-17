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
 A class to specifiy a filter to be used to iterate through components. More 
 flexible than filtering based on Type only. To write your custom filter, 
 extend this class and implement the isMatch() and clone() methods.
*/
class ComponentFilter {
protected:
    /** Default constructor of ComponentFilter, does nothing.  
    For use by derived classes only. */
    ComponentFilter() {}
public:
    /** Destructor of ComponentFilter, does nothing. */
    virtual ~ComponentFilter() {}
    /** This is the meat of the ComponentFilter, returns true if comp should
    be iterated over. */
    virtual bool isMatch(const Component& comp) const = 0;
    /** clone() method so that the filter is cached once passed in.  */
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
    /** Method to clone the filter so users don't change it behind us. */
    ComponentFilterByType<T>* clone() const {
        return new ComponentFilterByType<T>(*this);
    }
};
/**
 Collection (linked list) of components to iterate through.  Typical use is to
 call getComponentList() on a component (e.g. model) to obtain an instance of 
 this class, then call begin() to get an
 iterator pointing to the first entry in the list then increment the iterator
 until end(). 
 The linked list is formed by tree pre-order traversal where each component is 
 visited followed by all its immediate subcomponents (recursively).
 The traversal order is wired at the end of initSystem().
*/
template <typename T>
class ComponentList {
public:
    /** A const forward iterator for iterating through ComponentList<T>.
    The const indicates that the iterator provides only 
    const references/pointers, and that compoenents can't be modified 
    through this iterator. */
    typedef ComponentListIterator<T> const_iterator;
    /** Constructor that takes a Component to iterate over (itself and its
     descendents) and an optional ComponentFilter. If ComponentFilter is not
     specified then ComponentFilterByType is used internally. You can
     change the filter using setFilter() method. The filter is cloned on 
     construction and can only be changed using setFilter().
     */
    ComponentList(const Component& root, const ComponentFilter* f =nullptr
        ) : _root(root), _filter(f){
        if (f == nullptr) _filter = new ComponentFilterByType<T>();
    }
    /// Destructor of ComponentList.
    virtual ~ComponentList() {}
    /** Return an iterator pointing to the first component in the tree 
     traversal of components under and including the root component passed 
     to the ComponentList constructor.
     */
    ComponentListIterator<T> begin() {
        return ComponentListIterator<T>(&_root, _filter.getRef());
    }
    /** Allow users to specify a custom ComponentFilter. This object makes 
     clone of the passed in filter; you can delete the filter after this call.
     */
    void setFilter(const ComponentFilter* filter){
        if (filter == nullptr) {
            _filter = new ComponentFilterByType<T>();
        }
        else
          _filter = *filter;
    }
    /** Use this method to check if you have reached the end of the list.
    This points past the end of the list, *not* to the last item in the
    list.
    */
    ComponentListIterator<T> end() {
        return ComponentListIterator<T>(nullptr, _filter.getRef());
    }
private:
    const Component& _root; // root of subtree to be iterated over
    SimTK::ClonePtr<ComponentFilter> _filter; // filter to choose components 
    friend class ComponentListIterator<T>;
};

//==============================================================================
//                            OPENSIM ComponentListIterator
//==============================================================================
/** Class used to iterate over subcomponents of a specific type (by default,
 any Component).
 This iterator is const_iterator as it returns only const ref or pointer upon 
 dereferencing.
 This iterator works only in forward direction (not bidirectional)
 Only preIncrement (++iter) syntax is supported.

 Use as:
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
    /// Check that the component under the cuurent iterator is same 
    /// (has same address) as right-hand iter.
    bool operator==(const ComponentListIterator& iter) const {
        return _node == &*iter;
    }
    /// Check for inequality using same convention as operator==.
    bool operator!=(const ComponentListIterator& iter) const {
         return _node != &*iter;
    }
    /** Dereference the iterator to get a const ref to Component of proper 
     type (matching Filter if specified) this is const iterator, use only 
     const methods. 
    */
    const T& operator*() const { return *dynamic_cast<const T*>(_node); } 

    /// Another dereferencing operator that returns a const pointer.
    const T* operator->() const { return dynamic_cast<const T*>(_node); }

    /** Prefix increment operator to get the next item in the ComponentList.
     Only pre-increment is implemented, use ++iter and not iter++. */
    ComponentListIterator<T>& operator++();

    /** Method equivalent to increment operator for operator-deficient
     languages.
     */
    ComponentListIterator<T>& next() { return ++(*this); }
private:
    // Internal method to advance iterator to next valid component.
    void advanceToNextValidComponent();
    // Pointer to current Component that the iterator is processing.
    const Component* _node;
    // Root of subtree of Compoenents that we're iterating over.
    const Component& _root;
    /** Optional filter to further select Components under _root, defaults to
    Filter by type. */
    const ComponentFilter& _filter;
    /** Constructor that takes a Component and ComponentFilter.
     The iterator contains a const ref to filter and doesn't take ownership 
     of it. A pointer is used since iterator at end() doesn't have a valid 
     Component underneath it.
     */
    ComponentListIterator(const Component* node, const ComponentFilter& filter) :
        _node(node),
        _root(*node),
        _filter(filter) {
        advanceToNextValidComponent(); // in case node is not a match.
    }
};

} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_LIST_H_

