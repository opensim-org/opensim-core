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
 A class to specify a filter to be used to iterate through components. More 
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
    /** This is the meat of the ComponentFilter, returns false if the passed in 
    component should be skipped over. */
    virtual bool isMatch(const Component& comp) const = 0;
    /** clone() method needed to make a copy of the filter.  */
    virtual ComponentFilter* clone() const = 0;
};

/**
 ComponentFilterMatchAll is a trivial Filter that matches all components.
*/
class ComponentFilterMatchAll : public ComponentFilter {
public:
    /** Construct a ComponentFilter that selects all Components. */
    ComponentFilterMatchAll() {}
    /** The method isMatch() returns true always. */
    bool isMatch(const Component& comp) const {
        return true;
    }
    /** Destructor of ComponentFilterMatchAll, does nothing. */
    ~ComponentFilterMatchAll() {}
    /** Method to clone the filter. */
    ComponentFilterMatchAll* clone() const {
        return new ComponentFilterMatchAll(*this);
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
    const references/pointers, and that components can't be modified 
    through this iterator. */
    typedef ComponentListIterator<T> const_iterator;
    /** Constructor that takes a Component to iterate over (itself and its
    descendents) and a ComponentFilter. You can change the filter later
    using the setFilter() method. The filter is cloned on
    construction and can only be changed using setFilter().
    */
    ComponentList(const Component& root, const ComponentFilter& f) : 
        _root(root), _filter(f){
    }
    /** Constructor that takes only a Component to iterate over (itself and its
    descendents). ComponentFilterMatchAll is used internally. You can
    change the filter using setFilter() method. 
    */
    ComponentList(const Component& root) : _root(root){
        setDefaultFilter();
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
    /** Allow users to specify a custom ComponentFilter. This object makes a
     clone of the passed in filter.
     */
    void setFilter(const ComponentFilter& filter){
          _filter = filter;
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
    // Internal method to setFilter to ComponentFilterMatchAll if no user specified 
    // filter is provided.
    void setDefaultFilter() { setFilter(ComponentFilterMatchAll()); }
};

} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_LIST_H_

