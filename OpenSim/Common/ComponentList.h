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
 * Copyright (c) 2005-2017 Stanford University and the Authors                *
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
#include <string>
#include <type_traits>
#include "SimTKcommon/basics.h"

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
    bool isMatch(const Component& comp) const override {
        return true;
    }
    /** Destructor of ComponentFilterMatchAll, does nothing. */
    ~ComponentFilterMatchAll() {}
    /** Method to clone the filter. */
    ComponentFilterMatchAll* clone() const override {
        return new ComponentFilterMatchAll(*this);
    }
};

/** A component is considered a match if its absolute path name contains the
given string. */
class OSIMCOMMON_API ComponentFilterAbsolutePathNameContainsString
        : public ComponentFilter {
public:
    ComponentFilterAbsolutePathNameContainsString(const std::string& substring)
        : _substring(substring) {}
    bool isMatch(const Component& comp) const override;
    ComponentFilterAbsolutePathNameContainsString* clone() const override {
        return new ComponentFilterAbsolutePathNameContainsString(*this);
    }
private:
    std::string _substring;
};


/**
Collection (linked list) of components to iterate through. Typical use is to
call getComponentList() on a component (e.g. model) to obtain an instance of 
this class, then call begin() to get an
iterator pointing to the first entry in the list then increment the iterator
until end(). 
The linked list is formed by tree pre-order traversal where each component is 
visited followed by all its immediate subcomponents (recursively).
@internal The traversal order is wired via
    Component::initComponentTreeTraversal(), which is called just before
    getting a ComponentList from a Component, either via
    Component::getComponentList() or Component::updComponentList().
*/
template <typename T>
class ComponentList {
public:

    static_assert(std::is_base_of<Component, T>::value,
        "Can only create a ComponentList of Components.");

    // This typedef is used to avoid "duplicate const" errors with SWIG,
    // caused when "T" is "const Component," leading to
    // "const const Component."
    typedef typename std::add_const<T>::type ConstT;
    
    /** A const forward iterator for iterating through ComponentList<T>.
    The const indicates that the iterator provides only 
    const references/pointers, and that components can't be modified 
    through this iterator. */
    typedef ComponentListIterator<ConstT> const_iterator;
    
    /** If T is const (e.g., ComponentList<const Body>), then this is
    the same as const_iterator, and does not allow modifying the elements.
    If T is not const, then this iterator allows modifying the elements. */
    typedef ComponentListIterator<T> iterator;
    
    /** Constructor that takes a Component to iterate over (itself and its
    descendants) and a ComponentFilter. You can change the filter later
    using the setFilter() method. The filter is cloned on
    construction and can only be changed using setFilter().
    */
    ComponentList(const Component& root, const ComponentFilter& f) : 
        _root(root), _filter(f) {
    }
    /** Constructor that takes only a Component to iterate over (itself and its
    descendants). ComponentFilterMatchAll is used internally. You can
    change the filter using setFilter() method. 
    */
    ComponentList(const Component& root) : _root(root) {
        setDefaultFilter();
    }
    /// Destructor of ComponentList.
    virtual ~ComponentList() {}
    /** Return an iterator pointing to the first component in the tree 
    traversal of components under and including the root component passed 
    to the ComponentList constructor. If T is non-const, then this iterator
    allows you to modify the elements of this list. */
    iterator begin() {
        return iterator(&_root, _filter.getRef());
    }
    /** Same as cbegin(). */
    const_iterator begin() const {
        return const_iterator(&_root, _filter.getRef());
    }
    /** Similar to begin(), except it does not permit
    modifying the elements of the list, even if T is non-const (e.g., 
    ComponentList<Body>). */
    const_iterator cbegin() const {
        return const_iterator(&_root, _filter.getRef());
    }
    /** Use this method to check if you have reached the end of the list.
    This points past the end of the list, *not* to the last item in the
    list. */
    iterator end() {
        return iterator(nullptr, _filter.getRef());
    }
    /** Same as cend(). */
    const_iterator end() const { return cend(); }
    /** Use this method to check if you have reached the end of the list.
    This points past the end of the list, *not* to the last item in the
    list. Use this if you used cbegin(). */
    const_iterator cend() const {
        return const_iterator(nullptr, _filter.getRef());
    }
    /** Allow users to specify a custom ComponentFilter. This object makes a
    clone of the passed in filter. */
    void setFilter(const ComponentFilter& filter) {
          _filter = filter;
    }
private:
    const Component& _root; // root of subtree to be iterated over
    SimTK::ClonePtr<ComponentFilter> _filter; // filter to choose components 
    // Internal method to setFilter to ComponentFilterMatchAll if no user specified
    // filter is provided.
    void setDefaultFilter() { setFilter(ComponentFilterMatchAll()); }
};

//==============================================================================
//                            OPENSIM ComponentListIterator
//==============================================================================
/** Class used to iterate over subcomponents of a specific type (by default,
any Component).
This iterator is can either be a const_iterator or non-const iterator, depending
on how you got it. If this is a const_iterator, it returns only a const
reference to a component. If this is a non-const iterator, then it returns
a non-const reference to a component, and thus you can modify the component.

If you got this iterator from something like a ComponentList<const Body>, then
it is necessarily a const_iterator. If you got this iterator from something like
ComponentList<Body>, then this may be either a const_iterator (e.g., from
ComponentList<Body>::cbegin()) or non-const iterator (e.g.,
from ComponentList<Body>::begin()).

If you have a non-const iterator, you should *not* add (sub)components to any
components.

This iterator works only in the forward direction (not bidirectional).

Here is an example of using this iterator with a range for loop (const_iterator):
@code
ComponentList<const GeometryPath> geomPathList = model.getComponentList<GeometryPath>();
for (const GeometryPath& gpath : geomPathList) {
    // do something with gpath
}
@endcode

Here is a similar example, but where you can modify the components:
@code
ComponentList<GeometryPath> geomPathList = model.updComponentList<GeometryPath>();
for (GeometryPath& gpath : geomPathList) {
    // do something with gpath
}
@endcode
*/
template <typename T>
class ComponentListIterator :
    public std::iterator<std::forward_iterator_tag, Component>
{
    // This typedef is used to avoid "duplicate const" errors with SWIG,
    // caused when "T" is "const Component," leading to
    // "const const Component."
    typedef typename std::add_const<T>::type ConstT;
    // The template argument T may be const or non-const; this typedef is
    // always the non-const variant of T. The typedef is useful for friend
    // declarations and casting.
    typedef typename std::remove_const<T>::type NonConstT;
    /** @internal Allow ComponentList to use the private constructor of this
    class. */
    friend class ComponentList<T>;
    /** @internal This is required to allow ComponentList<non-const T>::%begin()
    to construct a `ComponentListIterator<const T>` (that is, const_iterator). */
    friend class ComponentList<NonConstT>;
public:
    
    /// Check that the component under the current iterator is the same
    /// (has same address) as the right-hand iterator.
    // Templatized to allow comparison between const and non_const iterators.
    template <typename OtherT>
    bool operator==(const ComponentListIterator<OtherT>& other) const
    { return _node == other._node; }
    
    /// Check for inequality using same convention as operator==.
    template <typename OtherT>
    bool operator!=(const ComponentListIterator<OtherT>& other) const
    { return _node != &*other._node; }
    
    /// @ Comparison operators for scripting
    /// These variants accept only an iterator with the same template parameter.
    /// @{
    /// Check for (non)equality using a normal method rather than an operator.
    bool equals(const ComponentListIterator& other) const
    { return operator==<T>(other); }
    bool operator==(const ComponentListIterator& other) const
    { return operator==<T>(other); }
    bool operator!=(const ComponentListIterator& other) const
    { return operator!=<T>(other); }
    /// @}
    
    /** Dereference the iterator to get a reference to Component of proper
    type (matching Filter if specified). If you have a const iterator, then
    this returns a const reference; otherwise, this returns a non-const
    reference. */
    // This method is const to match other iterators, like that for
    // std::vector; one can mutate the component even if the iterator itself
    // is const (e.g., `const ComponentListIterator<T>`). This is consistent
    // with the behavior of const pointers: if one has an `int* const x`, then
    // `x = new int` is not valid but `*x = 5` is.
    T& operator*() const { return *operator->(); }

    /// Another dereferencing operator that returns a pointer.
    // The const cast is required for the case when T is not const. In the
    // case where T is const, it is okay that we do the const cast,
    // since the return type is still const.
    T* operator->() const
    { return const_cast<NonConstT*>(dynamic_cast<const T*>(_node)); }
    
    /** Prefix increment operator to get the next item in the ComponentList.
     Prefer to use ++iter and not iter++. */
    ComponentListIterator& operator++();

    /** Postfix increment operator to get the next item in the ComponentList.
    To enable usage as iter++, although ++iter is more efficient. */
    ComponentListIterator operator++(int) {
        ComponentListIterator current = *this;
        next();
        return current;
    }

    /** Method equivalent to pre-increment operator for operator-deficient
     languages. */
    ComponentListIterator& next() { return ++(*this); }
    
    /** Allow converting from non-const iterator to const_iterator.
    This helps with iterating through the list in a const
    way, even if you have a non-const list. This
    allows the following code:
    @code
    ComponentList<Body> bodies = model.getComponentList<Body>();
    ComponentList<Body>::const_iterator = bodies.begin();
    @endcode
    This constructor does *not* allow converting from a const_iterator to a 
    non-const iterator (through the enable_if). */
    // The intent is that FromT = std::remove_const<T>::type
    // Without the enable_if, this constructor would allow conversion of
    // const_iterator to iterator, which is *not* something we want.
    // We must use the pointer types in is_convertible, since T could be an
    // abstract class (AbstractClass is not convertible to const AbstractClass,
    // but AbstractClass* is convertible to const AbstractClass*).
    template <typename FromT>
    ComponentListIterator(const ComponentListIterator<FromT>& source,
        typename std::enable_if<std::is_convertible<FromT*, T*>::value>::type* = 0) :
        _node(source._node),
        _root(source._root),
        _filter(source._filter)
    {/*No need to advanceToNextValid; was done when source was constructed.*/}
    
    /** @internal ComponentListIterator<const T> needs access to the members
    of ComponentListIterator<T> for the templated constructor above. */
    friend class ComponentListIterator<ConstT>;
    /** @internal Comparison operators for ComponentListIterator<T> need
    access to members of ComponentListIterator<const T> (e.g., when invoking
    operator==() with a ComponentListIterator<T> as the left operand and a
    ComponentListIterator<const T> as the right operand). */
    friend class ComponentListIterator<NonConstT>;
private:
    // Internal method to advance iterator to next valid component.
    void advanceToNextValidComponent();
    // Pointer to current Component that the iterator is processing.
    // While it may seem that this variable should be non-const if T is
    // non-const, that would require duplicating the implementations of
    // advanceToNextValidComponent(), etc. So instead, we cast away the const
    // just before giving the node to the user (operator*() and operator->()).
    const Component* _node;
    // Root of subtree of Components that we're iterating over.
    const Component* _root = nullptr;
    /** Optional filter to further select Components under _root, defaults to
    Filter by type. */
    const ComponentFilter& _filter;
    
    /** Constructor that takes a Component and ComponentFilter.
     The iterator contains a const ref to filter and doesn't take ownership
     of it. A pointer is used since iterator at end() doesn't have a valid
     Component underneath it. */
    ComponentListIterator(const Component* node,
                          const ComponentFilter& filter) :
        _node(node),
        _root(node),
        _filter(filter) {
        advanceToNextValidComponent(); // in case node is not a match.
    }
}; // end of ComponentListIterator
} // end of namespace OpenSim

#endif // OPENSIM_COMPONENT_LIST_H_

