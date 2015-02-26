#ifndef OPENSIM_COMPONENT_LIST_ITERATOR_H_
#define OPENSIM_COMPONENT_LIST_ITERATOR_H_
/* -------------------------------------------------------------------------- *
 *                             OpenSim: ComponentListIterator.h               *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2014-2015 Stanford University and the Authors                *
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

#include "Component.h"

#include <iterator>

namespace OpenSim {

//==============================================================================
//                            OPENSIM ComponentListIterator
//==============================================================================
/** Class used to iterate over subcomponents of a specific type (by default,
 any Component).
 This iterator is const_iterator as it returns only const ref or pointer upon 
 dereferencing.
 This iterator works only in forward direction (not bidirectional)

 Use as:
@code
ComponentList<GeometryPath> geomPathList = model.getComponentList<GeometryPath>();
for (const GeometryPath& gpath : geomPathList) {
    // do something with gpath
}
@endcode
*/
template <typename T>
class OSIMCOMMON_API ComponentListIterator :
    public std::iterator<std::forward_iterator_tag, OpenSim::Component> {
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
     Prefer to use ++iter and not iter++. */
    ComponentListIterator<T>& operator++() {
        if (_node==nullptr)
            return *this;
        // If _node has children then successor is first child
        // move _node to point to it
        if (_node->_components.size() > 0)
            _node = _node->_components[0];
        // If processing a subtree under _root we stop when our successor is
        // the same as the successor of _root as this indicates we're leaving
        // the _root's subtree.
        else if (_node->_nextComponent.get() == _root._nextComponent.get())
            _node = nullptr;
        else
            // move on to the next component we computed earlier for the full
            // tree
            _node = _node->_nextComponent.get();
        // make sure we have a _node of type T after advancing
        advanceToNextValidComponent();
        return *this;
    }

    /** Postfix increment operator to get the next item in the ComponentList.
    To enable usage as iter++, although ++iter is more efficient. */
    ComponentListIterator<T> operator++(int) {
        ComponentListIterator<T> current = *this;
        next();
        return current;
    }

    /** Method equivalent to pre-increment operator for operator-deficient
     languages.
     */
    ComponentListIterator<T>& next() { return ++(*this); }
private:
    // Internal method to advance iterator to next valid component.
    void advanceToNextValidComponent() {
        // Advance _node to next valid (of type T) if needed
        // Similar logic to operator++ but applies _filter->isMatch()
        while (_node != nullptr &&
                (dynamic_cast<const T*>(_node) == nullptr ||
                 !_filter.isMatch(*_node))){
            if (_node->_components.size() > 0)
                _node = _node->_components[0];
            else {
                if (_node->_nextComponent.get() == _root._nextComponent.get()){
                    // end of subtree under _root
                    _node = nullptr;
                    continue;
                }
                _node = _node->_nextComponent;
            }
        }
        return;
    }
    // Pointer to current Component that the iterator is processing.
    const Component* _node;
    // Root of subtree of Components that we're iterating over.
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

#endif // OPENSIM_COMPONENT_LIST_ITERATOR_H_
