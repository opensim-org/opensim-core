/* -------------------------------------------------------------------------- *
 *                            OpenSim:  ValueArray.h                          *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2015 Stanford University and the Authors                *
 * Authors:                                                                   *
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

#ifndef OPENSIM_VALUE_ARRAY_H_
#define OPENSIM_VALUE_ARRAY_H_

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"

namespace OpenSim {

/** ValueArray (of type T) represents an array of SimTK::Value (of type T). 
AbstractValueArray is the base class of all ValueArray thereby hiding the type
of the underlying array (which is T).                                         */
class AbstractValueArray {
public:
    using AbstractValue = SimTK::AbstractValue;

    AbstractValueArray()                                     = default;
    AbstractValueArray(const AbstractValueArray&)            = default;
    AbstractValueArray(AbstractValueArray&&)                 = default;
    AbstractValueArray& operator=(const AbstractValueArray&) = default;
    AbstractValueArray& operator=(AbstractValueArray&&)      = default;
    virtual ~AbstractValueArray()                            = default;

    virtual AbstractValueArray* clone() const = 0;

    /** Get the size of the array.                                            */
    virtual size_t size() const = 0;

    /** Access an element of the array using its index.                       */
    virtual AbstractValue& operator[](size_t index) = 0;

    /** Access an element of the array using its index.                       */
    virtual const AbstractValue& operator[](size_t index) const = 0;
};


template<typename T>
class ValueArray : public AbstractValueArray {
public:
    template<typename U>
    using Value = SimTK::Value<U>;

    ValueArray* clone() const override {
        return new ValueArray{*this};
    }

    /** Get the size of the array.                                            */
    size_t size() const override {
        return _values.size();
    }

    /** Access an element of the array using its index.                       

    \throws std::out_of_range If index is out of range.                       */
    Value<T>& operator[](size_t index) override {
        throwIfIndexOutOfRange(index);
        return _values.at(index);
    } 

    /** Access an element of the array using its index.                       

    \throws std::out_of_range If index is out of range.                       */
    const Value<T>& operator[](size_t index) const override {
        throwIfIndexOutOfRange(index);
        return _values.at(index);
    }

    /** Get the underlying array.                                             */
    const std::vector<Value<T>>& get() const {
        return _values;
    }

    /** Get the underlying array.                                             */
    std::vector<Value<T>>& upd() {
        return _values;
    }

private:
    void throwIfIndexOutOfRange(size_t index) const {
        if(index >= _values.size())
            throw IndexOutOfRange{__FILE__, __LINE__, __func__, index, 0, _values.size()};
    }
    
    std::vector<Value<T>> _values;
};

}

#endif // OPENSIM_VALUE_ARRAY_H_
