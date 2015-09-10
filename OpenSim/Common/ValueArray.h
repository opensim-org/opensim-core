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

#ifndef OPENSIM_VALUEARRAY_H_
#define OPENSIM_VALUEARRAY_H_

// Non-standard headers.
#include "SimTKcommon.h"
#include "OpenSim/Common/Exception.h"

namespace OpenSim {

class AbstractValueArray {
public:
    using AbstractValue = SimTK::AbstractValue;

    virtual AbstractValueArray* clone() const = 0;

    virtual SimTK::AbstractValue& operator[](size_t index) = 0;

    virtual const SimTK::AbstractValue& operator[](size_t index) const = 0;
};


template<typename T>
class ValueArray : public AbstractValueArray {
public:
    template<typename U>
    using Value = SimTK::Value<U>;

    ValueArray* clone() const override {
        auto copy = new ValueArray{};
        copy->_values = _values;
        return copy;
    }

    Value<T>& operator[](size_t index) override {
        return _values[index];
    } 

    const Value<T>& operator[](size_t index) const override {
        return _values[index];
    }

    const std::vector<Value<T>>& get() const {
        return _values;
    }

    std::vector<Value<T>>& upd() {
        return _values;
    }

private:
    std::vector<Value<T>> _values;
};

}

#endif // OPENSIM_VALUEARRAY_H_
