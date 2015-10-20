/* -------------------------------------------------------------------------- *
 *                            OpenSim:  ValueArrayDictionary.h                *
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

#ifndef OPENSIM_VALUE_ARRAY_DICTIONARY_H_
#define OPENSIM_VALUE_ARRAY_DICTIONARY_H_

#include "OpenSim/Common/ValueArray.h"

#include <memory>

namespace OpenSim {

/** ValueArrayDictionary represents an associative array mapping from a string 
to an AbstractValueArray.                                                     */
class ValueArrayDictionary {
private:
    using Dictionary = std::map<std::string, 
                                SimTK::ClonePtr<AbstractValueArray>>;

public:
    using AbstractValue = SimTK::AbstractValue;

    /** Get the first entry of the array corresponding to the given key.      

    \throws KeyNotFound If key is not found.                                  */
    const AbstractValue& getValueForKey(const std::string& key) const {
        throwIfKeyNotFound(key, __func__);
        return (*(_dictionary.at(key)))[0];
    }

    /** Set the value corresponding to a given key.                           */
    template<typename ValueType>
    void setValueForKey(const std::string& key,
                        const ValueType& value) {
        using Value = SimTK::ClonePtr<AbstractValueArray>;

        auto valueArray = new ValueArray<ValueType>{};
        valueArray->upd().push_back(SimTK::Value<ValueType>{value});
        AbstractValueArray* absValueArray{valueArray};

        _dictionary.emplace(key, Value{absValueArray});
    }

    /** Get the array corresponding to a given key.                           

    \throws KeyNotFound If key is not found.                                  */
    const AbstractValueArray& 
    getValueArrayForKey(const std::string& key) const {
        throwIfKeyNotFound(key, __func__);
        return *(_dictionary.at(key));
    }

    /** Set the array corresponding to a given key.                           */
    void setValueArrayForKey(const std::string& key, 
                             const AbstractValueArray& abstractValueArray) {
        using Value = SimTK::ClonePtr<AbstractValueArray>;

        _dictionary.emplace(key, Value{abstractValueArray.clone()});
    }

    /** Remove a key and its associated array.                                */
    void removeValueForKey(const std::string& key) {
        _dictionary.erase(key);
    }

    /** Remove a key and its associated array.                                */
    void removeValueArrayForKey(const std::string& key) {
        _dictionary.erase(key);
    }

    /** Get all the existing keys as a std::vector.                           */
    std::vector<std::string> getKeys() const {
        std::vector<std::string> keys{};
        std::transform(_dictionary.begin(), 
                       _dictionary.end(),
                       std::back_inserter(keys),
                       [] (const Dictionary::value_type& keyValue) {
                           return keyValue.first;
                       });

        return keys;
    }

    /** Get begin iterator to the associative array.                          */
    Dictionary::const_iterator getKeyValueBegin() const {
        return _dictionary.cbegin();
    }

    /** Get begin iterator to the associative array.                          */
    Dictionary::const_iterator getKeyValueEnd() const {
        return _dictionary.cend();
    }
private:
    void throwIfKeyNotFound(const std::string& key,
                            const std::string& func) const {
        if(_dictionary.find(key) == _dictionary.end())
            throw KeyNotFound{__FILE__, __LINE__, func, key};
    }

    Dictionary _dictionary;
};

}

#endif // OPENSIM_VALUE_ARRAY_DICTIONARY_H_
