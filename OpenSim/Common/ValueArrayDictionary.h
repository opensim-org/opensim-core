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

#ifndef OPENSIM_VALUEARRAYDICTIONARY_H_
#define OPENSIM_VALUEARRAYDICTIONARY_H_

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
    /** Get the array corresponding to a given key.                           */
    const AbstractValueArray& 
    getValueArrayForKey(const std::string& key) const {
        return *_dictionary.at(key);
    }

    /** Set the array corresponding to a given key.                           */
    void setValueArrayForKey(const std::string& key, 
                             const AbstractValueArray& abstractValueArray) {
        using Value = SimTK::ClonePtr<AbstractValueArray>;

        _dictionary.emplace(key, 
                            Value{abstractValueArray.clone()});
    }

    /** Remove a key and its associated array.                                */
    void removeValueArrayForKey(const std::string& key) {
        _dictionary.erase(key);
    }

    /** Get an iterator to the associative array. The returned object is an 
    std::pair where the first element is the iterator representing the
    beginning and second element is the iterator representing the end.        */
    std::pair<Dictionary::const_iterator, Dictionary::const_iterator>
    getKeyValueIterator() const {
        return std::make_pair(_dictionary.cbegin(), _dictionary.cend());
    }

    /** Get all the existing keys as a std::vector.                           */
    std::vector<std::string> getKeys() const {
        std::vector<std::string> keys{};
        std::transform(_dictionary.begin(), 
                       _dictionary.end(),
                       std::back_inserter(keys),
                       [] (const Dictionary::value_type& key_value) {
                           return key_value.first;
                       });

        return keys;
    }
private:
    Dictionary _dictionary;
};

}

#endif // OPENSIM_VALUEARRAYDICTIONARY_H_
